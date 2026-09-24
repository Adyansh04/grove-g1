/**
 * @file g1_sensor_relay_node.cpp
 * @brief Turns sensor frames sampled inside unitree_mujoco into ROS 2 messages.
 *
 * The simulator samples against its own mjData and sends finished frames over a local socket.
 * It links no ROS: unitree_sdk2 owns CycloneDDS there, and rmw_cyclonedds cannot share it.
 */

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <system_error>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_sensor_relay/frame_reader.hpp"

namespace g1_sensor_relay
{

class SensorRelay : public rclcpp::Node
{
public:
    SensorRelay()
      : rclcpp::Node("g1_sensor_relay")
    {
        socket_path_    = declare_parameter<std::string>("socket_path", "/tmp/g1_sensors.sock");
        frame_id_       = declare_parameter<std::string>("frame_id", "mid360_link");
        world_frame_id_ = declare_parameter<std::string>("world_frame_id", "world");
        const std::string topic = declare_parameter<std::string>("topic", "/livox/lidar");
        // 500 Hz drains a ~2.9 MB depth+colour frame within the sender's retry deadline even
        // without a forced send buffer. An idle poll costs one EAGAIN.
        poll_hz_ = declare_parameter<double>("poll_hz", 500.0);

        // Sensor QoS: only the newest cloud matters. A reliable subscriber receives nothing.
        cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS());

        // REP-145 optical frames, not d435_link: depth consumers assume z forward, and the body
        // frame would rotate the cloud 90 degrees.
        depth_frame_id_ =
            declare_parameter<std::string>("depth_frame_id", "camera_depth_optical_frame");
        color_frame_id_ =
            declare_parameter<std::string>("color_frame_id", "camera_color_optical_frame");
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
            declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw"),
            rclcpp::SensorDataQoS());
        // Same intrinsics, second namespace: rviz's DepthCloud looks for camera_info
        // beside the depth image, and a real D435i with align_depth publishes both.
        depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            declare_parameter<std::string>(
                "depth_info_topic",
                "/camera/aligned_depth_to_color/camera_info"),
            rclcpp::SensorDataQoS());
        color_pub_ = create_publisher<sensor_msgs::msg::Image>(
            declare_parameter<std::string>("color_topic", "/camera/color/image_raw"),
            rclcpp::SensorDataQoS());
        info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            declare_parameter<std::string>("info_topic", "/camera/color/camera_info"),
            rclcpp::SensorDataQoS());

        // A topic, not TF: mid360_link already has a parent. Lets a test check cloud geometry
        // before odom -> pelvis exists.
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/sensor_pose",
            rclcpp::SensorDataQoS());

        // Reliable like the real driver, since FAST-LIO subscribes reliably. Depth 400, not the
        // driver's 10: these 200 Hz samples arrive in bursts behind 2.9 MB depth frames.
        imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "mid360_imu");
        imu_pub_      = create_publisher<sensor_msgs::msg::Imu>(
            declare_parameter<std::string>("imu_topic", "/livox/imu"),
            rclcpp::QoS(400));

        // Raw ground truth in the camera frame; g1_object_pose_source turns it into /objects.
        // Node-relative so nothing subscribes to ground truth by accident.
        objects_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
            "~/object_poses",
            rclcpp::SensorDataQoS());

        // Exact pelvis state for the odometry publisher's ground-truth source. Not /odom: it is
        // truth, and nothing on the robot publishes it.
        base_frame_id_   = declare_parameter<std::string>("base_state_frame_id", "pelvis");
        base_odom_frame_ = declare_parameter<std::string>("base_state_odom_frame", "odom");
        base_state_pub_ =
            create_publisher<nav_msgs::msg::Odometry>("~/base_state", rclcpp::QoS(50));

        if (!openListener())
        {
            throw std::runtime_error("could not open " + socket_path_);
        }

        // Polled on one thread: a nonblocking accept plus a read per tick.
        timer_ =
            create_wall_timer(std::chrono::duration<double>(1.0 / poll_hz_), [this]() { poll(); });

        RCLCPP_INFO(
            get_logger(),
            "Listening on %s, publishing %s in frame %s",
            socket_path_.c_str(),
            topic.c_str(),
            frame_id_.c_str());
    }

    ~SensorRelay() override
    {
        closeClient();
        if (listen_fd_ >= 0)
        {
            ::close(listen_fd_);
        }
        // The simulator reconnects by path, so a stale node must not leave one behind.
        ::unlink(socket_path_.c_str());
    }

private:
    /// Thread-safe, unlike std::strerror().
    static std::string lastErrorMessage() { return std::system_category().message(errno); }

    bool openListener()
    {
        // A socket file left by a crashed run makes bind() fail with EADDRINUSE.
        ::unlink(socket_path_.c_str());

        listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
        if (listen_fd_ < 0)
        {
            RCLCPP_ERROR(get_logger(), "socket(): %s", lastErrorMessage().c_str());
            return false;
        }
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "bind(%s): %s",
                socket_path_.c_str(),
                lastErrorMessage().c_str());
            return false;
        }
        if (::listen(listen_fd_, 1) != 0)
        {
            RCLCPP_ERROR(get_logger(), "listen(): %s", lastErrorMessage().c_str());
            return false;
        }
        return true;
    }

    void closeClient()
    {
        if (client_fd_ >= 0)
        {
            ::close(client_fd_);
            client_fd_ = -1;
        }
        buffer_.clear();
        // The clock offset belongs to the connection: after a simulator restart sim_time is near
        // zero, and the old running minimum would pin every new stamp near the Unix epoch.
        have_clock_offset_ = false;
    }

    void poll()
    {
        if (client_fd_ < 0)
        {
            const int fd = ::accept4(listen_fd_, nullptr, nullptr, SOCK_NONBLOCK);
            if (fd < 0)
            {
                return;
            }
            client_fd_ = fd;
            RCLCPP_INFO(get_logger(), "Simulator connected.");
        }

        // Drain fully, then publish every complete frame: after a hiccup several can be queued.
        std::array<std::uint8_t, 65536> chunk;
        for (;;)
        {
            const ssize_t n = ::recv(client_fd_, chunk.data(), chunk.size(), MSG_DONTWAIT);
            if (n > 0)
            {
                buffer_.insert(buffer_.end(), chunk.data(), chunk.data() + n);
                continue;
            }
            if (n == 0)
            {
                RCLCPP_INFO(get_logger(), "Simulator disconnected.");
                closeClient();
                return;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }
            RCLCPP_WARN(get_logger(), "recv(): %s", lastErrorMessage().c_str());
            closeClient();
            return;
        }

        // Hoisted so a burst reuses one frame's capacity; tryReadFrame fills by resize().
        CloudFrame frame;
        for (;;)
        {
            const FrameStatus status = tryReadFrame(buffer_, frame);
            if (status == FrameStatus::kIncomplete)
            {
                return;
            }
            if (status != FrameStatus::kOk)
            {
                // A desynchronised stream cannot be realigned; guessing publishes nonsense.
                RCLCPP_ERROR(get_logger(), "Dropping connection: %s", toString(status));
                closeClient();
                return;
            }
            switch (frame.kind)
            {
                case FrameKind::kDepth:
                    publishDepth(frame);
                    break;
                case FrameKind::kObjectPoses:
                    publishObjects(frame);
                    break;
                case FrameKind::kPointCloud:
                    publish(frame);
                    break;
                case FrameKind::kImu:
                    publishImu(frame);
                    break;
                case FrameKind::kBaseState:
                    publishBaseState(frame);
                    break;
            }
        }
    }

    /// The Mid360's IMU, stamped through the same clock mapping as the sweep it is fused with.
    void publishImu(const CloudFrame& frame)
    {
        auto imu             = std::make_unique<sensor_msgs::msg::Imu>();
        imu->header.stamp    = stampFor(frame.sim_time_s);
        imu->header.frame_id = imu_frame_id_;

        // MuJoCo's framequat is wxyz.
        imu->orientation.w = frame.sensor_quat[0];
        imu->orientation.x = frame.sensor_quat[1];
        imu->orientation.y = frame.sensor_quat[2];
        imu->orientation.z = frame.sensor_quat[3];

        imu->angular_velocity.x = frame.imu.gyro[0];
        imu->angular_velocity.y = frame.imu.gyro[1];
        imu->angular_velocity.z = frame.imu.gyro[2];

        // Proper acceleration, gravity included, as a real IMU reads. FAST-LIO rescales by the
        // magnitude at init, so units only need to be consistent.
        imu->linear_acceleration.x = frame.imu.acc[0];
        imu->linear_acceleration.y = frame.imu.acc[1];
        imu->linear_acceleration.z = frame.imu.acc[2];

        imu_pub_->publish(std::move(imu));
    }

    /// The simulator's exact pelvis pose and twist, the twist body-frame as nav_msgs expects.
    void publishBaseState(const CloudFrame& frame)
    {
        auto odom             = std::make_unique<nav_msgs::msg::Odometry>();
        odom->header.stamp    = stampFor(frame.sim_time_s);
        odom->header.frame_id = base_odom_frame_;
        odom->child_frame_id  = base_frame_id_;

        odom->pose.pose.position.x = frame.sensor_pos[0];
        odom->pose.pose.position.y = frame.sensor_pos[1];
        odom->pose.pose.position.z = frame.sensor_pos[2];

        // MuJoCo's framequat is wxyz.
        odom->pose.pose.orientation.w = frame.sensor_quat[0];
        odom->pose.pose.orientation.x = frame.sensor_quat[1];
        odom->pose.pose.orientation.y = frame.sensor_quat[2];
        odom->pose.pose.orientation.z = frame.sensor_quat[3];

        odom->twist.twist.linear.x  = frame.base.lin_vel[0];
        odom->twist.twist.linear.y  = frame.base.lin_vel[1];
        odom->twist.twist.linear.z  = frame.base.lin_vel[2];
        odom->twist.twist.angular.x = frame.base.ang_vel[0];
        odom->twist.twist.angular.y = frame.base.ang_vel[1];
        odom->twist.twist.angular.z = frame.base.ang_vel[2];

        // Zero covariance: exact state, not something to fuse.
        base_state_pub_->publish(std::move(odom));
    }

    /// Ground truth as the camera would see it, converted sim-side so g1_object_pose_source
    /// runs the same code on the robot.
    void publishObjects(const CloudFrame& frame)
    {
        geometry_msgs::msg::TransformStamped world_to_camera;
        if (!worldToCamera(world_to_camera))
        {
            return;
        }

        auto msg             = std::make_unique<vision_msgs::msg::Detection3DArray>();
        msg->header.stamp    = now();
        msg->header.frame_id = color_frame_id_;
        msg->detections.reserve(frame.objects.size());

        for (const grove_g1::ObjectPoseRecord& record : frame.objects)
        {
            vision_msgs::msg::Detection3D detection;
            detection.header = msg->header;
            detection.id     = record.name;

            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = record.name;
            // Ground truth; a real detector fills in its own confidence.
            hypothesis.hypothesis.score = 1.0;
            geometry_msgs::msg::Pose in_world;
            in_world.position.x    = record.pos[0];
            in_world.position.y    = record.pos[1];
            in_world.position.z    = record.pos[2];
            in_world.orientation.w = record.quat[0];
            in_world.orientation.x = record.quat[1];
            in_world.orientation.y = record.quat[2];
            in_world.orientation.z = record.quat[3];
            tf2::doTransform(in_world, hypothesis.pose.pose, world_to_camera);

            detection.bbox.center = hypothesis.pose.pose;
            // Full widths, as BoundingBox3D means size; consumers build collision geometry
            // from it.
            detection.bbox.size.x = record.size[0];
            detection.bbox.size.y = record.size[1];
            detection.bbox.size.z = record.size[2];
            detection.results.push_back(hypothesis);
            msg->detections.push_back(std::move(detection));
        }
        objects_pub_->publish(std::move(msg));
    }

    /// camera_T_world from the LiDAR's ground-truth pose and the URDF's LiDAR-to-camera
    /// transform, one sweep stale. False until the first sweep and the robot's TF have arrived.
    bool worldToCamera(geometry_msgs::msg::TransformStamped& out)
    {
        if (!sensor_in_world_)
        {
            return false;
        }
        geometry_msgs::msg::TransformStamped sensor_to_camera;
        try
        {
            sensor_to_camera =
                tf_buffer_.lookupTransform(color_frame_id_, frame_id_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "No %s -> %s yet: %s",
                frame_id_.c_str(),
                color_frame_id_.c_str(),
                ex.what());
            return false;
        }

        // sensor_in_world_ maps sensor points into world; its inverse makes camera_T_world.
        tf2::Transform sensor_to_world;
        tf2::fromMsg(*sensor_in_world_, sensor_to_world);
        tf2::Transform to_camera;
        tf2::fromMsg(sensor_to_camera.transform, to_camera);
        out.transform = tf2::toMsg(to_camera * sensor_to_world.inverse());
        return true;
    }

    void publishDepth(const CloudFrame& frame)
    {
        auto img             = std::make_unique<sensor_msgs::msg::Image>();
        img->header.stamp    = now();
        img->header.frame_id = depth_frame_id_;
        img->height          = frame.height;
        img->width           = frame.width;
        // 32FC1 metres, already linearised from MuJoCo's depth buffer by the simulator.
        img->encoding     = "32FC1";
        img->is_bigendian = 0;
        img->step         = frame.width * sizeof(float);
        img->data.resize(frame.depth.size() * sizeof(float));
        std::memcpy(img->data.data(), frame.depth.data(), img->data.size());

        sensor_msgs::msg::CameraInfo info;
        info.header           = img->header;
        info.height           = frame.height;
        info.width            = frame.width;
        info.distortion_model = "plumb_bob";
        info.d.assign(5, 0.0);
        // MuJoCo's fovy is vertical; carried in the frame so camera_info matches the render.
        const double f  = frame.height / (2.0 * std::tan(frame.fovy_deg * M_PI / 180.0 / 2.0));
        const double cx = frame.width / 2.0;
        const double cy = frame.height / 2.0;
        info.k          = { f, 0, cx, 0, f, cy, 0, 0, 1 };
        info.r          = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        info.p          = { f, 0, cx, 0, 0, f, cy, 0, 0, 0, 1, 0 };

        // Captured before publish: img is a null pointer afterwards, only ownership moves.
        const auto render_stamp = img->header.stamp;

        depth_pub_->publish(std::move(img));
        depth_info_pub_->publish(info);

        // Same render, so the colour stream shares the depth intrinsics exactly; a real
        // D435i only gets that from its align_depth_to_color step.
        info.header.frame_id = color_frame_id_;
        info_pub_->publish(info);

        if (!frame.rgb.empty())
        {
            auto color             = std::make_unique<sensor_msgs::msg::Image>();
            color->header.stamp    = render_stamp;
            color->header.frame_id = color_frame_id_;
            color->height          = frame.height;
            color->width           = frame.width;
            color->encoding        = "rgb8";
            color->is_bigendian    = 0;
            color->step            = frame.width * 3;
            color->data.assign(frame.rgb.begin(), frame.rgb.end());
            color_pub_->publish(std::move(color));
        }
    }

    /**
     * @brief The wall-clock stamp for a frame captured at @p sim_time_s.
     *
     * A sweep arrives ~35 ms after capture, long enough for the gait to tilt the pose. The clock
     * offset is the smallest arrival lag seen, leaking upward slowly to follow clock drift.
     */
    rclcpp::Time stampFor(double sim_time_s)
    {
        const rclcpp::Time arrival = now();
        // A NaN would latch clock_offset_ for good, since every comparison against it is false.
        if (!std::isfinite(sim_time_s))
        {
            return arrival;
        }
        const double delta = arrival.seconds() - sim_time_s;

        if (!have_clock_offset_ || delta < clock_offset_)
        {
            clock_offset_      = delta;
            have_clock_offset_ = true;
        }
        else
        {
            // ~20 ms/s at the 200 Hz IMU rate: follows clock drift, far slower than the latency.
            // IMU frames arrive within microseconds, so they pin the offset near the true one.
            clock_offset_ += 1.0e-4;
        }

        const double stamped = sim_time_s + clock_offset_;
        // Never stamp after arrival: tf2 refuses future stamps.
        if (stamped >= arrival.seconds())
        {
            return arrival;
        }
        return rclcpp::Time(static_cast<std::int64_t>(stamped * 1.0e9), arrival.get_clock_type());
    }

    void publish(const CloudFrame& frame)
    {
        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        // The capture instant mapped onto this node's clock, not arrival. See stampFor().
        msg->header.stamp    = stampFor(frame.sim_time_s);
        msg->header.frame_id = frame_id_;

        const std::size_t points = frame.points.size() / 3;
        msg->height              = 1;
        msg->width               = static_cast<std::uint32_t>(points);
        msg->is_bigendian        = false;
        msg->is_dense            = false;
        msg->point_step          = 12;
        msg->row_step            = msg->point_step * msg->width;

        msg->fields.resize(3);
        const std::array<const char*, 3> names = { "x", "y", "z" };
        for (int i = 0; i < 3; ++i)
        {
            msg->fields[i].name     = names[i];
            msg->fields[i].offset   = static_cast<std::uint32_t>(i * 4);
            msg->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg->fields[i].count    = 1;
        }

        msg->data.resize(frame.points.size() * sizeof(float));
        std::memcpy(msg->data.data(), frame.points.data(), msg->data.size());

        geometry_msgs::msg::PoseStamped pose;
        pose.header             = msg->header;
        pose.header.frame_id    = world_frame_id_;
        pose.pose.position.x    = frame.sensor_pos[0];
        pose.pose.position.y    = frame.sensor_pos[1];
        pose.pose.position.z    = frame.sensor_pos[2];
        pose.pose.orientation.w = frame.sensor_quat[0];
        pose.pose.orientation.x = frame.sensor_quat[1];
        pose.pose.orientation.y = frame.sensor_quat[2];
        pose.pose.orientation.z = frame.sensor_quat[3];
        pose_pub_->publish(pose);

        // publishObjects needs the sensor's world pose; object frames carry none of their own.
        sensor_in_world_ = pose.pose;

        cloud_pub_->publish(std::move(msg));
    }

    std::optional<geometry_msgs::msg::Pose> sensor_in_world_;
    tf2_ros::Buffer                         tf_buffer_{ get_clock() };
    tf2_ros::TransformListener              tf_listener_{ tf_buffer_ };

    std::string socket_path_;
    std::string frame_id_;
    std::string world_frame_id_;
    std::string imu_frame_id_;
    std::string depth_frame_id_;
    std::string color_frame_id_;
    std::string base_frame_id_;
    std::string base_odom_frame_;
    double      poll_hz_ = 500.0;

    /// Estimated offset from the simulator's clock to this node's, in seconds. See stampFor().
    double clock_offset_      = 0.0;
    bool   have_clock_offset_ = false;

    int                       listen_fd_ = -1;
    int                       client_fd_ = -1;
    std::vector<std::uint8_t> buffer_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr      cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr            depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr            color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr       info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr       depth_info_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr objects_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            base_state_pub_;
    rclcpp::TimerBase::SharedPtr                                     timer_;
};

}  // namespace g1_sensor_relay

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_sensor_relay::SensorRelay>());
    rclcpp::shutdown();
    return 0;
}
