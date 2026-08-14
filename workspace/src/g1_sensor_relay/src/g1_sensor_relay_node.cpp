/**
 * @file g1_sensor_relay_node.cpp
 * @brief Turns sensor frames sampled inside unitree_mujoco into ROS 2 messages.
 *
 * The simulator computes the sweep against its own mjData, because that is the only place
 * the scene exists, and hands finished frames over a local socket. This node owns the ROS
 * side. The split is forced: unitree_sdk2 already calls dds_create_domain in that process
 * and rmw_cyclonedds does the same unconditionally, so only one of them can live there.
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
        // 500 Hz rather than 200 as margin for the simulator's fallback path: when it cannot
        // force a large send buffer, a ~2.9 MB depth+colour frame arrives about one receive
        // buffer per wakeup, and at 200 Hz the sender hit its retry deadline mid-frame and
        // reset the connection. With the buffer forced the frame lands in one go and the
        // rate does not matter; polling this often costs only a recv that returns EAGAIN.
        poll_hz_ = declare_parameter<double>("poll_hz", 500.0);

        // Sensor QoS: only the newest cloud matters, and a reliable publisher against a
        // best-effort subscriber is the usual reason nothing shows up in rviz.
        cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS());

        // The simulator knows the sensor's world pose exactly and already sends it in the
        // frame header. Published as plain diagnostic data rather than TF: mid360_link
        // already has a parent through robot_state_publisher, and a second one would make
        // the tree ambiguous. It is what lets a test check cloud geometry against the room
        // before odom -> pelvis exists.
        // REP-145 optical frames, not d435_link. Depth consumers assume z forward / x right
        // / y down; handed the body frame they project the cloud rotated 90 degrees.
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

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/sensor_pose",
            rclcpp::SensorDataQoS());

        // RELIABLE like livox_ros_driver2 (lddc.cpp CreatePublisher passes a bare queue size,
        // which is reliable by default): FAST-LIO subscribes reliably and a best-effort
        // publisher against it is silently unmatched. Deeper than the driver's 10, though --
        // the driver streams from its own thread, while this node publishes 200 Hz IMU frames
        // out of the same timer callback that ships a 2.9 MB depth+colour pair, so they arrive
        // in bursts and ten of writer history is 50 ms of them.
        imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "mid360_imu");
        imu_pub_      = create_publisher<sensor_msgs::msg::Imu>(
            declare_parameter<std::string>("imu_topic", "/livox/imu"),
            rclcpp::QoS(400));

        // Node-relative and raw: this is the simulator's world frame with no staleness
        // policy applied. g1_object_pose_source is what turns it into /objects, and naming
        // it apart keeps a consumer from subscribing to ground truth by accident.
        objects_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
            "~/object_poses",
            rclcpp::SensorDataQoS());

        if (!openListener())
        {
            throw std::runtime_error("could not open " + socket_path_);
        }

        // Polled rather than event-driven on purpose: one node, one thread, no executor
        // surprises, and the cost is a nonblocking accept plus a read at 500 Hz.
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
    /// std::strerror() keeps its message in a shared static buffer and is not thread-safe;
    /// system_category().message() allocates a fresh std::string per call instead.
    static std::string lastErrorMessage() { return std::system_category().message(errno); }

    bool openListener()
    {
        // A leftover socket file from a crashed run makes bind() fail with EADDRINUSE, and
        // the launch would look broken for a reason that has nothing to do with this run.
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

        // Drain whatever is available, then publish every complete frame in it. Draining
        // fully matters: at 500 Hz polling against 10 Hz frames the socket is usually
        // empty, but after any hiccup several frames can be queued.
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

        // Hoisted out of the loop deliberately: tryReadFrame fills via resize(), so reusing
        // one frame reuses its capacity while draining a burst. A cloud can reach tens of MB
        // and, as above, several can be queued behind one poll().
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
                // Unrecoverable by design: a desynchronised stream cannot be realigned, and
                // guessing would publish plausible-looking nonsense.
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
            }
        }
    }

    /// The Mid360's own IMU, in the sensor's frame, stamped from the same clock mapping as the
    /// sweep. Both come off one socket from one simulator, so the pair FAST-LIO fuses is
    /// consistent by construction rather than by two nodes agreeing about wall time.
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

        // Proper acceleration, gravity included, which is what MuJoCo's accelerometer sensor
        // reports and what a real IMU reads. FAST-LIO normalises by the measured magnitude
        // during its init, so the units only have to be self-consistent.
        imu->linear_acceleration.x = frame.imu.acc[0];
        imu->linear_acceleration.y = frame.imu.acc[1];
        imu->linear_acceleration.z = frame.imu.acc[2];

        imu_pub_->publish(std::move(imu));
    }

    /// Ground truth, re-expressed as the camera would have measured it. The simulator reports
    /// world poses; a detector reports what it sees from its own lens, and everything
    /// downstream is built for the latter. Doing the conversion here keeps the difference
    /// inside the sim-only boundary, so g1_object_pose_source and the skills below it run the
    /// same code on the robot.
    ///
    /// A world coordinate under a fixed-frame label is only correct while that frame IS the
    /// world, which stops holding the moment odom becomes an estimate rather than ground truth.
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
            // Ground truth: there is nothing to be uncertain about. A real detector fills
            // this with its own confidence and the consumer can threshold on it.
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
            // Full widths, which is what BoundingBox3D means by size. A consumer builds its
            // collision geometry from this rather than from its own table of object
            // dimensions, so replacing this source with a real detector changes nothing
            // downstream.
            detection.bbox.size.x = record.size[0];
            detection.bbox.size.y = record.size[1];
            detection.bbox.size.z = record.size[2];
            detection.results.push_back(hypothesis);
            msg->detections.push_back(std::move(detection));
        }
        objects_pub_->publish(std::move(msg));
    }

    /// Inverse of the camera's world pose, built from the LiDAR's ground-truth world pose and
    /// the rigid LiDAR-to-camera transform out of the URDF. False until the first sweep, and
    /// while TF has not yet published the robot's own links.
    ///
    /// One sweep stale: the simulator sends the object frame just before the sweep it shares a
    /// cycle with, so this is the previous cycle's pose. That is a few centimetres at walking
    /// pace, and it is a truer model of a real detector than an exact answer would be -- a
    /// camera's measurement is always slightly behind the world too.
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

        // sensor_to_world, not the reverse: sensor_in_world_ is the sensor's pose expressed in
        // world, which maps sensor points into world. Inverting it is what makes the product
        // camera_T_world, so the inverse is load-bearing rather than redundant.
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
        // 32FC1 metres. The simulator linearises MuJoCo's non-linear depth buffer before
        // sending, so nothing downstream has to know about znear/zfar.
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
        // fovy is vertical in MuJoCo, and is carried in the frame rather than assumed so
        // camera_info cannot drift from what the render actually used.
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
     * Stamping on arrival is wrong by the whole pipeline latency. The simulator snapshots
     * mjData at one instant, then raycasts for ~32 ms OUTSIDE the lock (holding it across the
     * sweep would stall physics) and ships the frame over the socket, so a cloud stamped on
     * arrival is labelled ~35 ms after the instant its points actually describe. Everything
     * that transforms the cloud then uses a pose from the wrong moment -- the costmap's TF
     * lookup, and FAST-LIO's IMU integration up to the scan time -- and the error is
     * proportional to angular rate. Standing it is invisible; walking, a pelvis swinging ~9
     * degrees per gait cycle turns 35 ms into degrees, and that lands on the floor plane.
     *
     * sim_time_s is MuJoCo's own clock at the snapshot, so the only unknown is the constant
     * offset between that clock and this one. Latency is never negative, so the smallest
     * (arrival - sim_time) yet seen is the best estimate of it. The slow upward leak keeps one
     * early sample from pinning the estimate forever once the two clocks drift apart, which
     * they do whenever the simulator cannot hold real time.
     */
    rclcpp::Time stampFor(double sim_time_s)
    {
        const rclcpp::Time arrival = now();
        const double       delta   = arrival.seconds() - sim_time_s;

        if (!have_clock_offset_ || delta < clock_offset_)
        {
            clock_offset_      = delta;
            have_clock_offset_ = true;
        }
        else
        {
            // Roughly 20 ms per second at the IMU rate this now runs at, which is fast enough
            // to follow a drifting sim clock and still far slower than the latency being
            // removed. The IMU frames also pin the estimate harder than the sweep ever did:
            // they are sampled and sent in microseconds, so their arrival lag is close to the
            // true clock offset, and the sweep gets back-dated by the right amount as a result.
            clock_offset_ += 1.0e-4;
        }

        const double stamped = sim_time_s + clock_offset_;
        // A frame cannot have been captured after it arrived. Clamping keeps a bad offset from
        // putting stamps in the future, where tf2 refuses them outright.
        if (stamped >= arrival.seconds())
        {
            return arrival;
        }
        return rclcpp::Time(static_cast<std::int64_t>(stamped * 1.0e9), arrival.get_clock_type());
    }

    void publish(const CloudFrame& frame)
    {
        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        // The capture instant, mapped onto this node's clock -- NOT arrival. See stampFor().
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

        // The only ground-truth world pose of anything on the robot that reaches this node.
        // publishObjects needs it to work out what the camera would have seen, and the object
        // frame arrives with no sensor pose of its own (sensor_publisher.cc says so).
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
