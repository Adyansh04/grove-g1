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
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
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
    bool openListener()
    {
        // A leftover socket file from a crashed run makes bind() fail with EADDRINUSE, and
        // the launch would look broken for a reason that has nothing to do with this run.
        ::unlink(socket_path_.c_str());

        listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
        if (listen_fd_ < 0)
        {
            RCLCPP_ERROR(get_logger(), "socket(): %s", std::strerror(errno));
            return false;
        }
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
        {
            RCLCPP_ERROR(get_logger(), "bind(%s): %s", socket_path_.c_str(), std::strerror(errno));
            return false;
        }
        if (::listen(listen_fd_, 1) != 0)
        {
            RCLCPP_ERROR(get_logger(), "listen(): %s", std::strerror(errno));
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
        std::uint8_t chunk[65536];
        for (;;)
        {
            const ssize_t n = ::recv(client_fd_, chunk, sizeof(chunk), MSG_DONTWAIT);
            if (n > 0)
            {
                buffer_.insert(buffer_.end(), chunk, chunk + n);
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
            RCLCPP_WARN(get_logger(), "recv(): %s", std::strerror(errno));
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
            if (status == FrameStatus::Incomplete)
            {
                return;
            }
            if (status != FrameStatus::Ok)
            {
                // Unrecoverable by design: a desynchronised stream cannot be realigned, and
                // guessing would publish plausible-looking nonsense.
                RCLCPP_ERROR(get_logger(), "Dropping connection: %s", toString(status));
                closeClient();
                return;
            }
            switch (frame.kind)
            {
                case FrameKind::Depth:
                    publishDepth(frame);
                    break;
                case FrameKind::ObjectPoses:
                    publishObjects(frame);
                    break;
                case FrameKind::PointCloud:
                    publish(frame);
                    break;
            }
        }
    }

    /// Ground truth, republished verbatim in the simulator's world frame. This node does no
    /// interpreting: g1_object_pose_source is the boundary that decides whether a consumer is
    /// allowed to believe any of it, and on hardware that node refuses to run at all.
    void publishObjects(const CloudFrame& frame)
    {
        vision_msgs::msg::Detection3DArray msg;
        msg.header.stamp    = now();
        msg.header.frame_id = world_frame_id_;
        msg.detections.reserve(frame.objects.size());

        for (const grove_g1::ObjectPoseRecord& record : frame.objects)
        {
            vision_msgs::msg::Detection3D detection;
            detection.header = msg.header;
            detection.id     = record.name;

            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = record.name;
            // Ground truth: there is nothing to be uncertain about. A real detector fills
            // this with its own confidence and the consumer can threshold on it.
            hypothesis.hypothesis.score        = 1.0;
            hypothesis.pose.pose.position.x    = record.pos[0];
            hypothesis.pose.pose.position.y    = record.pos[1];
            hypothesis.pose.pose.position.z    = record.pos[2];
            hypothesis.pose.pose.orientation.w = record.quat[0];
            hypothesis.pose.pose.orientation.x = record.quat[1];
            hypothesis.pose.pose.orientation.y = record.quat[2];
            hypothesis.pose.pose.orientation.z = record.quat[3];

            detection.bbox.center = hypothesis.pose.pose;
            detection.results.push_back(hypothesis);
            msg.detections.push_back(std::move(detection));
        }
        objects_pub_->publish(std::move(msg));
    }

    void publishDepth(const CloudFrame& frame)
    {
        sensor_msgs::msg::Image img;
        img.header.stamp    = now();
        img.header.frame_id = depth_frame_id_;
        img.height          = frame.height;
        img.width           = frame.width;
        // 32FC1 metres. The simulator linearises MuJoCo's non-linear depth buffer before
        // sending, so nothing downstream has to know about znear/zfar.
        img.encoding     = "32FC1";
        img.is_bigendian = 0;
        img.step         = frame.width * sizeof(float);
        img.data.resize(frame.depth.size() * sizeof(float));
        std::memcpy(img.data.data(), frame.depth.data(), img.data.size());

        sensor_msgs::msg::CameraInfo info;
        info.header           = img.header;
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

        // Captured before the move: the colour image below is stamped from it, and reading it
        // back off a moved-from message would put the shared-timestamp guarantee at the mercy
        // of how Image happens to move its header.
        const auto render_stamp = img.header.stamp;

        depth_pub_->publish(std::move(img));
        depth_info_pub_->publish(info);

        // Same render, so the colour stream shares the depth intrinsics exactly; a real
        // D435i only gets that from its align_depth_to_color step.
        info.header.frame_id = color_frame_id_;
        info_pub_->publish(info);

        if (!frame.rgb.empty())
        {
            sensor_msgs::msg::Image color;
            color.header.stamp    = render_stamp;
            color.header.frame_id = color_frame_id_;
            color.height          = frame.height;
            color.width           = frame.width;
            color.encoding        = "rgb8";
            color.is_bigendian    = 0;
            color.step            = frame.width * 3;
            color.data.assign(frame.rgb.begin(), frame.rgb.end());
            color_pub_->publish(std::move(color));
        }
    }

    void publish(const CloudFrame& frame)
    {
        sensor_msgs::msg::PointCloud2 msg;
        // now(), not the simulator's internal time. unitree_mujoco publishes no /clock, so
        // everything else on this track (TF included) is stamped with wall time; stamping
        // clouds with sim-seconds-since-start put them decades in the past and no consumer
        // could transform them. now() also stays correct if a /clock ever appears, because
        // it follows this node's use_sim_time.
        msg.header.stamp    = now();
        msg.header.frame_id = frame_id_;

        const std::size_t points = frame.points.size() / 3;
        msg.height               = 1;
        msg.width                = static_cast<std::uint32_t>(points);
        msg.is_bigendian         = false;
        msg.is_dense             = false;
        msg.point_step           = 12;
        msg.row_step             = msg.point_step * msg.width;

        msg.fields.resize(3);
        const char* names[3] = { "x", "y", "z" };
        for (int i = 0; i < 3; ++i)
        {
            msg.fields[i].name     = names[i];
            msg.fields[i].offset   = static_cast<std::uint32_t>(i * 4);
            msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg.fields[i].count    = 1;
        }

        msg.data.resize(frame.points.size() * sizeof(float));
        std::memcpy(msg.data.data(), frame.points.data(), msg.data.size());

        geometry_msgs::msg::PoseStamped pose;
        pose.header             = msg.header;
        pose.header.frame_id    = world_frame_id_;
        pose.pose.position.x    = frame.sensor_pos[0];
        pose.pose.position.y    = frame.sensor_pos[1];
        pose.pose.position.z    = frame.sensor_pos[2];
        pose.pose.orientation.w = frame.sensor_quat[0];
        pose.pose.orientation.x = frame.sensor_quat[1];
        pose.pose.orientation.y = frame.sensor_quat[2];
        pose.pose.orientation.z = frame.sensor_quat[3];
        pose_pub_->publish(pose);

        cloud_pub_->publish(std::move(msg));
    }

    std::string socket_path_;
    std::string frame_id_;
    std::string world_frame_id_;
    std::string depth_frame_id_;
    std::string color_frame_id_;
    double      poll_hz_ = 500.0;

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
