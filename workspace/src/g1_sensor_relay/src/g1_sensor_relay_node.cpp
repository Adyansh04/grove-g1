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
#include <cstring>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

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
        poll_hz_                = declare_parameter<double>("poll_hz", 200.0);

        // Sensor QoS: only the newest cloud matters, and a reliable publisher against a
        // best-effort subscriber is the usual reason nothing shows up in rviz.
        cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS());

        // The simulator knows the sensor's world pose exactly and already sends it in the
        // frame header. Published as plain diagnostic data rather than TF: mid360_link
        // already has a parent through robot_state_publisher, and a second one would make
        // the tree ambiguous. It is what lets a test check cloud geometry against the room
        // before odom -> pelvis exists.
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/sensor_pose",
            rclcpp::SensorDataQoS());

        if (!openListener())
        {
            throw std::runtime_error("could not open " + socket_path_);
        }

        // Polled rather than event-driven on purpose: one node, one thread, no executor
        // surprises, and the cost is a nonblocking accept plus a read at 200 Hz.
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
        // fully matters: at 200 Hz polling against 10 Hz frames the socket is usually
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

        for (;;)
        {
            CloudFrame        frame;
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
            publish(frame);
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
    double      poll_hz_ = 200.0;

    int                       listen_fd_ = -1;
    int                       client_fd_ = -1;
    std::vector<std::uint8_t> buffer_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
};

}  // namespace g1_sensor_relay

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_sensor_relay::SensorRelay>());
    rclcpp::shutdown();
    return 0;
}
