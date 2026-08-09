/**
 * @file g1_livox_bridge_node.cpp
 * @brief Makes the simulator's sensors look like a Livox Mid360 to FAST-LIO.
 *
 * FAST-LIO does not consume what the rest of the stack consumes. It wants the Livox CustomMsg,
 * because that is the only format carrying a per-point timestamp, and it wants an IMU stream
 * beside it. On the robot both come out of livox_ros_driver2. In simulation there is no driver:
 * the sweep arrives as a PointCloud2 from g1_sensor_relay and the IMU lives inside
 * unitree_hg::LowState. This node restates both in the driver's own formats so the odometry
 * pipeline below it is identical in either place.
 *
 * SIMULATION ONLY. On hardware the real driver publishes these topics and this node must not
 * run -- two publishers on /livox/custom_msg would interleave scans from different sources.
 */

#include <chrono>
#include <cmath>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <unitree_hg/msg/low_state.hpp>
#include <utility>

namespace g1_sensor_relay
{

class LivoxBridge : public rclcpp::Node
{
public:
    LivoxBridge()
      : rclcpp::Node("g1_livox_bridge")
    {
        // RELIABLE, with the same depths livox_ros_driver2 uses (lddc.cpp CreatePublisher
        // passes a bare queue size, which is reliable by default). Not a style choice:
        // FAST-LIO subscribes to both of these reliably, and a best-effort publisher is
        // silently incompatible with that -- DDS drops the match and logs one warning about
        // RELIABILITY_QOS_POLICY that is easy to read past.
        custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
            declare_parameter<std::string>("custom_msg_topic", "/livox/custom_msg"),
            rclcpp::QoS(20));
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            declare_parameter<std::string>("imu_topic", "/livox/imu"),
            rclcpp::QoS(10));

        // The inputs keep sensor QoS: these are the simulator's own streams, published
        // best-effort by g1_sensor_relay and the DDS bridge.
        const auto sensor_qos = rclcpp::SensorDataQoS();
        cloud_sub_            = create_subscription<sensor_msgs::msg::PointCloud2>(
            declare_parameter<std::string>("cloud_topic", "/livox/lidar"),
            sensor_qos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(*msg); });
        low_state_sub_ = create_subscription<unitree_hg::msg::LowState>(
            declare_parameter<std::string>("low_state_topic", "/lowstate"),
            sensor_qos,
            [this](unitree_hg::msg::LowState::ConstSharedPtr msg) { onLowState(*msg); });

        imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "pelvis");
    }

private:
    void onCloud(const sensor_msgs::msg::PointCloud2& cloud)
    {
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");

        livox_ros_driver2::msg::CustomMsg msg;
        msg.header = cloud.header;
        // Informational only -- FAST-LIO times scans off header.stamp -- but a real driver puts
        // the first point's absolute time here, so match that rather than leave it zero.
        msg.timebase = static_cast<std::uint64_t>(rclcpp::Time(cloud.header.stamp).nanoseconds());
        msg.lidar_id = 0;
        msg.points.reserve(cloud.width * cloud.height);

        for (; x != x.end(); ++x, ++y, ++z)
        {
            // Misses come through as non-finite. Dropping them here rather than passing them on
            // keeps point_num honest, and FAST-LIO's own range gate would discard them anyway.
            if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
            {
                continue;
            }
            livox_ros_driver2::msg::CustomPoint point;
            point.x = *x;
            point.y = *y;
            point.z = *z;
            // Zero, and correct rather than merely convenient. The simulator raycasts against a
            // frozen mjData, so every point in a frame really is sampled at the same instant.
            // FAST-LIO reads this as milliseconds-since-scan-start into its motion
            // undistortion, which then finds nothing to undo -- which is the truth here.
            point.offset_time = 0;
            // Both are gates in FAST-LIO's Livox handler, not decoration: `line` must be under
            // scan_line, and tag bits 4-5 must read 00 or 01 or the point is discarded.
            point.line = 0;
            point.tag  = 0;
            // The sweep carries no intensity, and nothing downstream registers on it.
            point.reflectivity = 0;
            msg.points.push_back(point);
        }

        msg.point_num = static_cast<std::uint32_t>(msg.points.size());
        custom_pub_->publish(std::move(msg));
    }

    void onLowState(const unitree_hg::msg::LowState& state)
    {
        sensor_msgs::msg::Imu imu;
        // The simulator publishes no /clock and stamps nothing itself, so arrival time is the
        // only stamp available -- the same choice g1_sensor_relay makes for the sweep, which is
        // what keeps the two streams comparable.
        imu.header.stamp    = now();
        imu.header.frame_id = imu_frame_id_;

        // Unitree order the quaternion w-first.
        imu.orientation.w = state.imu_state.quaternion[0];
        imu.orientation.x = state.imu_state.quaternion[1];
        imu.orientation.y = state.imu_state.quaternion[2];
        imu.orientation.z = state.imu_state.quaternion[3];

        imu.angular_velocity.x = state.imu_state.gyroscope[0];
        imu.angular_velocity.y = state.imu_state.gyroscope[1];
        imu.angular_velocity.z = state.imu_state.gyroscope[2];

        // Proper acceleration, gravity included, which is what MuJoCo's accelerometer sensor
        // reports and what a real IMU reads. FAST-LIO normalises by the measured magnitude
        // during its init, so the units only have to be self-consistent.
        imu.linear_acceleration.x = state.imu_state.accelerometer[0];
        imu.linear_acceleration.y = state.imu_state.accelerometer[1];
        imu.linear_acceleration.z = state.imu_state.accelerometer[2];

        imu_pub_->publish(std::move(imu));
    }

    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             imu_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_sub_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr      low_state_sub_;
    std::string                                                     imu_frame_id_;
};

}  // namespace g1_sensor_relay

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_sensor_relay::LivoxBridge>());
    rclcpp::shutdown();
    return 0;
}
