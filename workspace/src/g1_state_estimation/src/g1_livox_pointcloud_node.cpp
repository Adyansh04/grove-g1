/**
 * @file g1_livox_pointcloud_node.cpp
 * @brief Republishes the Livox CustomMsg as a PointCloud2, so both formats exist at once.
 *
 * The driver emits one format per run, and the consumers disagree. FAST-LIO needs CustomMsg,
 * the only format carrying the per-point timestamps a scan taken while walking is undistorted
 * with; everything else reads the PointCloud2 on /livox/lidar. So the driver runs in CustomMsg
 * mode and this fills the gap.
 *
 * Downstream of the driver rather than folded into it, so the cloud everything else depends on
 * cannot be taken down by FAST-LIO.
 *
 * Hardware only. In simulation the relay publishes /livox/lidar and g1_livox_bridge runs this
 * conversion in the other direction.
 */

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <utility>

#include "g1_state_estimation/livox_cloud.hpp"

namespace g1_state_estimation
{

class LivoxPointCloud : public rclcpp::Node
{
public:
    LivoxPointCloud()
      : rclcpp::Node("g1_livox_pointcloud")
    {
        // Sensor QoS out, matching what g1_sensor_relay publishes on this topic in simulation
        // so the consumers cannot tell the two apart.
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            declare_parameter<std::string>("cloud_topic", "/livox/lidar"),
            rclcpp::SensorDataQoS());
        // Reliable in, because that is what the driver offers (lddc.cpp CreatePublisher takes
        // a bare queue size). A best-effort subscriber would match, but would also drop scans
        // under load for no reason.
        custom_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            declare_parameter<std::string>("custom_msg_topic", "/livox/custom_msg"),
            rclcpp::QoS(20),
            [this](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg) {
                onCustom(*msg);
            });
    }

private:
    void onCustom(const livox_ros_driver2::msg::CustomMsg& custom)
    {
        auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
        toPointCloud2(custom, *cloud);
        cloud_pub_->publish(std::move(cloud));
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        cloud_pub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_sub_;
};

}  // namespace g1_state_estimation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_state_estimation::LivoxPointCloud>());
    rclcpp::shutdown();
    return 0;
}
