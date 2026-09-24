/**
 * @file g1_livox_pointcloud_node.cpp
 * @brief Republishes the Livox CustomMsg as PointCloud2 on /livox/lidar. Hardware only.
 *
 * The driver emits one format per run and FAST-LIO needs CustomMsg for its per-point
 * timestamps. In simulation g1_livox_bridge converts the other way.
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
        // Sensor QoS, as g1_sensor_relay publishes this topic in simulation.
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            declare_parameter<std::string>("cloud_topic", "/livox/lidar"),
            rclcpp::SensorDataQoS());
        // Reliable, as the driver publishes (lddc.cpp passes a bare queue size).
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
