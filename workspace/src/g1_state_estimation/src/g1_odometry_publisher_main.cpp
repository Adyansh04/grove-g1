/**
 * @file g1_odometry_publisher_main.cpp
 * @brief Entry point for the odom -> base_link publisher.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "g1_state_estimation/g1_odometry_publisher_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<g1_state_estimation::G1OdometryPublisher>(rclcpp::NodeOptions());
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
