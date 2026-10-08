/**
 * @file g1_odometry_publisher_main.cpp
 * @brief Entry point for the odom -> base publisher.
 */
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "g1_state_estimation/g1_odometry_publisher_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        auto                                      node =
            std::make_shared<g1_state_estimation::G1OdometryPublisher>(rclcpp::NodeOptions());
        // Brought up here, not by launch events: an inactive state reported before launch had
        // matched the node's event topic was lost, and the node stayed inactive.
        if (node->declare_parameter<bool>("autostart", false) &&
            node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            node->activate();
        }
        executor.add_node(node->get_node_base_interface());
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("g1_odometry_publisher"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
