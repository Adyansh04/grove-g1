/**
 * @file g1_loco_bridge_main.cpp
 * @brief Entry point -- one SingleThreadedExecutor, per the node's thread-ownership contract.
 */
#include <rclcpp/rclcpp.hpp>

#include "g1_locomotion/g1_loco_bridge_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<g1_locomotion::G1LocoBridge>(rclcpp::NodeOptions());
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
