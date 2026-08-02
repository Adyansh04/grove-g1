/**
 * @file g1_loco_bridge_main.cpp
 * @brief Entry point -- one SingleThreadedExecutor, per the node's thread-ownership contract.
 */
#include <rclcpp/rclcpp.hpp>

#include "g1_locomotion/g1_loco_bridge_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // Load-bearing, not a default left alone: G1LocoBridge's thread-ownership contract (see its
    // class comment) only holds because this is single-threaded -- the base class's own lifecycle
    // and parameter services live outside callback_group_ and have no other way to be kept out
    // from under a concurrently running timer/subscription callback.
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<g1_locomotion::G1LocoBridge>(rclcpp::NodeOptions());
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
