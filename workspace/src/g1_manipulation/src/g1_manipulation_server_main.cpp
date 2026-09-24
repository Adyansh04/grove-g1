/**
 * @file g1_manipulation_server_main.cpp
 * @brief Entry point for the pick/place skills.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "g1_manipulation/g1_manipulation_server_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Spinning before initialize(): MoveGroupInterface blocks on the robot description and the
    // current state, which arrive on this node's own callbacks.
    auto node = std::make_shared<g1_manipulation::G1ManipulationServer>(rclcpp::NodeOptions());
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor] { executor.spin(); });

    node->initialize();
    spinner.join();

    rclcpp::shutdown();
    return 0;
}
