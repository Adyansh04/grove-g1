/**
 * @file g1_vla_server_main.cpp
 * @brief Entry point for the grasp skill.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "g1_vla/g1_vla_server_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Multi-threaded, and spinning BEFORE initialize(): the robot model arrives on this node's
    // own callbacks, and goals then wait on service futures without spinning.
    auto node = std::make_shared<g1_vla::G1VlaServer>(rclcpp::NodeOptions());
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor] { executor.spin(); });

    node->initialize();
    spinner.join();

    rclcpp::shutdown();
    return 0;
}
