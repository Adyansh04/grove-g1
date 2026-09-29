/**
 * @file g1_base_approach_main.cpp
 * @brief Entry point for the g1_base_approach node.
 */

#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "g1_locomotion/g1_base_approach_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        // Multi-threaded, so /objects, TF and cancels keep flowing while a goal loop runs.
        rclcpp::executors::MultiThreadedExecutor executor;
        auto node = std::make_shared<g1_locomotion::BaseApproachNode>();
        executor.add_node(node);
        executor.spin();
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("g1_base_approach"), "fatal: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
