/**
 * @file world_model_main.cpp
 * @brief Runs the world model node on a single-threaded executor.
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "canopy/world_model_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<canopy::WorldModelNode>();
    rclcpp::spin(node);
    // Destroyed before shutdown so its final save still has a logger.
    node.reset();
    rclcpp::shutdown();
    return 0;
}
