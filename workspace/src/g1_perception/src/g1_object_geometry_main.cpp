/**
 * @file g1_object_geometry_main.cpp
 * @brief Entry point for the node that turns instance masks into object poses.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "g1_perception/g1_object_geometry_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // Held here: an executor keeps only a weak pointer, so a temporary node is gone before spin.
    const auto node = std::make_shared<g1_perception::G1ObjectGeometry>(rclcpp::NodeOptions());
    // Two threads for the node's two callback groups: masks on one, depth ingest on the other.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
