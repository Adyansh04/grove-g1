/**
 * @file g1_object_pose_source_main.cpp
 * @brief Entry point for the object-pose source.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "g1_manipulation/g1_object_pose_source_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<g1_manipulation::G1ObjectPoseSource>(rclcpp::NodeOptions());
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
