/**
 * @file mock_grasp_source_main.cpp
 * @brief Entry point for the stand-in grasp generator.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "g1_perception/mock_grasp_source_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_perception::G1MockGraspSource>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
