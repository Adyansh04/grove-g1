/**
 * @file perception_visualizer_main.cpp
 * @brief Entry point for the node that draws perception's output for RViz.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "g1_perception/perception_visualizer_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_perception::G1PerceptionVisualizer>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
