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
    rclcpp::spin(std::make_shared<g1_perception::G1ObjectGeometry>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
