/**
 * @file mock_engine_main.cpp
 * @brief Entry point for the stand-in policy engine.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "g1_vla/mock_engine_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_vla::G1VlaMockEngine>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
