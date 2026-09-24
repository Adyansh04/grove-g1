/**
 * @file mock_detector_main.cpp
 * @brief Entry point for the stand-in detector.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "g1_perception/mock_detector_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_perception::G1MockDetector>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
