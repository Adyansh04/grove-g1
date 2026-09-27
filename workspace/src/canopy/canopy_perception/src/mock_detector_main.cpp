/**
 * @file mock_detector_main.cpp
 * @brief Entry point for the stand-in detector.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "canopy_perception/mock_detector_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<canopy_perception::MockDetector>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
