#include <rclcpp/rclcpp.hpp>

#include "g1_bringup/arm_sdk_sim_bridge_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_bringup::ArmSdkSimBridge>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
