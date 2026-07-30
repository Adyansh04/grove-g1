#include "g1_bringup/walk_policy_sim_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_bringup::WalkPolicySim>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
