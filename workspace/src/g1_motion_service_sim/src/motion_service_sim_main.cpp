#include <rclcpp/rclcpp.hpp>

#include "g1_motion_service_sim/motion_service_sim_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_motion_service_sim::MotionServiceSim>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
