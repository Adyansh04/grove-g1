/**
 * @file retreat.cpp
 * @brief Ports and goal for the Retreat leaf.
 */

#include "g1_orchestration/skills/retreat.hpp"

#include <string>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

Retreat::Retreat(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : SkillActionNode(name, config, std::move(context), "/g1_base_approach/retreat")
{}

BT::PortsList Retreat::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("distance", 0.6, "How far to reverse, in metres."),
        ports::goalTimeout(),
    });
}

bool Retreat::fillGoal(Goal& goal)
{
    goal.distance_m = getInput<double>("distance").value_or(0.6);
    goal.timeout_s  = getInput<double>("timeout_s").value_or(0.0);
    if (goal.distance_m <= 0.0)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] distance must be positive", name().c_str());
        return false;
    }
    return true;
}

}  // namespace g1_orchestration
