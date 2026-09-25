/**
 * @file step_clear.cpp
 * @brief Ports and goal for the StepClear leaf.
 */

#include "g1_orchestration/skills/step_clear.hpp"

#include <string>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

StepClear::StepClear(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : SkillActionNode(name, config, std::move(context), "/g1_base_approach/step_clear")
{}

BT::PortsList StepClear::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>(
            "clearance",
            0.55,
            "Distance to end up from the nearest obstacle, in metres. Nav2's circle is 0.45."),
        ports::goalTimeout(),
    });
}

bool StepClear::fillGoal(Goal& goal)
{
    goal.clearance_m = getInput<double>("clearance").value_or(0.55);
    goal.timeout_s   = getInput<double>("timeout_s").value_or(0.0);
    if (goal.clearance_m <= 0.0)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] clearance must be positive", name().c_str());
        return false;
    }
    return true;
}

}  // namespace g1_orchestration
