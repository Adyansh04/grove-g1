/**
 * @file set_arm_posture.cpp
 * @brief Ports and goal for the SetArmPosture leaf.
 */

#include "g1_orchestration/skills/set_arm_posture.hpp"

#include <string>

namespace g1_orchestration
{

SetArmPosture::SetArmPosture(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : SkillActionNode(name, config, std::move(context), "/g1_manipulation_server/set_arm_posture")
{}

BT::PortsList SetArmPosture::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("group", "A group in g1.srdf, e.g. right_arm."),
        BT::InputPort<std::string>("named_target", "A group_state of that group, e.g. tucked."),
    });
}

bool SetArmPosture::fillGoal(Goal& goal)
{
    const auto group        = getInput<std::string>("group");
    const auto named_target = getInput<std::string>("named_target");
    if (!group || !named_target)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] needs both group and named_target", name().c_str());
        return false;
    }
    goal.group        = *group;
    goal.named_target = *named_target;
    return true;
}

}  // namespace g1_orchestration
