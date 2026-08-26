/**
 * @file grasp.cpp
 * @brief Ports and goal for the Grasp leaf.
 */

#include "g1_orchestration/skills/grasp.hpp"

#include <string>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

Grasp::Grasp(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : SkillActionNode(name, config, std::move(context), "/g1_vla_server/grasp")
{}

BT::PortsList Grasp::providedPorts()
{
    return providedBasicPorts({ BT::InputPort<std::string>(
                                    "instruction",
                                    "What the policy is asked to do, in plain language."),
                                ports::objectId(),
                                ports::arm() });
}

bool Grasp::fillGoal(Goal& goal)
{
    const auto instruction = getInput<std::string>("instruction");
    if (!instruction || instruction->empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] needs an instruction", name().c_str());
        return false;
    }
    // Separate from the instruction on purpose: the policy is told what to do, and this names
    // the object whose measured lift decides whether it did it.
    const auto object_id = getInput<std::string>("object_id");
    if (!object_id || object_id->empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] needs an object_id", name().c_str());
        return false;
    }
    goal.instruction = *instruction;
    goal.object_id   = *object_id;
    goal.arm         = getInput<std::string>("arm").value_or("right");
    return true;
}

}  // namespace g1_orchestration
