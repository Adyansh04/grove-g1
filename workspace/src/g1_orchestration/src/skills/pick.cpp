#include "g1_orchestration/skills/pick.hpp"

#include <string>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

Pick::Pick(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : SkillActionNode(name, config, std::move(context), "/g1_manipulation_server/pick")
{}

BT::PortsList Pick::providedPorts()
{
    return providedBasicPorts({ ports::objectId(), ports::arm() });
}

bool Pick::fillGoal(Goal& goal)
{
    const auto object_id = getInput<std::string>("object_id");
    if (!object_id || object_id->empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] needs an object_id", name().c_str());
        return false;
    }
    goal.object_id = *object_id;
    goal.arm       = getInput<std::string>("arm").value_or("right");
    return true;
}

}  // namespace g1_orchestration
