#include "g1_orchestration/skills/approach_object.hpp"

#include <string>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

ApproachObject::ApproachObject(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : SkillActionNode(name, config, context, "/g1_base_approach/approach_object")
{}

BT::PortsList ApproachObject::providedPorts()
{
    return providedBasicPorts({
        ports::objectId(),
        ports::arm(),
        BT::InputPort<double>(
            "working_yaw",
            "Heading to hold while approaching. Pass the same yaw the staging goal used."),
        ports::goalTimeout(),
    });
}

bool ApproachObject::fillGoal(Goal& goal)
{
    const auto object_id = getInput<std::string>("object_id");
    if (!object_id || object_id->empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] needs an object_id", name().c_str());
        return false;
    }
    // Required, with no default. A missing heading would silently mean "face +x", which is a
    // valid yaw and almost never the right one -- the skill would approach square to nothing and
    // the failure would look like bad geometry rather than a missing port.
    const auto working_yaw = getInput<double>("working_yaw");
    if (!working_yaw)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] needs working_yaw: %s",
            name().c_str(),
            working_yaw.error().c_str());
        return false;
    }
    goal.object_id           = *object_id;
    goal.arm                 = getInput<std::string>("arm").value_or("right");
    goal.working_yaw         = *working_yaw;
    goal.use_current_heading = false;
    goal.timeout_s           = getInput<double>("timeout_s").value_or(0.0);
    return true;
}

}  // namespace g1_orchestration
