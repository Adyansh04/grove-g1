#include "g1_orchestration/skills/approach_object.hpp"

#include <string>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

ApproachObject::ApproachObject(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : SkillActionNode(name, config, std::move(context), "/g1_base_approach/approach_object")
{}

BT::PortsList ApproachObject::providedPorts()
{
    return providedBasicPorts({
        ports::objectId(),
        ports::arm(),
        BT::InputPort<double>(
            "working_yaw",
            "Heading to hold while approaching, usually the staging goal's goal_yaw. Ignored "
            "when use_current_heading is true."),
        BT::InputPort<bool>(
            "use_current_heading",
            false,
            "Hold whatever heading the robot already has, instead of working_yaw."),
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

    goal.use_current_heading = getInput<bool>("use_current_heading").value_or(false);

    // working_yaw is required UNLESS the caller asked to keep the current heading. Defaulting it
    // would silently mean "face +x", which is a valid yaw and almost never the right one: the
    // skill would approach square to nothing and the failure would read as bad geometry rather
    // than a missing port.
    const auto working_yaw = getInput<double>("working_yaw");
    if (!goal.use_current_heading && !working_yaw)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] needs working_yaw, or use_current_heading: %s",
            name().c_str(),
            working_yaw.error().c_str());
        return false;
    }

    goal.object_id   = *object_id;
    goal.arm         = getInput<std::string>("arm").value_or("right");
    goal.working_yaw = working_yaw.value_or(0.0);
    goal.timeout_s   = getInput<double>("timeout_s").value_or(0.0);
    return true;
}

}  // namespace g1_orchestration
