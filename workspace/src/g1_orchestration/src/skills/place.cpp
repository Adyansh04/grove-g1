/**
 * @file place.cpp
 * @brief Ports and goal for the Place leaf.
 */

#include "g1_orchestration/skills/place.hpp"

#include <string>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

Place::Place(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : SkillActionNode(name, config, std::move(context), "/g1_manipulation_server/place")
{}

BT::PortsList Place::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>(
            "surface",
            "",
            "Detected surface to place ON TOP OF. Preferred over 'target'."),
        BT::InputPort<Point3>("target", "Where the OBJECT should end up, as 'x;y;z'."),
        ports::arm(),
        BT::InputPort<std::string>(
            "frame_id",
            "",
            "Frame of the target. Empty means the server's planning frame."),
    });
}

bool Place::fillGoal(Goal& goal)
{
    goal.arm = getInput<std::string>("arm").value_or("right");

    // A detected surface beats a fixed coordinate: it is measured on /objects, the same stream
    // ApproachObject parks against.
    goal.surface_object_id = getInput<std::string>("surface").value_or("");
    if (!goal.surface_object_id.empty())
    {
        return true;
    }

    const auto target = getInput<Point3>("target");
    if (!target)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name().c_str(), target.error().c_str());
        return false;
    }
    // Position only; the server picks the orientation, since it knows how the object is held.
    goal.pose.header.frame_id    = getInput<std::string>("frame_id").value_or("");
    goal.pose.pose.position.x    = target->x;
    goal.pose.pose.position.y    = target->y;
    goal.pose.pose.position.z    = target->z;
    goal.pose.pose.orientation.w = 1.0;
    return true;
}

}  // namespace g1_orchestration
