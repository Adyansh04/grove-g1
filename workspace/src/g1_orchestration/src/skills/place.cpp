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

    // A named surface beats a coordinate: a coordinate here is in map, while ApproachObject
    // parks against /objects in odom. Those agree only as well as AMCL does, measured 0.23 m
    // out against an arm window of 0.04 m.
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
    // Position only. How the object is oriented when it lands is the server's business: it knows
    // how the object is held and this tree does not.
    goal.pose.header.frame_id    = getInput<std::string>("frame_id").value_or("");
    goal.pose.pose.position.x    = target->x;
    goal.pose.pose.position.y    = target->y;
    goal.pose.pose.position.z    = target->z;
    goal.pose.pose.orientation.w = 1.0;
    return true;
}

}  // namespace g1_orchestration
