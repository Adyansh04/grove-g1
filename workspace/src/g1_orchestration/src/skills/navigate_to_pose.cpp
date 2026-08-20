/**
 * @file navigate_to_pose.cpp
 * @brief Ports, goal and result judgement for the NavigateToPose leaf.
 */

#include "g1_orchestration/skills/navigate_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace g1_orchestration
{

namespace
{

geometry_msgs::msg::PoseStamped toPose(const Station& station, const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = station.x;
    pose.pose.position.y = station.y;

    tf2::Quaternion heading;
    heading.setRPY(0.0, 0.0, station.yaw);
    pose.pose.orientation = tf2::toMsg(heading);
    return pose;
}

}  // namespace

NavigateToPose::NavigateToPose(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : RosActionNode(name, config, std::move(context), "/navigate_to_pose")
{}

BT::PortsList NavigateToPose::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<Station>("goal", "Where to drive to, as 'x;y;yaw'."),
        BT::InputPort<std::string>("frame_id", "map", "Frame the goal is expressed in."),
        BT::InputPort<std::string>(
            "behavior_tree",
            "",
            "Nav2 tree to drive with. Empty uses its default."),
        BT::OutputPort<double>(
            "goal_yaw",
            "The goal's heading, for an ApproachObject that has to hold it."),
    });
}

bool NavigateToPose::fillGoal(Goal& goal)
{
    const auto station = getInput<Station>("goal");
    if (!station)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name().c_str(), station.error().c_str());
        return false;
    }
    goal.pose = toPose(*station, getInput<std::string>("frame_id").value_or("map"));
    // Nav2 picks its own default when this is empty, so a tree that does not care says nothing.
    goal.behavior_tree = getInput<std::string>("behavior_tree").value_or("");

    // Published so the approach that follows can hold the heading this goal arrived on, rather
    // than needing a matching literal typed by hand in both places.
    setOutput("goal_yaw", station->yaw);
    return true;
}

BT::NodeStatus NavigateToPose::judgeResult(const WrappedResult& result)
{
    // Nav2's result is empty: reaching the goal is reported by the result CODE and nothing else,
    // unlike this stack's own skills.
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] did not reach the goal", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] arrived", name().c_str());
    return BT::NodeStatus::SUCCESS;
}

}  // namespace g1_orchestration
