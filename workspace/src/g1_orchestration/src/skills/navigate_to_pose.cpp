/**
 * @file navigate_to_pose.cpp
 * @brief Ports, goal and result judgement for the NavigateToPose leaf.
 */

#include "g1_orchestration/skills/navigate_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
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

std::string describeNavigationCode(std::uint16_t code)
{
    using Plan   = nav2_msgs::action::ComputePathToPose::Result;
    using Follow = nav2_msgs::action::FollowPath::Result;
    switch (code)
    {
        case Plan::GOAL_OUTSIDE_MAP:
            return "the goal is outside the map";
        case Plan::GOAL_OCCUPIED:
            return "the goal is inside an obstacle on the map";
        case Plan::START_OUTSIDE_MAP:
            return "the robot is outside the map";
        case Plan::START_OCCUPIED:
            return "the robot stands inside an obstacle on the map";
        case Plan::NO_VALID_PATH:
            return "no path leads there";
        case Plan::TIMEOUT:
            return "the planner ran out of time";
        case Plan::TF_ERROR:
        case Follow::TF_ERROR:
            return "there was no pose of the robot to plan from";
        case Follow::FAILED_TO_MAKE_PROGRESS:
            return "the robot stopped making progress along the path";
        case Follow::NO_VALID_CONTROL:
            return "the controller found no safe way along the path";
        case Follow::PATIENCE_EXCEEDED:
        case Follow::CONTROLLER_TIMED_OUT:
            return "the controller ran out of time";
        default:
            return {};
    }
}

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
    goal.pose          = toPose(*station, getInput<std::string>("frame_id").value_or("map"));
    goal.behavior_tree = getInput<std::string>("behavior_tree").value_or("");

    setOutput("goal_yaw", station->yaw);
    return true;
}

BT::NodeStatus NavigateToPose::judgeResult(const WrappedResult& result)
{
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] did not reach the goal", name().c_str());
        // Nav2 1.3 leaves error_msg empty, so its code is worded here; the result code is all
        // there is without either.
        std::string why = result.result ? result.result->error_msg : std::string{};
        if (why.empty() && result.result)
        {
            why = describeNavigationCode(result.result->error_code);
        }
        setFailureText(
            "Nav2 did not reach the goal: " + (why.empty() ? describeResultCode(result.code) : why));
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] arrived", name().c_str());
    return BT::NodeStatus::SUCCESS;
}

}  // namespace g1_orchestration
