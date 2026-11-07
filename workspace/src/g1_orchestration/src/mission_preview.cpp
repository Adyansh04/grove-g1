/**
 * @file mission_preview.cpp
 * @brief The PreviewMission service: each step's end pose and Nav2 path, without moving.
 */

#include "g1_orchestration/mission_preview.hpp"

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <map>
#include <numbers>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <utility>

#include "g1_orchestration/skills/navigate_to_pose.hpp"
#include "g1_orchestration/text_values.hpp"

namespace g1_orchestration
{
namespace
{
using StepPreview = nervros_interfaces::msg::StepPreview;

std::optional<double> number(const std::map<std::string, std::string>& args, const char* key)
{
    const auto it = args.find(key);
    if (it == args.end())
    {
        return std::nullopt;
    }
    return realNumber(it->second);
}

/// Whether the preview follows @p macro from where the robot stands.
bool followed(const std::string& macro)
{
    return macro == "GoToPose" || macro == "GoToTarget" || macro == "WalkStraight" ||
           macro == "TurnInPlace" || macro == "PickObject" || macro == "PlaceInto" ||
           macro == "PlaceOn";
}

std::string argOf(const MissionStep& step, const char* key)
{
    const auto it = step.args.find(key);
    return it == step.args.end() ? std::string() : it->second;
}
}  // namespace

MissionPreview::MissionPreview(
    rclcpp::Node& node, Params params, const rclcpp::CallbackGroup::SharedPtr& group)
  : params_(std::move(params))
  , clock_(node.get_clock())
  , buffer_(node.get_clock())
  , listener_(buffer_)
{
    planner_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
        &node,
        params_.planner_action,
        group);
    approach_ = node.create_client<canopy_msgs::srv::GetApproachPose>(
        params_.approach_service,
        rclcpp::ServicesQoS(),
        group);
    clear_ = node.create_client<nav2_msgs::srv::ClearEntireCostmap>(
        params_.clear_costmap_service,
        rclcpp::ServicesQoS(),
        group);
}

std::chrono::duration<double> MissionPreview::wait() const
{
    const std::chrono::duration<double> left = deadline_ - std::chrono::steady_clock::now();
    return std::clamp(
        left,
        std::chrono::duration<double>(0.0),
        std::chrono::duration<double>(params_.timeout_s));
}

bool MissionPreview::clearCostmap()
{
    if (!clear_->wait_for_service(wait()))
    {
        return false;
    }
    auto future =
        clear_->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>());
    if (future.wait_for(wait()) != std::future_status::ready)
    {
        clear_->remove_pending_request(future);
        return false;
    }
    // The planner sees the clear with the costmap's next update.
    std::this_thread::sleep_for(std::chrono::duration<double>(params_.clear_wait_s));
    return true;
}

Station MissionPreview::walkEnd(const Station& from, double metres)
{
    return { from.x + metres * std::cos(from.yaw), from.y + metres * std::sin(from.yaw), from.yaw };
}

double MissionPreview::backOffAfter(const std::string& macro)
{
    if (macro == "PickObject")
    {
        return 1.2;
    }
    if (macro == "PlaceInto")
    {
        return 0.8;
    }
    if (macro == "PlaceOn")
    {
        return 0.5;
    }
    return 0.0;
}

Station MissionPreview::turnEnd(const Station& from, double degrees)
{
    const double yaw =
        std::remainder(from.yaw + degrees * std::numbers::pi / 180.0, 2.0 * std::numbers::pi);
    return { from.x, from.y, yaw };
}

geometry_msgs::msg::PoseStamped MissionPreview::stamped(const Station& station) const
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = params_.map_frame;
    pose.header.stamp    = clock_->now();
    pose.pose.position.x = station.x;
    pose.pose.position.y = station.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, station.yaw);
    pose.pose.orientation = tf2::toMsg(q);
    return pose;
}

std::optional<Station> MissionPreview::robotPose(std::string& why) const
{
    try
    {
        const auto t = buffer_.lookupTransform(
            params_.map_frame,
            params_.base_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(wait().count()));
        return Station{ t.transform.translation.x,
                        t.transform.translation.y,
                        tf2::getYaw(t.transform.rotation) };
    }
    catch (const tf2::TransformException& e)
    {
        why =
            std::format("no pose for {} in {}: {}", params_.base_frame, params_.map_frame, e.what());
        return std::nullopt;
    }
}

std::optional<Station> MissionPreview::approachPose(const std::string& target, std::string& why)
{
    if (approach_absent_ || !approach_->wait_for_service(wait()))
    {
        // Within one preview, a second wait would only repeat the first.
        approach_absent_ = true;
        why              = "the world model does not answer";
        return std::nullopt;
    }
    auto request    = std::make_shared<canopy_msgs::srv::GetApproachPose::Request>();
    request->target = target;
    auto future     = approach_->async_send_request(request);
    if (future.wait_for(wait()) != std::future_status::ready)
    {
        approach_->remove_pending_request(future);
        why = "the world model did not answer in time";
        return std::nullopt;
    }
    const auto response = future.get();
    if (!response->success)
    {
        why = response->message;
        return std::nullopt;
    }
    const auto& pose = response->pose.pose;
    return Station{ pose.position.x, pose.position.y, tf2::getYaw(pose.orientation) };
}

std::optional<nav_msgs::msg::Path>
MissionPreview::plan(const Station& from, const Station& to, std::string& why)
{
    using Action = nav2_msgs::action::ComputePathToPose;
    if (planner_absent_ || !planner_->wait_for_action_server(wait()))
    {
        planner_absent_ = true;
        why             = "Nav2's planner is not up";
        return std::nullopt;
    }
    Action::Goal goal;
    goal.start     = stamped(from);
    goal.goal      = stamped(to);
    goal.use_start = true;
    auto sent      = planner_->async_send_goal(goal);
    if (sent.wait_for(wait()) != std::future_status::ready || sent.get() == nullptr)
    {
        why = "the planner did not take the request";
        return std::nullopt;
    }
    auto result = planner_->async_get_result(sent.get());
    if (result.wait_for(wait()) != std::future_status::ready)
    {
        why = "the planner took too long";
        try
        {
            planner_->async_cancel_goal(sent.get());
        }
        catch (const rclcpp_action::exceptions::UnknownGoalHandleError&)
        {
            // The goal ended, and rclcpp forgot it, as the cancel went out.
            why = "the planner took too long, and ended as it was cancelled";
        }
        return std::nullopt;
    }
    const auto& wrapped = result.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || wrapped.result->path.poses.empty())
    {
        if (wrapped.result && !wrapped.result->error_msg.empty())
        {
            why = wrapped.result->error_msg;
        }
        else if (wrapped.result && wrapped.result->error_code != Action::Result::NONE)
        {
            why = describeNavigationCode(wrapped.result->error_code);
            if (why.empty())
            {
                why = std::format("no path found (Nav2 error {})", wrapped.result->error_code);
            }
        }
        else
        {
            why = "no path found";
        }
        return std::nullopt;
    }
    return wrapped.result->path;
}

std::vector<StepPreview> MissionPreview::preview(
    const std::vector<MissionStep>& steps, std::string& why, bool may_clear,
    const std::set<std::string>& walkers)
{
    deadline_ = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(params_.total_s));
    approach_absent_          = false;
    planner_absent_           = false;
    std::optional<Station> at = robotPose(why);
    if (!at)
    {
        return {};
    }
    std::vector<StepPreview> out;
    out.reserve(steps.size());
    for (const MissionStep& step : steps)
    {
        StepPreview preview;
        preview.step_id = step.id;
        preview.name    = step.name;
        if (std::chrono::steady_clock::now() >= deadline_)
        {
            // A picture that comes late is none: the rest is listed without a path.
            preview.note = "not previewed: the preview ran out of time";
            out.push_back(std::move(preview));
            continue;
        }
        // Past a step whose end it cannot tell, a walk would be drawn from the wrong place.
        if (!at && (walkers.contains(step.macro) || followed(step.macro)))
        {
            preview.note = "moves the base from where an earlier step leaves it, which is not "
                           "predicted";
            out.push_back(std::move(preview));
            continue;
        }
        std::optional<Station> end;
        bool                   planned = false;
        if (step.macro == "GoToTarget" && argOf(step, "target").starts_with('{'))
        {
            preview.note = "walks to what an earlier step finds, known only when it runs";
            at.reset();
        }
        else if (step.macro == "GoToPose" || step.macro == "GoToTarget")
        {
            std::string problem;
            if (step.macro == "GoToPose")
            {
                try
                {
                    end = BT::convertFromString<Station>(argOf(step, "station"));
                }
                catch (const std::exception& e)
                {
                    problem = e.what();
                }
            }
            else
            {
                end = approachPose(argOf(step, "target"), problem);
            }
            if (end)
            {
                // The walk clears the costmaps before it plans: on a refusal, clear and ask again,
                // unless something moves the robot now and plans with the live costmap.
                auto path = plan(*at, *end, problem);
                if (!path && may_clear && clearCostmap())
                {
                    path = plan(*at, *end, problem);
                }
                else if (!path && !may_clear)
                {
                    problem += " (asked without clearing the costmap, since the robot is busy)";
                }
                if (path)
                {
                    preview.path = std::move(*path);
                    planned      = true;
                }
            }
            preview.note = planned ? "walks there along Nav2's path" : "no path: " + problem;
            // A walk to a place the preview cannot find ends where it cannot tell.
            if (!end)
            {
                at.reset();
            }
        }
        else if (step.macro == "WalkStraight")
        {
            const double metres        = number(step.args, "distance_m").value_or(0.0);
            const double signed_metres = argOf(step, "direction") == "backward" ? -metres : metres;
            end                        = walkEnd(*at, signed_metres);
            preview.note =
                std::format("walks {:.1f} m {}", metres, signed_metres < 0 ? "backward" : "forward");
        }
        else if (step.macro == "TurnInPlace")
        {
            const double degrees = number(step.args, "degrees").value_or(0.0);
            end                  = turnEnd(*at, degrees);
            preview.note         = std::format(
                "turns {:.0f} degrees {} in place",
                std::abs(degrees),
                degrees < 0 ? "right" : "left");
        }
        else if (step.macro == "PickObject" || step.macro == "PlaceInto" || step.macro == "PlaceOn")
        {
            // The next walk starts where it backs off to, outside the furniture's inflated band.
            const double back = backOffAfter(step.macro);
            end               = walkEnd(*at, -back);
            preview.note      = std::format(
                "{}, then backs off {:.1f} m",
                step.macro == "PlaceOn" ? "sets it down where the hand reaches" :
                                               "closes in on what it sees",
                back);
        }
        else if (step.macro == "ExploreBuilding")
        {
            preview.note =
                "walks the world model's viewpoints; the route is decided as it explores";
            at.reset();
        }
        else if (walkers.contains(step.macro))
        {
            preview.note = "moves the base; where it ends is not predicted";
            at.reset();
        }
        else
        {
            preview.note = "does not move the base";
        }
        if (end)
        {
            preview.goal = stamped(*end);
            at           = end;
        }
        out.push_back(std::move(preview));
    }
    return out;
}

}  // namespace g1_orchestration
