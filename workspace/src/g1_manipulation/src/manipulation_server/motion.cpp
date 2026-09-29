/**
 * @file motion.cpp
 * @brief Moving the arm: planning, straight lines, settling above a target and descending.
 */

#include <chrono>
#include <cmath>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "g1_manipulation/g1_manipulation_server_node.hpp"

namespace g1_manipulation
{

void G1ManipulationServer::setStartStateInBounds(MoveGroup& group)
{
    const auto state = group.getCurrentState();
    if (!state)
    {
        group.setStartStateToCurrentState();
        return;
    }
    moveit::core::RobotState bounded(*state);
    bounded.enforceBounds(bounded.getRobotModel()->getJointModelGroup(group.getName()));
    group.setStartState(bounded);
}

std::optional<geometry_msgs::msg::Point> G1ManipulationServer::residualTo(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    geometry_msgs::msg::TransformStamped here;
    try
    {
        here = tf_buffer_.lookupTransform(
            group.getPlanningFrame(),
            link,
            tf2::TimePointZero,
            tf2::durationFromSec(0.5));
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_WARN(get_logger(), "%s: cannot measure %s: %s", what.c_str(), link.c_str(), e.what());
        return std::nullopt;
    }
    geometry_msgs::msg::Point residual;
    residual.x = pose.position.x - here.transform.translation.x;
    residual.y = pose.position.y - here.transform.translation.y;
    residual.z = pose.position.z - here.transform.translation.z;
    return residual;
}

double G1ManipulationServer::moveStraight(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what, double min_fraction)
{
    setStartStateInBounds(group);
    const std::string previous_tip = group.getEndEffectorLink();
    group.setEndEffectorLink(link);
    moveit_msgs::msg::RobotTrajectory           path;
    const std::vector<geometry_msgs::msg::Pose> waypoints{ pose };
    const double                                fraction =
        group.computeCartesianPath(waypoints, cartesian_step_m_, path, /*avoid_collisions=*/true);
    group.setEndEffectorLink(previous_tip);
    if (fraction <= 0.0 || fraction < min_fraction)
    {
        return fraction;
    }
    MoveGroup::Plan plan;
    plan.trajectory = path;
    if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_WARN(get_logger(), "%s: the straight line would not execute", what.c_str());
        return 0.0;
    }
    // execute() returns before the arm has sagged into its steady-state error.
    std::this_thread::sleep_for(std::chrono::duration<double>(settle_wait_s_));
    return fraction;
}

geometry_msgs::msg::Pose G1ManipulationServer::stagingPose(
    const geometry_msgs::msg::Pose& pregrasp, const geometry_msgs::msg::Pose& grasp) const
{
    const double axis_length = std::hypot(
        pregrasp.position.x - grasp.position.x,
        pregrasp.position.y - grasp.position.y,
        pregrasp.position.z - grasp.position.z);
    if (axis_length <= 0.0)
    {
        return grasp;
    }
    const double             scale   = reaim_clearance_m_ / axis_length;
    geometry_msgs::msg::Pose staging = grasp;
    staging.position.x += (pregrasp.position.x - grasp.position.x) * scale;
    staging.position.y += (pregrasp.position.y - grasp.position.y) * scale;
    staging.position.z += (pregrasp.position.z - grasp.position.z) * scale;
    return staging;
}

void G1ManipulationServer::backOff(const ArmContext& arm, MoveGroup& group)
{
    geometry_msgs::msg::Pose clear = group.getCurrentPose(arm.grasp_frame).pose;
    clear.position.z += reaim_clearance_m_;
    moveStraight(group, clear, arm.grasp_frame, "back off", 0.0);
}

bool G1ManipulationServer::descendOnto(
    MoveGroup& group, const geometry_msgs::msg::Pose& pregrasp,
    const geometry_msgs::msg::Pose& grasp, const std::string& link, double max_offset_m)
{
    if (stagingPose(pregrasp, grasp).position == grasp.position)
    {
        RCLCPP_ERROR(get_logger(), "approach: the pregrasp and the grasp are the same pose");
        return false;
    }

    geometry_msgs::msg::Pose commanded = grasp;
    for (int attempt = 0; attempt <= settle_attempts_; ++attempt)
    {
        const geometry_msgs::msg::Pose staging = stagingPose(pregrasp, commanded);
        if (attempt > 0)
        {
            // Straight back up the axis first: a planned move from grasp height can sweep the
            // object aside.
            geometry_msgs::msg::Pose raised = commanded;
            raised.position                 = group.getCurrentPose(link).pose.position;
            raised.position.x += staging.position.x - commanded.position.x;
            raised.position.y += staging.position.y - commanded.position.y;
            raised.position.z += staging.position.z - commanded.position.z;
            moveStraight(group, raised, link, "back up", 0.0);
        }
        // Planned, because a retry needs a fresh arm configuration to stop the line truncating
        // in the same place. The first attempt skips it when the caller already staged.
        const auto to_staging = residualTo(group, staging, link, "re-stage");
        const bool staged =
            attempt == 0 && to_staging &&
            std::hypot(to_staging->x, to_staging->y, to_staging->z) <= settle_tolerance_m_;
        if (!staged && !moveTo(group, staging, link, "re-stage"))
        {
            RCLCPP_WARN(get_logger(), "approach: could not re-stage; descending from here");
        }

        const double walked   = moveStraight(group, commanded, link, "approach", 0.0);
        const auto   residual = residualTo(group, grasp, link, "approach");
        if (!residual)
        {
            return false;
        }
        const double error = std::hypot(residual->x, residual->y, residual->z);
        if (!std::isfinite(error))
        {
            RCLCPP_ERROR(get_logger(), "approach: the hand's position is not finite");
            return false;
        }
        if (error <= settle_tolerance_m_)
        {
            RCLCPP_INFO(
                get_logger(),
                "approach: on the grasp pose to %.0f mm after %d descent(s)",
                error * 1000.0,
                attempt + 1);
            return true;
        }
        if (attempt == settle_attempts_)
        {
            // Past max_offset_m the object cannot be between the fingers.
            if (error > max_offset_m)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "approach: %.0f mm off after %d descents, past the %.0f mm the hand can still "
                    "close over; not closing on air",
                    error * 1000.0,
                    attempt + 1,
                    max_offset_m * 1000.0);
                return false;
            }
            RCLCPP_WARN(
                get_logger(),
                "approach: still %.0f mm off after %d descents; closing from there",
                error * 1000.0,
                attempt + 1);
            return true;
        }
        // Overshoot only what the arm failed to hold. A line that ran out early has its unwalked
        // rest in the residual, and adding that aims the hand into the surface.
        if (walked >= 1.0)
        {
            RCLCPP_INFO(
                get_logger(),
                "approach: %.0f mm off, re-aiming by (%+.0f %+.0f %+.0f) mm",
                error * 1000.0,
                residual->x * 1000.0,
                residual->y * 1000.0,
                residual->z * 1000.0);
            commanded.position.x += residual->x;
            commanded.position.y += residual->y;
            commanded.position.z += residual->z;
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "approach: the straight line ran out %.0f%% in, %.0f mm short; starting it again",
                walked * 100.0,
                error * 1000.0);
        }
    }
    return true;
}

bool G1ManipulationServer::settleOnPose(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    // Re-commanding the residual beats raising gains, which the balance controller shares.
    geometry_msgs::msg::Pose commanded = pose;
    // One more measurement than nudges, so the last nudge is checked too.
    for (int attempt = 0; attempt <= settle_attempts_; ++attempt)
    {
        const auto residual = residualTo(group, pose, link, what);
        if (!residual)
        {
            return false;
        }
        const double error = std::hypot(residual->x, residual->y, residual->z);
        if (error <= settle_tolerance_m_)
        {
            RCLCPP_INFO(
                get_logger(),
                "%s: settled %.0f mm from the grasp pose after %d nudge(s)",
                what.c_str(),
                error * 1000.0,
                attempt);
            return true;
        }
        if (attempt == settle_attempts_)
        {
            break;
        }
        // The next target droops by about the same amount, so it lands on the one asked for.
        commanded.position.x += residual->x;
        commanded.position.y += residual->y;
        commanded.position.z += residual->z;
        RCLCPP_INFO(
            get_logger(),
            "%s: %.0f mm short, nudging (%+.0f %+.0f %+.0f) mm",
            what.c_str(),
            error * 1000.0,
            residual->x * 1000.0,
            residual->y * 1000.0,
            residual->z * 1000.0);
        // A straight line only: re-planning a small correction discards the arm's configuration.
        if (moveStraight(group, commanded, link, what + " settle", 0.0) <= 0.0)
        {
            RCLCPP_WARN(
                get_logger(),
                "%s: no straight line to the correction; carrying on",
                what.c_str());
            return true;
        }
    }
    RCLCPP_WARN(
        get_logger(),
        "%s: gave up nudging after %d attempts; carrying on from where it is",
        what.c_str(),
        settle_attempts_);
    return true;
}

bool G1ManipulationServer::moveTo(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    setStartStateInBounds(group);
    // The goal is for `link`, which is not the group's own tip.
    group.setPoseTarget(pose, link);

    MoveGroup::Plan plan;
    const auto      planned = planWithinBudget(group, plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "%s: planning failed (%d)", what.c_str(), planned.val);
        return false;
    }
    const auto executed = group.execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            get_logger(),
            "%s: execution failed (%d). The usual cause is the arm not being acquired; run "
            "g1_bringup's activate_arm first.",
            what.c_str(),
            executed.val);
        return false;
    }
    return true;
}

bool G1ManipulationServer::moveToNamed(MoveGroup& group, const std::string& named_target)
{
    setStartStateInBounds(group);
    if (!group.setNamedTarget(named_target))
    {
        RCLCPP_ERROR(
            get_logger(),
            "'%s' is not a named pose of group '%s'",
            named_target.c_str(),
            group.getName().c_str());
        return false;
    }

    // Plan then execute, not move(): move() re-validates against every octomap update, and the
    // arm's own fresh voxels invalidate it constantly.
    MoveGroup::Plan plan;
    const auto      planned = planWithinBudget(group, plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "'%s': planning failed (%d)", named_target.c_str(), planned.val);
        return false;
    }
    return group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

moveit::core::MoveItErrorCode
G1ManipulationServer::planWithinBudget(MoveGroup& group, MoveGroup::Plan& plan)
{
    using Clock         = std::chrono::steady_clock;
    const auto deadline = Clock::now() + std::chrono::duration<double>(planning_time_s_);
    moveit::core::MoveItErrorCode result = moveit::core::MoveItErrorCode::FAILURE;
    for (int attempt = 1; attempt <= planning_attempts_; ++attempt)
    {
        const double remaining_s = std::chrono::duration<double>(deadline - Clock::now()).count();
        if (remaining_s <= 0.0)
        {
            break;
        }
        group.setPlanningTime(remaining_s);
        result = group.plan(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            break;
        }
        RCLCPP_WARN(
            get_logger(),
            "%s: plan %d of %d failed with %.1f s of budget left",
            group.getName().c_str(),
            attempt,
            planning_attempts_,
            std::chrono::duration<double>(deadline - Clock::now()).count());
    }
    group.setPlanningTime(planning_time_s_);
    return result;
}

}  // namespace g1_manipulation
