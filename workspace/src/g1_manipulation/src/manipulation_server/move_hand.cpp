/**
 * @file move_hand.cpp
 * @brief Moving one hand to a point, or pointing it at one: what a learned skill's gestures use.
 */

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <format>
#include <moveit/planning_scene/planning_scene.hpp>
#include <numbers>
#include <random>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>

#include "g1_manipulation/g1_manipulation_server_node.hpp"

namespace g1_manipulation
{

namespace
{

double distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
    return std::hypot(b.x - a.x, b.y - a.y, b.z - a.z);
}

}  // namespace

geometry_msgs::msg::Point pointingHand(
    const geometry_msgs::msg::Point& shoulder, const geometry_msgs::msg::Point& target,
    double reach_m)
{
    const double              length = distance(shoulder, target);
    geometry_msgs::msg::Point hand   = shoulder;
    if (length < 1e-6)
    {
        return hand;
    }
    // A target nearer than the reach is reached, not overshot.
    const double share = std::min(reach_m, length) / length;
    hand.x += (target.x - shoulder.x) * share;
    hand.y += (target.y - shoulder.y) * share;
    hand.z += (target.z - shoulder.z) * share;
    return hand;
}

std::vector<geometry_msgs::msg::Quaternion>
handOrientations(const geometry_msgs::msg::Quaternion& current, int count)
{
    std::vector<geometry_msgs::msg::Quaternion> out{ current };
    // Fixed seed: one goal plans the same way each time it is asked.
    std::mt19937                           random(7);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < count; ++i)
    {
        // Shoemake's uniform rotations.
        const double                   u1 = unit(random);
        const double                   u2 = 2.0 * std::numbers::pi * unit(random);
        const double                   u3 = 2.0 * std::numbers::pi * unit(random);
        geometry_msgs::msg::Quaternion q;
        q.x = std::sqrt(1.0 - u1) * std::sin(u2);
        q.y = std::sqrt(1.0 - u1) * std::cos(u2);
        q.z = std::sqrt(u1) * std::sin(u3);
        q.w = std::sqrt(u1) * std::cos(u3);
        out.push_back(q);
    }
    return out;
}

void G1ManipulationServer::executeMoveHand(const std::shared_ptr<GoalHandle<MoveHand>>& goal_handle)
{
    const auto goal   = goal_handle->get_goal();
    auto       result = std::make_shared<MoveHand::Result>();
    const auto refuse = [&](const std::string& why) {
        result->success = false;
        result->message = why;
        goal_handle->abort(result);
    };

    ArmContext arm;
    if (!resolveArm(goal->arm, arm))
    {
        refuse("arm must be left or right, not '" + goal->arm + "'");
        return;
    }
    if (goal->mode != "reach" && goal->mode != "point")
    {
        refuse("mode must be reach or point, not '" + goal->mode + "'");
        return;
    }
    MoveGroup* group = groupFor(arm.arm_group);
    if (group == nullptr)
    {
        refuse("'" + arm.arm_group + "' is not a group this node drives");
        return;
    }

    geometry_msgs::msg::Point aim;
    geometry_msgs::msg::Point shoulder;
    try
    {
        aim = tf_buffer_.transform(goal->target, planning_frame_, tf2::durationFromSec(0.5)).point;
        const auto at = tf_buffer_
                            .lookupTransform(
                                planning_frame_,
                                arm.is_left ? move_hand_left_shoulder_ : move_hand_right_shoulder_,
                                tf2::TimePointZero,
                                tf2::durationFromSec(0.5))
                            .transform.translation;
        shoulder.x = at.x;
        shoulder.y = at.y;
        shoulder.z = at.z;
    }
    catch (const tf2::TransformException& e)
    {
        refuse(std::string("cannot place the target: ") + e.what());
        return;
    }
    aim.x += goal->offset.x;
    aim.y += goal->offset.y;
    aim.z += goal->offset.z;

    const geometry_msgs::msg::Point hand =
        goal->mode == "point" ? pointingHand(shoulder, aim, move_hand_point_reach_m_) : aim;
    if (const double reach = distance(shoulder, hand); reach > move_hand_max_reach_m_)
    {
        refuse(std::format(
            "the hand would be {:.2f} m from the shoulder; it reaches {:.2f} m at most",
            reach,
            move_hand_max_reach_m_));
        return;
    }

    const auto now = group->getCurrentState(1.0);
    if (!now)
    {
        refuse("the arm's state is not known");
        return;
    }
    const moveit::core::JointModelGroup* jmg = now->getJointModelGroup(arm.arm_group);
    const Eigen::Quaterniond             held(now->getGlobalLinkTransform(arm.palm_link).linear());
    geometry_msgs::msg::Quaternion       own;
    own.x = held.x();
    own.y = held.y();
    own.z = held.z();
    own.w = held.w();
    // IK knows nothing of the body: a pose with the arm through the torso is no pose at all.
    const planning_scene::PlanningScene body(now->getRobotModel());
    const auto                          free = [&body](
                          moveit::core::RobotState*            state,
                          const moveit::core::JointModelGroup* group_model,
                          const double*                        values) {
        state->setJointGroupPositions(group_model, values);
        state->update();
        return !body.isStateColliding(*state, group_model->getName());
    };
    std::vector<std::pair<double, moveit::core::RobotState>> solutions;
    geometry_msgs::msg::Pose                                 target;
    target.position = hand;
    for (const auto& q : handOrientations(own, move_hand_orientations_))
    {
        target.orientation               = q;
        moveit::core::RobotState attempt = *now;
        if (attempt.setFromIK(jmg, target, arm.palm_link, ik_timeout_s_, free))
        {
            solutions.emplace_back(attempt.distance(*now, jmg), attempt);
        }
    }
    if (solutions.empty())
    {
        refuse(std::format(
            "no way of holding the hand reaches ({:.2f}, {:.2f}, {:.2f})",
            hand.x,
            hand.y,
            hand.z));
        return;
    }
    // The least change of pose first; one the scene blocks fails its plan at once.
    std::ranges::sort(solutions, {}, &std::pair<double, moveit::core::RobotState>::first);

    // Slower than the skills' own moves: a gesture is made near people, and nothing waits on it.
    const double speed   = goal->speed > 0.0 ? std::min(goal->speed, 1.0) : 1.0;
    double       scaling = velocity_scaling_ * move_hand_speed_ * speed;
    group->setMaxVelocityScalingFactor(scaling);
    group->setMaxAccelerationScalingFactor(scaling);
    setStartStateInBounds(*group);
    MoveGroup::Plan plan;
    bool            planned_ok = false;
    std::size_t     chosen     = 0;
    for (; chosen < std::min<std::size_t>(solutions.size(), 3) && !planned_ok; ++chosen)
    {
        group->setJointValueTarget(solutions[chosen].second);
        planned_ok = planWithinBudget(*group, plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
    // A move too slow to end in time is planned again faster, never past the arm's own speed.
    const auto&  points = plan.trajectory.joint_trajectory.points;
    const double took =
        points.empty() ? 0.0 : rclcpp::Duration(points.back().time_from_start).seconds();
    if (planned_ok && took > move_hand_max_s_ && scaling < velocity_scaling_)
    {
        scaling = std::min(velocity_scaling_, scaling * took / move_hand_max_s_);
        RCLCPP_INFO(
            get_logger(),
            "move_hand: %.0f s at this speed, past %.0f s; planned again at %.0f%% of the arm's "
            "speed",
            took,
            move_hand_max_s_,
            100.0 * scaling / velocity_scaling_);
        group->setMaxVelocityScalingFactor(scaling);
        group->setMaxAccelerationScalingFactor(scaling);
        group->setJointValueTarget(solutions[chosen - 1].second);
        // Unplannable faster, the slow plan still goes: late beats not at all.
        if (MoveGroup::Plan faster;
            planWithinBudget(*group, faster) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            plan = std::move(faster);
        }
    }
    const bool moved = planned_ok && !goal_handle->is_canceling() &&
                       group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    group->setMaxVelocityScalingFactor(velocity_scaling_);
    group->setMaxAccelerationScalingFactor(velocity_scaling_);

    if (goal_handle->is_canceling())
    {
        result->success = false;
        result->message = "cancelled: the arm holds where it is";
        goal_handle->canceled(result);
        return;
    }
    if (!planned_ok)
    {
        refuse("no collision-free path takes the hand there");
        return;
    }
    if (!moved)
    {
        refuse("the arm did not get there");
        return;
    }
    result->success = true;
    result->message = std::format(
        "{} hand at ({:.2f}, {:.2f}, {:.2f}) in {}",
        goal->arm,
        hand.x,
        hand.y,
        hand.z,
        planning_frame_);
    goal_handle->succeed(result);
}

}  // namespace g1_manipulation
