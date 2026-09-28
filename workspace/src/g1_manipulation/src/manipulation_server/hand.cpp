/**
 * @file hand.cpp
 * @brief The Dex3 hand: commands, closing on an object, and the grip check.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <utility>
#include <vector>

#include "g1_manipulation/g1_manipulation_server_node.hpp"
#include "g1_manipulation/grip_check.hpp"

namespace g1_manipulation
{

namespace
{

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// Shortest hand trajectory, so a small grip-search step is still a ramp.
constexpr double kMinHandMoveS = 0.15;

constexpr auto kHandSamplePeriod = std::chrono::milliseconds(100);

/// `open` to `closed` interpolated at @p fraction, per joint; empty unless both poses name it.
std::map<std::string, double> handTargetsAt(MoveGroupInterface& hand, double fraction)
{
    const std::map<std::string, double> open_pose = hand.getNamedTargetValues("open");
    std::map<std::string, double>       targets;
    for (const auto& [joint, closed] : hand.getNamedTargetValues("closed"))
    {
        const auto open = open_pose.find(joint);
        if (open == open_pose.end())
        {
            return {};
        }
        targets.emplace(joint, open->second + (fraction * (closed - open->second)));
    }
    return targets;
}

}  // namespace

std::optional<G1ManipulationServer::JointSample>
G1ManipulationServer::readJoints(const std::vector<std::string>& joints)
{
    const std::lock_guard<std::mutex> lock(joint_states_mutex_);
    const std::vector<std::string>&   names = joint_states_.name;
    if (names.size() != joint_states_.position.size())
    {
        return std::nullopt;
    }
    const bool  with_effort = joint_states_.effort.size() == names.size();
    JointSample sample;
    sample.position.reserve(joints.size());
    if (with_effort)
    {
        sample.effort.reserve(joints.size());
    }
    for (const std::string& joint : joints)
    {
        const auto at = std::ranges::find(names, joint);
        if (at == names.end())
        {
            return std::nullopt;
        }
        const auto index = static_cast<std::size_t>(std::distance(names.begin(), at));
        sample.position.push_back(joint_states_.position[index]);
        if (with_effort)
        {
            sample.effort.push_back(joint_states_.effort[index]);
        }
    }
    return sample;
}

bool G1ManipulationServer::isHolding(const ArmContext& arm, std::string& why, bool opposed)
{
    if (!grip_check_enabled_)
    {
        why = "grip check disabled";
        return true;
    }
    MoveGroup* hand = groupFor(arm.hand_group);
    if (hand == nullptr)
    {
        why = "no hand group called " + arm.hand_group;
        return false;
    }
    // Against the last close, not `closed`: a close onto a measured width stops short on purpose.
    // After holdCurrentGrip the fingers sit short of these by construction, so effort decides.
    const std::map<std::string, double> targets = handTargetsAt(*hand, last_close_fraction_);
    std::vector<std::string>            joints;
    joints.reserve(targets.size());
    for (const auto& entry : targets)
    {
        joints.push_back(entry.first);
    }
    const std::optional<JointSample> measured = readJoints(joints);
    if (targets.empty() || !measured)
    {
        why = "the hand's joints are not all in /joint_states";
        return false;
    }
    if (measured->effort.empty())
    {
        why = "joint states carry no effort, so a grip cannot be told from an empty hand";
        return false;
    }

    std::vector<JointGrip> fingers;
    fingers.reserve(joints.size());
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        fingers.push_back(
            { joints[i], targets.at(joints[i]), measured->position[i], measured->effort[i] });
    }
    const GripVerdict verdict =
        verifyGrip(fingers, grip_min_position_error_rad_, grip_min_effort_nm_);
    why = verdict.why;
    return opposed ? verdict.opposed : verdict.holding;
}

bool G1ManipulationServer::commandHand(
    MoveGroup& hand, const std::map<std::string, double>& targets, const std::string& what)
{
    const std::vector<std::string>&  joints = hand.getActiveJoints();
    const std::optional<JointSample> from   = readJoints(joints);
    if (!from)
    {
        RCLCPP_ERROR(
            get_logger(),
            "%s: the %s joints are not all in /joint_states",
            what.c_str(),
            hand.getName().c_str());
        return false;
    }
    const std::map<std::string, double> open   = handTargetsAt(hand, 0.0);
    const std::map<std::string, double> closed = handTargetsAt(hand, 1.0);

    trajectory_msgs::msg::JointTrajectoryPoint start;
    trajectory_msgs::msg::JointTrajectoryPoint end;
    moveit_msgs::msg::RobotTrajectory          path;
    // Largest move as a share of that finger's full open-to-closed travel.
    double travel = 0.0;
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        const auto target = targets.find(joints[i]);
        if (target == targets.end() || !std::isfinite(target->second))
        {
            RCLCPP_ERROR(
                get_logger(),
                "%s: no usable target for %s",
                what.c_str(),
                joints[i].c_str());
            return false;
        }
        path.joint_trajectory.joint_names.push_back(joints[i]);
        start.positions.push_back(from->position[i]);
        end.positions.push_back(target->second);
        const auto o = open.find(joints[i]);
        const auto c = closed.find(joints[i]);
        if (o != open.end() && c != closed.end() && c->second != o->second)
        {
            travel = std::max(
                travel,
                std::abs(target->second - from->position[i]) / std::abs(c->second - o->second));
        }
    }
    // Starts where the fingers are: execution refuses a trajectory that starts elsewhere.
    start.velocities.assign(start.positions.size(), 0.0);
    end.velocities.assign(end.positions.size(), 0.0);
    start.time_from_start = rclcpp::Duration::from_seconds(0.0);
    end.time_from_start   = rclcpp::Duration::from_seconds(
        std::clamp(travel * hand_close_s_, kMinHandMoveS, std::max(kMinHandMoveS, hand_close_s_)));
    path.joint_trajectory.points = { start, end };

    MoveGroup::Plan plan;
    plan.trajectory = std::move(path);
    if (hand.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "%s: the hand would not execute", what.c_str());
        return false;
    }
    return true;
}

bool G1ManipulationServer::moveHandTo(MoveGroup& hand, const std::string& named_target)
{
    const std::map<std::string, double> targets = hand.getNamedTargetValues(named_target);
    if (targets.empty())
    {
        RCLCPP_ERROR(
            get_logger(),
            "'%s' is not a named pose of group '%s'",
            named_target.c_str(),
            hand.getName().c_str());
        return false;
    }
    return commandHand(hand, targets, named_target);
}

bool G1ManipulationServer::moveHandToFraction(
    MoveGroup& hand, double fraction, const std::string& what)
{
    if (!std::isfinite(fraction))
    {
        RCLCPP_ERROR(get_logger(), "%s: refusing a non-finite hand fraction", what.c_str());
        return false;
    }
    const double                        f       = std::clamp(fraction, 0.0, 1.0);
    const std::map<std::string, double> targets = handTargetsAt(hand, f);
    if (targets.empty())
    {
        RCLCPP_ERROR(get_logger(), "%s needs both 'open' and 'closed'", hand.getName().c_str());
        return false;
    }
    if (!commandHand(hand, targets, what))
    {
        return false;
    }
    last_close_fraction_ = f;
    return true;
}

void G1ManipulationServer::holdCurrentGrip(MoveGroup& hand)
{
    // From /joint_states, not MoveGroup's cached state, which lags the close and would reopen it.
    const std::vector<std::string>&     joints  = hand.getActiveJoints();
    const std::optional<JointSample>    stalled = readJoints(joints);
    const std::map<std::string, double> open    = handTargetsAt(hand, 0.0);
    const std::map<std::string, double> closed  = handTargetsAt(hand, 1.0);

    std::map<std::string, double> targets;
    for (std::size_t i = 0; stalled && i < joints.size(); ++i)
    {
        const auto o = open.find(joints[i]);
        const auto c = closed.find(joints[i]);
        if (o == open.end() || c == closed.end())
        {
            break;
        }
        // Signed per joint: the thumb closes the other way on each hand.
        const double toward = c->second > o->second ? 1.0 : -1.0;
        const double biased = stalled->position[i] + (toward * grip_hold_bias_rad_);
        targets[joints[i]] =
            toward > 0.0 ? std::min(biased, c->second) : std::max(biased, c->second);
    }
    if (targets.size() != joints.size() || !commandHand(hand, targets, "hold"))
    {
        RCLCPP_WARN(
            get_logger(),
            "could not re-command the grip; the fingers keep the close's targets");
    }
}

bool G1ManipulationServer::waitForHandToSettle(MoveGroup& hand)
{
    const std::vector<std::string>& joints = hand.getActiveJoints();
    const double                    still_rad =
        grip_settled_speed_rad_s_ * std::chrono::duration<double>(kHandSamplePeriod).count();
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(hand_close_s_));
    std::optional<JointSample> before = readJoints(joints);
    while (before && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(kHandSamplePeriod);
        std::optional<JointSample> after = readJoints(joints);
        if (!after)
        {
            return false;
        }
        double moved = 0.0;
        for (std::size_t i = 0; i < joints.size(); ++i)
        {
            moved = std::max(moved, std::abs(after->position[i] - before->position[i]));
        }
        if (moved < still_rad)
        {
            return true;
        }
        before = std::move(after);
    }
    return false;
}

bool G1ManipulationServer::closeHandOn(
    MoveGroup& hand, const ArmContext& arm, double widest_m, double narrowest_m)
{
    const double range       = hand_span_open_m_ - hand_span_closed_m_;
    const auto   fraction_at = [&](double span) {
        return range > 0.0 ? std::clamp((hand_span_open_m_ - span) / range, 0.0, 1.0) : 1.0;
    };
    // Start outside the widest side, since perception tends to measure an object short, and never
    // wider than the hand takes.
    const double start =
        fraction_at(std::min(widest_m, grip_max_width_m_) + grip_start_margin_m_ - grip_preload_m_);
    // Stop a little past the narrowest side: with no contact by then the object is not between
    // the fingers, and closing further only sweeps it away.
    const double stop_span = std::max(hand_span_closed_m_, narrowest_m - grip_search_beyond_m_);
    const double stop      = std::max(start, fraction_at(stop_span));
    RCLCPP_INFO(
        get_logger(),
        "closing for an object %.0f to %.0f mm across, from %.0f%% of the way to closed, then "
        "feeling for contact",
        1000.0 * narrowest_m,
        1000.0 * widest_m,
        100.0 * start);

    // Nothing can feel contact without the grip check, so close straight to the object's size.
    if (!grip_check_enabled_)
    {
        return moveHandToFraction(hand, stop, "grasp");
    }
    const double step  = range > 0.0 ? grip_search_step_m_ / range : 0.0;
    const int    steps = step > 0.0 ? static_cast<int>(std::ceil((stop - start) / step)) : 0;
    std::string  why;
    for (int i = 0; i <= steps; ++i)
    {
        const double f = std::min(stop, start + (i * step));
        if (!moveHandToFraction(hand, f, "grasp"))
        {
            return false;
        }
        // A finger still travelling reads exactly like a blocked one.
        if (!waitForHandToSettle(hand))
        {
            RCLCPP_WARN(get_logger(), "the fingers had not stopped after %.1f s", hand_close_s_);
            continue;
        }
        // The thumb has to be among them: index and middle side by side oppose nothing.
        if (isHolding(arm, why, /*opposed=*/true))
        {
            RCLCPP_INFO(
                get_logger(),
                "contact at %.0f%% of the way to closed: %s",
                100.0 * f,
                why.c_str());
            // No squeeze past contact: on a round object it ejects rather than holds.
            holdCurrentGrip(hand);
            return true;
        }
    }
    RCLCPP_WARN(
        get_logger(),
        "no thumb-and-finger contact by %.0f mm, the object's size plus the search slop: %s",
        1000.0 * stop_span,
        why.c_str());
    return true;
}

}  // namespace g1_manipulation
