/**
 * @file chunk_utils.cpp
 * @brief Chunk shape, continuity and speed checks.
 */

#include "g1_vla/chunk_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace g1_vla
{

namespace
{

double seconds(const builtin_interfaces::msg::Duration& d)
{
    return static_cast<double>(d.sec) + (static_cast<double>(d.nanosec) * 1e-9);
}

}  // namespace

bool wellFormed(const trajectory_msgs::msg::JointTrajectory& chunk)
{
    if (chunk.joint_names.empty() || chunk.points.empty())
    {
        return false;
    }
    double previous = 0.0;
    for (const trajectory_msgs::msg::JointTrajectoryPoint& point : chunk.points)
    {
        if (point.positions.size() != chunk.joint_names.size())
        {
            return false;
        }
        const double t = seconds(point.time_from_start);
        if (!(t > previous))
        {
            return false;
        }
        previous = t;
    }
    return true;
}

trajectory_msgs::msg::JointTrajectory splitByController(
    const trajectory_msgs::msg::JointTrajectory& chunk, const std::vector<std::string>& joints)
{
    std::vector<std::size_t> keep;
    for (std::size_t i = 0; i < chunk.joint_names.size(); ++i)
    {
        if (std::find(joints.begin(), joints.end(), chunk.joint_names[i]) != joints.end())
        {
            keep.push_back(i);
        }
    }

    trajectory_msgs::msg::JointTrajectory out;
    out.header = chunk.header;
    if (keep.empty())
    {
        return out;
    }
    for (const std::size_t i : keep)
    {
        out.joint_names.push_back(chunk.joint_names[i]);
    }
    for (const trajectory_msgs::msg::JointTrajectoryPoint& point : chunk.points)
    {
        trajectory_msgs::msg::JointTrajectoryPoint kept;
        kept.time_from_start = point.time_from_start;
        for (const std::size_t i : keep)
        {
            kept.positions.push_back(point.positions[i]);
        }
        out.points.push_back(kept);
    }
    return out;
}

std::optional<double>
startJump(const trajectory_msgs::msg::JointTrajectory& chunk, const JointMap& measured)
{
    double worst = 0.0;
    for (std::size_t i = 0; i < chunk.joint_names.size(); ++i)
    {
        const auto it = measured.find(chunk.joint_names[i]);
        if (it == measured.end())
        {
            return std::nullopt;
        }
        worst = std::max(worst, std::abs(chunk.points.front().positions[i] - it->second));
    }
    return worst;
}

double maxSegmentStep(const trajectory_msgs::msg::JointTrajectory& chunk)
{
    double worst = 0.0;
    for (std::size_t p = 1; p < chunk.points.size(); ++p)
    {
        for (std::size_t i = 0; i < chunk.joint_names.size(); ++i)
        {
            worst = std::max(
                worst,
                std::abs(chunk.points[p].positions[i] - chunk.points[p - 1].positions[i]));
        }
    }
    return worst;
}

std::optional<double> maxVelocityRatio(
    const trajectory_msgs::msg::JointTrajectory& chunk, const JointMap& measured,
    const JointMap& limits)
{
    std::vector<double> previous;
    previous.reserve(chunk.joint_names.size());
    std::vector<double> limit;
    limit.reserve(chunk.joint_names.size());
    for (const std::string& name : chunk.joint_names)
    {
        const auto measured_it = measured.find(name);
        const auto limit_it    = limits.find(name);
        if (measured_it == measured.end() || limit_it == limits.end() || !(limit_it->second > 0.0))
        {
            return std::nullopt;
        }
        previous.push_back(measured_it->second);
        limit.push_back(limit_it->second);
    }

    double worst      = 0.0;
    double previous_t = 0.0;
    for (const trajectory_msgs::msg::JointTrajectoryPoint& point : chunk.points)
    {
        const double dt = seconds(point.time_from_start) - previous_t;
        for (std::size_t i = 0; i < chunk.joint_names.size(); ++i)
        {
            const double speed = std::abs(point.positions[i] - previous[i]) / dt;
            worst              = std::max(worst, speed / limit[i]);
            previous[i]        = point.positions[i];
        }
        previous_t = seconds(point.time_from_start);
    }
    return worst;
}

}  // namespace g1_vla
