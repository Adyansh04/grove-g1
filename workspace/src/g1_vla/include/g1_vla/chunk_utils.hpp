#ifndef G1_VLA__CHUNK_UTILS_HPP_
#define G1_VLA__CHUNK_UTILS_HPP_

/**
 * @file chunk_utils.hpp
 * @brief Kinematic checks and the controller split, over one chunk of policy output.
 *
 * Free functions with no node behind them, so the gate's arithmetic is testable on its own.
 */

#include <map>
#include <optional>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace g1_vla
{

/// Joint positions by name, as read from /joint_states.
using JointMap = std::map<std::string, double>;

/**
 * @brief Whether a chunk is shaped like a trajectory at all.
 *
 * Names and points non-empty, every point as wide as the name list, and time_from_start
 * positive and strictly increasing. Everything below assumes this passed.
 */
[[nodiscard]] bool wellFormed(const trajectory_msgs::msg::JointTrajectory& chunk);

/**
 * @brief The part of @p chunk that a controller owning @p joints can execute.
 *
 * @return Empty joint_names when the chunk names none of them, which is a controller to skip.
 */
[[nodiscard]] trajectory_msgs::msg::JointTrajectory splitByController(
    const trajectory_msgs::msg::JointTrajectory& chunk, const std::vector<std::string>& joints);

/**
 * @brief Largest per-joint gap between the measured pose and the chunk's first waypoint.
 *
 * A policy that misread the state opens its chunk somewhere the arm is not, and executing that
 * snaps the arm across space nothing has collision-checked.
 *
 * @return nullopt if a joint in the chunk was not measured.
 */
[[nodiscard]] std::optional<double>
startJump(const trajectory_msgs::msg::JointTrajectory& chunk, const JointMap& measured);

/**
 * @brief Largest per-joint move between consecutive waypoints.
 *
 * Per-waypoint collision checking only means something while consecutive waypoints stay close:
 * the space swept between two far-apart ones is never looked at.
 */
[[nodiscard]] double maxSegmentStep(const trajectory_msgs::msg::JointTrajectory& chunk);

/**
 * @brief Fastest segment as a fraction of that joint's limit, counting measured to first point.
 *
 * @return nullopt if a chunk joint has no positive limit, meaning the model and the chunk
 *         disagree about what the robot is.
 */
[[nodiscard]] std::optional<double> maxVelocityRatio(
    const trajectory_msgs::msg::JointTrajectory& chunk, const JointMap& measured,
    const JointMap& limits);

}  // namespace g1_vla

#endif  // G1_VLA__CHUNK_UTILS_HPP_
