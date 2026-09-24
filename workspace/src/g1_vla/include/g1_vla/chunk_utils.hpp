#ifndef G1_VLA__CHUNK_UTILS_HPP_
#define G1_VLA__CHUNK_UTILS_HPP_

/**
 * @file chunk_utils.hpp
 * @brief Kinematic checks and the controller split over one chunk of policy output, as free
 *        functions so the gate's arithmetic is unit-testable.
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
 * @brief Whether a chunk is shaped like a trajectory: names and points non-empty, every point as
 *        wide as the names, positions finite, and time_from_start positive and strictly
 *        increasing. The other checks assume this passed.
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
 * @brief Largest per-joint gap between the measured pose and the chunk's first waypoint, which
 *        catches a policy that misread the state.
 *
 * @return nullopt if a joint in the chunk was not measured.
 */
[[nodiscard]] std::optional<double>
startJump(const trajectory_msgs::msg::JointTrajectory& chunk, const JointMap& measured);

/**
 * @brief Largest per-joint move between consecutive waypoints. Bounds the swept space that
 *        per-waypoint collision checks never see.
 */
[[nodiscard]] double maxSegmentStep(const trajectory_msgs::msg::JointTrajectory& chunk);

/**
 * @brief Fastest segment as a fraction of that joint's limit, counting measured to first point.
 *
 * @return nullopt if a chunk joint is unmeasured or has no positive limit.
 */
[[nodiscard]] std::optional<double> maxVelocityRatio(
    const trajectory_msgs::msg::JointTrajectory& chunk, const JointMap& measured,
    const JointMap& limits);

/**
 * @brief Velocity that carries the arm from where it is now to the waypoint due after @p t.
 *
 * Closed-loop because servo integrates velocity and never looks at position: aiming from the
 * measured pose every tick keeps the arm on the validated path instead of accumulating error.
 *
 * @param measured Where the arm is right now, not where the chunk started.
 * @param min_dt Floor on the time left to the waypoint, so a tick landing on one does not
 *        divide by zero.
 * @return Empty at or past the last waypoint, which is the caller's signal to stop, or when a
 *         joint is unmeasured.
 */
[[nodiscard]] std::vector<double> trackingVelocity(
    const trajectory_msgs::msg::JointTrajectory& chunk, const JointMap& measured, double t,
    double min_dt);

}  // namespace g1_vla

#endif  // G1_VLA__CHUNK_UTILS_HPP_
