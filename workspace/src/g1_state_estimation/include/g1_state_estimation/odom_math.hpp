#ifndef G1_STATE_ESTIMATION__ODOM_MATH_HPP_
#define G1_STATE_ESTIMATION__ODOM_MATH_HPP_

/**
 * @file odom_math.hpp
 * @brief Frame and staleness math for the odom -> base_link publisher.
 *
 * ROS-free so it is testable without a node, DDS or a running sim, same split as
 * g1_bringup's blend_math and g1_hardware_interface's arm_ramp_engine.
 */

#include <array>
#include <cstddef>
#include <string>

namespace g1_state_estimation
{

/// Where the base pose comes from. Anything else is a configuration error.
enum class OdometrySource
{
    SimGroundTruth,     ///< MuJoCo generalized coordinates via planar joints. Sim-only.
    SimSportModeState,  ///< The converged track: pelvis pose from /sportmodestate. Sim-only.
    Hardware,           ///< Not implemented: the real G1 publishes no odometry at all.
};

/**
 * @brief Parses the `odometry_source` parameter.
 *
 * @param name   Parameter value, expected `sim_ground_truth` or `hardware`.
 * @param[out] out  Set only when the name is recognised.
 * @return False for an unrecognised name, so the caller can fail configure rather than
 *         silently fall back to a default that might fabricate transforms.
 */
bool parseOdometrySource(const std::string& name, OdometrySource& out);

/// Planar pose of base_link in the odom frame.
struct PlanarPose
{
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

/// Planar twist. Frame depends on context, see toBodyTwist().
struct PlanarTwist
{
    double vx    = 0.0;
    double vy    = 0.0;
    double omega = 0.0;
};

/// Quaternion, w-last to match geometry_msgs.
struct Quaternion
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

/**
 * @brief Yaw to a quaternion about +z.
 *
 * @param yaw  Rotation about +z, in radians.
 */
Quaternion yawToQuaternion(double yaw);

/**
 * @brief Recovers yaw from a quaternion assumed to be a pure +z rotation.
 *
 * Round-trip inverse of yawToQuaternion(); result is wrapped to (-pi, pi].
 */
double quaternionToYaw(const Quaternion& q);

/**
 * @brief Wraps an angle to (-pi, pi].
 *
 * The yaw joint is a continuous hinge, so its position grows without bound as the base
 * spins. Publishing that raw into a quaternion is harmless, but comparing two of them is
 * not, hence one wrap in one place.
 */
double wrapAngle(double angle);

/**
 * @brief Rotates a world-frame planar twist into base_link.
 *
 * nav_msgs/Odometry defines `twist` in the child frame, not the header frame, which is a
 * standing trap: the planar joints report velocity in the world frame, so handing it
 * straight to the message is wrong for every yaw except zero. Nav2's controller server
 * reads this, so getting it wrong shows up as the robot fighting its own heading.
 *
 * @param world_twist  Twist expressed in the odom frame.
 * @param yaw          Current base yaw in the odom frame.
 */
PlanarTwist toBodyTwist(const PlanarTwist& world_twist, double yaw);

/**
 * @brief Whether the source is too old to keep publishing transforms from.
 *
 * Compares elapsed against the timeout inclusively, so a sample exactly at the timeout is
 * NOT yet stale; the boundary is pinned by test because "off by one tick" here means
 * either a spurious TF gap or a transform that outlives its data.
 *
 * @param elapsed_s    Seconds since the last accepted sample.
 * @param timeout_s    Configured tolerance. Non-positive disables the check.
 */
bool isStale(double elapsed_s, double timeout_s);

/**
 * @brief Fills a 6x6 row-major covariance with a single value on the diagonal.
 *
 * Ground truth has no meaningful uncertainty, but an all-zero covariance is a known Nav2
 * footgun, so a small non-zero diagonal is written instead of leaving it empty.
 *
 * @param value  Written to all six diagonal entries; off-diagonals are zeroed.
 */
std::array<double, 36> diagonalCovariance(double value);

}  // namespace g1_state_estimation

#endif  // G1_STATE_ESTIMATION__ODOM_MATH_HPP_
