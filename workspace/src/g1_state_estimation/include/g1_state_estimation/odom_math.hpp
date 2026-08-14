#ifndef G1_STATE_ESTIMATION__ODOM_MATH_HPP_
#define G1_STATE_ESTIMATION__ODOM_MATH_HPP_

/**
 * @file odom_math.hpp
 * @brief Frame and staleness math for the odom -> base publisher.
 *
 * ROS-free so it is testable without a node, DDS or a running sim, same split as
 * g1_motion_service_sim's blend_math and g1_hardware_interface's arm_ramp_engine.
 */

#include <array>
#include <cstddef>
#include <string>

namespace g1_state_estimation
{

/// Where the base pose comes from. Anything else is a configuration error.
enum class OdometrySource
{
    kSimSportModeState,  ///< The converged track: pelvis pose from /sportmodestate. Sim-only.
    kFastLio,            ///< LiDAR-inertial odometry. The only source that runs on the robot.
    kHardware,           ///< Not a source: the real G1 publishes no odometry of its own.
};

/**
 * @brief Parses the `odometry_source` parameter.
 *
 * @param name   Parameter value, expected `sim_sportmodestate`, `fast_lio` or `hardware`.
 * @param[out] out  Set only when the name is recognised.
 * @return False for an unrecognised name, so the caller can fail configure rather than
 *         silently fall back to a default that might fabricate transforms.
 */
bool parseOdometrySource(const std::string& name, OdometrySource& out);

/// Planar pose of the base frame in the odom frame.
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

/// A rigid transform, in the same plain types as the rest of this header.
struct Pose3d
{
    double     x = 0.0;
    double     y = 0.0;
    double     z = 0.0;
    Quaternion q;
};

/**
 * @brief Whether a pose can safely be turned into a transform.
 *
 * Finite translation, and a quaternion whose norm is far enough from zero to normalise. A
 * scan-matching filter that diverges reports NaN rather than stopping, and tf2 normalises
 * silently -- so an unchecked NaN becomes a dropped transform with no message naming the
 * source. Worse at the origin latch, where one bad sample would be baked in for the whole run.
 */
bool isUsablePose(const Pose3d& pose);

/// The transform you get by applying @p b and then @p a. Reads left-to-right as frames:
/// composePose(a_from_b, b_from_c) is a_from_c.
Pose3d composePose(const Pose3d& a, const Pose3d& b);

/// The inverse transform: invertPose(a_from_b) is b_from_a.
Pose3d invertPose(const Pose3d& pose);

/**
 * @brief Yaw to a quaternion about +z.
 *
 * @param yaw  Rotation about +z, in radians.
 */
Quaternion yawToQuaternion(double yaw);

/**
 * @brief Heading about +z -- the ZYX yaw, valid under roll and pitch.
 *
 * Round-trip inverse of yawToQuaternion(); result is wrapped to (-pi, pi].
 *
 * The general form matters on the converged track. The short `2*atan2(z, w)` is exact only for
 * a pure +z rotation, and a walking G1 is never that.
 */
double quaternionToYaw(const Quaternion& q);

/**
 * @brief Angle between the body's +z and the world's +z, in radians.
 *
 * Roll and pitch together, without picking an Euler convention. Used to refuse a ground
 * projection whose heading has gone ill-conditioned: approaching 90 degrees of pitch the yaw
 * swings wildly for tiny attitude changes, and that is a fall rather than a navigation state.
 * Deliberately conservative -- it also trips on pure roll, where the yaw is still fine.
 */
double tiltFromVertical(const Quaternion& q);

/**
 * @brief A 6-DoF pose split into its REP-105 ground projection and the residual tilt.
 *
 * Nav2 and slam_toolbox both want a gravity-aligned, ground-projected base frame; the robot's
 * own root link pitches with the gait. Splitting here yields both as one chain
 * (odom -> footprint -> body) rather than two independent edges off odom that could disagree.
 */
struct GroundSplit
{
    /// odom -> footprint. Gravity-aligned by construction, so z is always 0.
    PlanarPose footprint;
    /// footprint -> body translation. Purely vertical: the yaw rotation cancels the x/y.
    double child_z = 0.0;
    /// footprint -> body rotation, Rz(-yaw) * q. Carries roll and pitch, no heading.
    Quaternion tilt;
};

/**
 * @brief Splits a pose about a given heading.
 *
 * @param x,y,z  Body origin in the parent frame.
 * @param q      Body orientation in the parent frame.
 * @param yaw    Heading to project about. Normally quaternionToYaw(q); passed separately so a
 *               caller mid-fall can hold the last well-conditioned heading instead.
 */
GroundSplit splitGroundProjection(double x, double y, double z, const Quaternion& q, double yaw);

/**
 * @brief Recombines a heading with a tilt: the inverse of splitGroundProjection's split.
 *
 * composeAttitude(yaw, splitGroundProjection(..., q, yaw).tilt) reproduces q. Used to build an
 * attitude from two sources -- heading from one, roll and pitch from another -- which is what
 * the fast_lio source does to keep the published frame gravity-true.
 */
Quaternion composeAttitude(double yaw, const Quaternion& tilt);

/// @p a composed with @p b: the rotation you get by applying @p b and then @p a.
Quaternion composeRotation(const Quaternion& a, const Quaternion& b);

/// The inverse rotation. Assumes a unit quaternion, which everything here maintains.
Quaternion invertRotation(const Quaternion& q);

/**
 * @brief Moves @p from a fraction @p t of the way toward @p to along the shortest arc.
 *
 * Used to low-pass a correction rather than apply it whole. t is clamped to [0, 1]; the
 * shorter arc is taken, so a correction never spins the long way round.
 */
Quaternion slerp(const Quaternion& from, const Quaternion& to, double t);

/**
 * @brief Wraps an angle to (-pi, pi].
 *
 * The yaw joint is a continuous hinge, so its position grows without bound as the base
 * spins. Publishing that raw into a quaternion is harmless, but comparing two of them is
 * not, hence one wrap in one place.
 */
double wrapAngle(double angle);

/**
 * @brief Rotates a world-frame planar twist into the base frame.
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
