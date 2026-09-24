#ifndef G1_STATE_ESTIMATION__ODOM_MATH_HPP_
#define G1_STATE_ESTIMATION__ODOM_MATH_HPP_

/**
 * @file odom_math.hpp
 * @brief Frame and staleness math for the odom -> base publisher, ROS-free so it tests alone.
 */

#include <array>
#include <cstddef>
#include <string>

namespace g1_state_estimation
{

/**
 * @brief Where the base pose comes from. Anything else is a configuration error.
 */
enum class OdometrySource
{
    kGroundTruth,  ///< Exact pelvis state out of the simulator, over the relay. Sim-only.
    kFastLio,      ///< LiDAR-inertial odometry. The only source that runs on the robot.
    kHardware,     ///< Not a source: the real G1 publishes no odometry of its own.
};

/**
 * @brief Parses the `odometry_source` parameter.
 *
 * @param name   Parameter value, expected `ground_truth`, `fast_lio` or `hardware`.
 * @param[out] out  Set only when the name is recognised.
 * @return False for an unrecognised name, so configure fails instead of falling back.
 */
bool parseOdometrySource(const std::string& name, OdometrySource& out);

/**
 * @brief Planar pose of the base frame in the odom frame.
 */
struct PlanarPose
{
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

/**
 * @brief Planar twist. Frame depends on context, see toBodyTwist().
 */
struct PlanarTwist
{
    double vx    = 0.0;
    double vy    = 0.0;
    double omega = 0.0;
};

/**
 * @brief Quaternion, w-last to match geometry_msgs.
 */
struct Quaternion
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

/**
 * @brief A rigid transform, in the same plain types as the rest of this header.
 */
struct Pose3d
{
    double     x = 0.0;
    double     y = 0.0;
    double     z = 0.0;
    Quaternion q;
};

/**
 * @brief Whether a pose can become a transform.
 *
 * Finite translation, and a quaternion far enough from zero to normalise; a diverged scan match
 * reports NaN.
 */
bool isUsablePose(const Pose3d& pose);

/**
 * @brief The transform you get by applying @p b and then @p a.
 *
 * Reads left-to-right as frames: composePose(a_from_b, b_from_c) is a_from_c.
 */
Pose3d composePose(const Pose3d& a, const Pose3d& b);

/**
 * @brief The inverse transform: invertPose(a_from_b) is b_from_a.
 */
Pose3d invertPose(const Pose3d& pose);

/**
 * @brief Yaw to a quaternion about +z.
 *
 * @param yaw  Rotation about +z, in radians.
 */
Quaternion yawToQuaternion(double yaw);

/**
 * @brief Heading about +z: the ZYX yaw, valid under roll and pitch, wrapped to (-pi, pi].
 *
 * Round-trip inverse of yawToQuaternion(). `2*atan2(z, w)` is exact only with no roll or pitch.
 */
double quaternionToYaw(const Quaternion& q);

/**
 * @brief Angle between the body's +z and the world's +z, in radians.
 *
 * Roll and pitch together, for the heading guard. Conservative: pure roll counts too, although
 * yaw stays well-conditioned there.
 */
double tiltFromVertical(const Quaternion& q);

/**
 * @brief A 6-DoF pose split into its REP-105 ground projection and the residual.
 *
 * Published as one chain, odom -> footprint -> body, so the two edges cannot disagree.
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
 * @brief Recombines a heading with a tilt, the inverse of splitGroundProjection().
 *
 * composeAttitude(yaw, splitGroundProjection(..., q, yaw).tilt) reproduces q. The two parts may
 * come from different sources.
 */
Quaternion composeAttitude(double yaw, const Quaternion& tilt);

/**
 * @brief @p a composed with @p b: the rotation you get by applying @p b and then @p a.
 */
Quaternion composeRotation(const Quaternion& a, const Quaternion& b);

/**
 * @brief The inverse rotation. Assumes a unit quaternion, which everything here maintains.
 */
Quaternion invertRotation(const Quaternion& q);

/**
 * @brief Moves @p from a fraction @p t of the way toward @p to along the shorter arc.
 *
 * t is clamped to [0, 1].
 */
Quaternion slerp(const Quaternion& from, const Quaternion& to, double t);

/**
 * @brief Wraps an angle to (-pi, pi].
 */
double wrapAngle(double angle);

/**
 * @brief Rotates a world-frame planar twist into the base frame.
 *
 * nav_msgs/Odometry carries `twist` in the child frame, not the header frame.
 *
 * @param world_twist  Twist expressed in the odom frame.
 * @param yaw          Current base yaw in the odom frame.
 */
PlanarTwist toBodyTwist(const PlanarTwist& world_twist, double yaw);

/**
 * @brief Whether the source is too old to keep publishing transforms from.
 *
 * A sample exactly at the timeout is not yet stale.
 *
 * @param elapsed_s    Seconds since the last accepted sample.
 * @param timeout_s    Configured tolerance. Non-positive disables the check.
 */
bool isStale(double elapsed_s, double timeout_s);

/**
 * @brief Fills a 6x6 row-major covariance with a single value on the diagonal.
 *
 * All-zero covariance is a known Nav2 pitfall, so even exact sources publish a small diagonal.
 *
 * @param value  Written to all six diagonal entries; off-diagonals are zeroed.
 */
std::array<double, 36> diagonalCovariance(double value);

}  // namespace g1_state_estimation

#endif  // G1_STATE_ESTIMATION__ODOM_MATH_HPP_
