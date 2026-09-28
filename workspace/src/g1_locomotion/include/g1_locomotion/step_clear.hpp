#ifndef G1_LOCOMOTION__STEP_CLEAR_HPP_
#define G1_LOCOMOTION__STEP_CLEAR_HPP_

/**
 * @file step_clear.hpp
 * @brief The straight step that takes the base out of Nav2's collision band.
 *
 * Nav2 checks a circle wider than the body, and refuses every motion once an obstacle is inside
 * it. The body itself is still clear, so a short straight step away is safe: this picks the one
 * that gets clear soonest, forward or backward, turning as little as it can.
 */

#include <span>

namespace g1_locomotion
{

/// An obstacle cell's centre, in the frame the base pose is given in, m.
struct ObstaclePoint
{
    double x = 0.0;
    double y = 0.0;
};

struct StepClearLimits
{
    /// Nothing may come nearer the base than this on the way out: the body with its arms.
    double body_radius_m = 0.30;

    /// The longest step considered. Further out is a job for Nav2 again.
    double max_step_m = 0.80;

    /// Walking worth one radian of turning: turns drift the gait, so a longer step can be better.
    double turn_cost_m_per_rad = 0.20;
};

struct StepClearPlan
{
    bool found = false;

    /// The direction to step in, rad, in the pose's frame.
    double direction = 0.0;

    /// Step backwards: face away from @c direction. The gait reverses as readily as it walks.
    bool reverse = false;

    /// How far to step; 0 when already clear.
    double distance_m = 0.0;
};

/**
 * @brief Distance from (@p x, @p y) to the nearest obstacle.
 * @return Infinity when there are none.
 */
double clearanceAt(std::span<const ObstaclePoint> obstacles, double x, double y);

/**
 * @brief The straight step that brings the clearance up to @p wanted_m soonest.
 *
 * On the way the clearance may not drop below the body radius, or below where it started when
 * the robot is already nearer than that.
 *
 * @return found false when no step within max_step_m gets clear.
 */
StepClearPlan planStepClear(
    std::span<const ObstaclePoint> obstacles, double x, double y, double yaw, double wanted_m,
    const StepClearLimits& limits);

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__STEP_CLEAR_HPP_
