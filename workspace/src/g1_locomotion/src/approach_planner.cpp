/**
 * @file approach_planner.cpp
 * @brief Turns an object pose and the reach window into a base velocity the gait will honour.
 */

#include "g1_locomotion/approach_planner.hpp"

#include <algorithm>
#include <cmath>

namespace g1_locomotion
{
namespace
{

/// Proportional above the tolerance, floored at the gait's deadband, capped at the ceiling.
/// Exactly zero inside the tolerance, which is what stops the loop rather than a separate test.
double axisSpeed(double error, double tolerance, double gain, double floor_mps, double ceiling)
{
    if (std::abs(error) <= tolerance)
    {
        return 0.0;
    }
    return std::copysign(std::clamp(std::abs(error) * gain, floor_mps, ceiling), error);
}

}  // namespace

bool limitsAreUsable(const ApproachLimits& limits)
{
    return limits.target_x_m > 0.0 && limits.forward_tolerance_m > 0.0 &&
           limits.lateral_tolerance_m > 0.0 && limits.heading_tolerance_rad > 0.0 &&
           limits.min_forward_m >= 0.0 &&
           limits.min_forward_m < limits.target_x_m - limits.forward_tolerance_m;
}

bool gaitLimitsAreUsable(const GaitLimits& gait)
{
    return gait.min_speed_x_mps > 0.0 && gait.min_speed_y_mps > 0.0 &&
           gait.min_speed_x_mps <= gait.max_speed_x_mps &&
           gait.min_speed_y_mps <= gait.max_speed_y_mps && gait.max_yaw_rate_rps > 0.0 &&
           gait.speed_per_m > 0.0 && gait.yaw_rate_per_rad > 0.0;
}

ApproachCommand planApproach(
    double object_x_m, double object_y_m, double heading_error_rad, const ApproachLimits& limits,
    const GaitLimits& gait)
{
    ApproachCommand command;
    if (!limitsAreUsable(limits) || !gaitLimitsAreUsable(gait))
    {
        return command;
    }

    command.forward_error_m = object_x_m - limits.target_x_m;
    command.lateral_error_m = object_y_m - limits.target_y_m;

    // The only terminal state: the object under the robot's own shell, where no walk helps.
    // Merely being past the window is recoverable, because the gait reverses as readily as it
    // advances: measured -0.140 m/s at a commanded -0.20.
    if (object_x_m < limits.min_forward_m)
    {
        command.state = ApproachState::kOvershot;
        return command;
    }

    command.vx_mps = axisSpeed(
        command.forward_error_m,
        limits.forward_tolerance_m,
        gait.speed_per_m,
        gait.min_speed_x_mps,
        gait.max_speed_x_mps);
    command.vy_mps = axisSpeed(
        command.lateral_error_m,
        limits.lateral_tolerance_m,
        gait.speed_per_m,
        gait.min_speed_y_mps,
        gait.max_speed_y_mps);

    if (command.vx_mps == 0.0 && command.vy_mps == 0.0)
    {
        command.state = ApproachState::kArrived;
        return command;
    }
    command.state = ApproachState::kClosing;

    // No floor here: yaw has no deadband, so a small correction actually lands, and flooring it
    // would swing the robot past square for a couple of degrees of error.
    if (std::abs(heading_error_rad) > limits.heading_tolerance_rad)
    {
        command.yaw_rate_rps = std::clamp(
            heading_error_rad * gait.yaw_rate_per_rad,
            -gait.max_yaw_rate_rps,
            gait.max_yaw_rate_rps);
    }
    return command;
}

}  // namespace g1_locomotion
