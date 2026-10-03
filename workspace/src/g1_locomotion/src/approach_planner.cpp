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
           limits.min_forward_m < limits.target_x_m - limits.forward_tolerance_m &&
           limits.settle_slack_m >= 0.0 && limits.nudge_band_m >= 0.0;
}

bool gaitLimitsAreUsable(const GaitLimits& gait)
{
    return gait.min_speed_x_mps > 0.0 && gait.min_speed_y_mps > 0.0 &&
           gait.min_speed_x_mps <= gait.max_speed_x_mps &&
           gait.min_speed_y_mps <= gait.max_speed_y_mps && gait.max_yaw_rate_rps > 0.0 &&
           gait.speed_per_m > 0.0 && gait.yaw_rate_per_rad > 0.0 && gait.nudge_y_s > 0.0;
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

    // The only terminal state. Merely past the window is recoverable: the gait reverses.
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

    // No floor: yaw has no deadband, and a floor would swing the robot past square.
    if (std::abs(heading_error_rad) > limits.heading_tolerance_rad)
    {
        command.yaw_rate_rps = std::clamp(
            heading_error_rad * gait.yaw_rate_per_rad,
            -gait.max_yaw_rate_rps,
            gait.max_yaw_rate_rps);
    }
    return command;
}

bool settledInReach(double object_x_m, double object_y_m, const ApproachLimits& limits)
{
    const double forward = object_x_m - limits.target_x_m;
    return object_x_m >= limits.min_forward_m && forward >= -limits.forward_tolerance_m &&
           forward <= limits.forward_tolerance_m + limits.settle_slack_m &&
           std::abs(object_y_m - limits.target_y_m) <= limits.lateral_tolerance_m;
}

std::optional<Nudge>
planNudge(double object_x_m, double object_y_m, const ApproachLimits& limits, const GaitLimits& gait)
{
    if (!limitsAreUsable(limits) || !gaitLimitsAreUsable(gait) ||
        settledInReach(object_x_m, object_y_m, limits))
    {
        return std::nullopt;
    }
    const double lateral = object_y_m - limits.target_y_m;
    const double out_y   = std::abs(lateral) - limits.lateral_tolerance_m;
    // In reach ahead once the sideways error is gone, and close enough sideways for a pulse.
    if (!settledInReach(object_x_m, limits.target_y_m, limits) || out_y > limits.nudge_band_m)
    {
        return std::nullopt;
    }
    return Nudge{ std::copysign(gait.min_speed_y_mps, lateral), gait.nudge_y_s };
}

}  // namespace g1_locomotion
