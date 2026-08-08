#include "g1_locomotion/approach_planner.hpp"

#include <algorithm>
#include <cmath>

namespace g1_locomotion
{

bool limitsAreUsable(const ApproachLimits& limits)
{
    return limits.target_x_m > 0.0 && limits.forward_tolerance_m > 0.0 &&
           limits.lateral_tolerance_m > 0.0 && limits.heading_tolerance_rad > 0.0 &&
           limits.pulse_advance_m > 0.0 && limits.max_oblique_rad > 0.0 &&
           limits.max_oblique_rad < M_PI_2 && limits.min_forward_m >= 0.0 &&
           limits.min_forward_m < limits.target_x_m - limits.forward_tolerance_m;
}

ApproachCommand planApproach(
    double object_x_m, double object_y_m, double heading_error_rad, const ApproachLimits& limits)
{
    ApproachCommand command;
    if (!limitsAreUsable(limits))
    {
        return command;
    }

    command.forward_error_m = object_x_m - limits.target_x_m;
    command.lateral_error_m = object_y_m - limits.target_y_m;

    // Checked before anything else. Every other branch assumes there is still room to walk into.
    if (object_x_m < limits.min_forward_m)
    {
        command.move = ApproachMove::kOvershot;
        return command;
    }

    // Forward first. It is the only move that wants a deliberately wrong heading, and the other
    // two mostly exist to clean up after it, so resolving them first would undo the aim.
    if (std::abs(command.forward_error_m) > limits.forward_tolerance_m)
    {
        if (command.forward_error_m < 0.0)
        {
            // Past the window on the near side but not yet under the robot. There is no command
            // that moves the base backwards, so there is nothing to plan.
            command.move = ApproachMove::kOvershot;
            return command;
        }

        if (command.forward_error_m < limits.pulse_advance_m)
        {
            // One step advances quantum*cos(theta). Solve for the theta that lands on the
            // window's centre; cap it, because the lateral excursion it creates has to be
            // strafed out afterwards at 0.035 m a pulse.
            const double ratio =
                std::clamp(command.forward_error_m / limits.pulse_advance_m, 0.0, 1.0);
            command.oblique_rad = std::min(std::acos(ratio), limits.max_oblique_rad);
        }

        // The step also moves quantum*sin(theta) sideways. Lean it toward the side the object
        // needs the robot to move, so the excursion pays down the lateral error instead of
        // adding to it. Both signs advance identically, so this is free.
        command.oblique_rad =
            std::copysign(command.oblique_rad, command.lateral_error_m >= 0.0 ? 1.0 : -1.0);
        command.move = ApproachMove::kStep;
        return command;
    }

    // Heading before lateral: a strafe taken on the wrong heading moves in the wrong world
    // direction, and the last forward step deliberately left the heading off by its oblique.
    if (std::abs(heading_error_rad) > limits.heading_tolerance_rad)
    {
        command.move     = ApproachMove::kTurn;
        command.turn_rad = heading_error_rad;
        return command;
    }

    if (std::abs(command.lateral_error_m) > limits.lateral_tolerance_m)
    {
        command.move         = ApproachMove::kStrafe;
        command.lateral_sign = command.lateral_error_m > 0.0 ? 1.0 : -1.0;
        return command;
    }

    command.move = ApproachMove::kDone;
    return command;
}

double maxObservedAdvance(double current_estimate_m, double observed_m)
{
    // A pulse that produced nothing says the gait failed to start, not that it is capable of
    // zero -- feeding that in would shrink the estimate and make the next oblique overshoot.
    if (!(observed_m > 0.0))
    {
        return current_estimate_m;
    }
    return std::max(current_estimate_m, observed_m);
}

}  // namespace g1_locomotion
