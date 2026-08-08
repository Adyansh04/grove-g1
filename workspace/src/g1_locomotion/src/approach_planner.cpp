#include "g1_locomotion/approach_planner.hpp"

#include <algorithm>
#include <cmath>

namespace g1_locomotion
{

bool limitsAreUsable(const ApproachLimits& limits)
{
    return limits.target_x_m > 0.0 && limits.forward_tolerance_m > 0.0 &&
           limits.lateral_tolerance_m > 0.0 && limits.heading_tolerance_rad > 0.0 &&
           limits.pulse_advance_m > 0.0 && limits.fine_offset_rad > 0.0 &&
           limits.fine_offset_rad < M_PI_2 && limits.min_forward_m >= 0.0 &&
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
    if (object_x_m < limits.min_forward_m || command.forward_error_m < -limits.forward_tolerance_m)
    {
        command.move = ApproachMove::kOvershot;
        return command;
    }

    // Coarse. The bound is deliberately quantum PLUS tolerance rather than minus: pulse_advance_m
    // is maintained as an upper bound on what a step does, so stepping only while there is more
    // than a full step to close means a straight step can never carry the robot past the window.
    // Undershooting costs one creep; overshooting cannot be undone at all.
    if (command.forward_error_m > limits.pulse_advance_m + limits.forward_tolerance_m)
    {
        command.move = ApproachMove::kStep;
        return command;
    }

    // Fine. Not a smaller forward step, because there is no such thing: forward is irreducible at
    // about 0.29 m however short the pulse. Turning off the working heading and strafing is what
    // produces a small, precise advance.
    if (command.forward_error_m > limits.forward_tolerance_m)
    {
        // Turning left and strafing right advances the robot and slides it right; turning right
        // and strafing left advances it and slides it left. So the offset is signed opposite to
        // the way the lateral error wants the robot to go, and the slide pays down that error
        // instead of adding to it.
        const double lateral_sign = command.lateral_error_m >= 0.0 ? 1.0 : -1.0;
        command.lateral_sign      = lateral_sign;
        command.fine_offset_rad   = -lateral_sign * limits.fine_offset_rad;
        command.move              = ApproachMove::kCreep;
        return command;
    }

    // Heading before lateral: a strafe taken on the wrong heading moves in the wrong world
    // direction, and a creep deliberately leaves the heading off by its offset.
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
    // zero -- feeding that in would shrink the estimate and let the next step overshoot.
    if (!(observed_m > 0.0))
    {
        return current_estimate_m;
    }
    return std::max(current_estimate_m, observed_m);
}

}  // namespace g1_locomotion
