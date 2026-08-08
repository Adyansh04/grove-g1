#include "g1_locomotion/approach_planner.hpp"

#include <algorithm>
#include <cmath>

namespace g1_locomotion
{

bool limitsAreUsable(const ApproachLimits& limits)
{
    return limits.target_x_m > 0.0 && limits.forward_tolerance_m > 0.0 &&
           limits.lateral_tolerance_m > 0.0 && limits.heading_tolerance_rad > 0.0 &&
           limits.step_threshold_m > 0.0 && limits.fine_offset_rad > 0.0 &&
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

    // Only a physical overshoot is terminal now: the object under the robot's own shell, where
    // no arm motion helps. Merely being past the window is recoverable, because a creep run the
    // other way BACKS THE ROBOT UP -- see below.
    if (object_x_m < limits.min_forward_m)
    {
        command.move = ApproachMove::kOvershot;
        return command;
    }

    // Coarse. One step covers 0.29 to 0.50 m depending on how warm the gait is, so this cannot
    // be exact; it only has to answer "is a step likely to help". Creeping covers 2.5 cm a
    // pulse, so guessing wrong by a step costs a dozen creeps and guessing wrong the other way
    // costs a creep backwards.
    if (command.forward_error_m > limits.step_threshold_m)
    {
        command.move = ApproachMove::kStep;
        return command;
    }

    // Fine, and in EITHER direction. Not a smaller forward step, because there is no such thing:
    // forward is irreducible at about 0.29 m however short the pulse.
    //
    // A strafe taken at offset phi with sign s displaces the robot (-s*L*sin(phi), s*L*cos(phi))
    // in the frame it started in. Two things fall out of that. The lateral part is s*L*cos(phi),
    // which for |phi| < 90 always takes the sign of s, so s is chosen by the lateral error. And
    // the forward part is -s*L*sin(phi), whose sign is free -- flipping phi backs the robot up.
    //
    // That last point is why this gait is not actually reverse-less. g1_gait_shaper zeroes any
    // negative vx, so nothing can walk backwards, but a strafe on a turned heading moves
    // backwards perfectly well. It is what makes overshooting the window recoverable instead of
    // fatal, which the first creeping run needed and did not have: one creep carried the robot
    // 0.34 m, well past the target, and there was nothing to be done about it.
    if (std::abs(command.forward_error_m) > limits.forward_tolerance_m)
    {
        const double lateral_sign = command.lateral_error_m >= 0.0 ? 1.0 : -1.0;
        const double forward_sign = command.forward_error_m >= 0.0 ? 1.0 : -1.0;
        command.lateral_sign      = lateral_sign;
        command.fine_offset_rad   = -forward_sign * lateral_sign * limits.fine_offset_rad;
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

}  // namespace g1_locomotion
