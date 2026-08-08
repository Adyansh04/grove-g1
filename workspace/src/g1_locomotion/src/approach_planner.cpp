#include "g1_locomotion/approach_planner.hpp"

#include <algorithm>
#include <cmath>

namespace g1_locomotion
{

bool limitsAreUsable(const ApproachLimits& limits)
{
    return limits.target_x_m > 0.0 && limits.forward_tolerance_m > 0.0 &&
           limits.lateral_tolerance_m > 0.0 && limits.step_threshold_m > 0.0 &&
           limits.min_forward_m >= 0.0 &&
           limits.min_forward_m < limits.target_x_m - limits.forward_tolerance_m;
}

ApproachCommand planApproach(double object_x_m, double object_y_m, const ApproachLimits& limits)
{
    ApproachCommand command;
    if (!limitsAreUsable(limits))
    {
        return command;
    }

    command.forward_error_m = object_x_m - limits.target_x_m;
    command.lateral_error_m = object_y_m - limits.target_y_m;

    // The only terminal state: the object under the robot's own shell, where no arm motion
    // helps. Merely being past the window is recoverable, because the robot can reverse.
    if (object_x_m < limits.min_forward_m)
    {
        command.move = ApproachMove::kOvershot;
        return command;
    }

    // Forward, whatever the size. `coarse` only tells the caller whether it may stop its drive
    // early and let the gait coast, or has to drive to zero and let a reverse take back the
    // overshoot. Forward is irreducible at about 0.29 m, so a small gap is closed by deliberately
    // going too far and coming back -- reverse resolves more finely than forward does.
    if (command.forward_error_m > limits.forward_tolerance_m)
    {
        command.move   = ApproachMove::kStep;
        command.coarse = command.forward_error_m > limits.step_threshold_m;
        return command;
    }

    // Too far in: straight back.
    if (command.forward_error_m < -limits.forward_tolerance_m)
    {
        command.move = ApproachMove::kReverse;
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
