#include "g1_locomotion/approach_planner.hpp"

#include <algorithm>
#include <cmath>

namespace g1_locomotion
{

namespace
{

double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

}  // namespace

bool limitsAreUsable(const ApproachLimits& limits)
{
    return limits.target_range_m > 0.0 && limits.range_tolerance_m > 0.0 &&
           limits.bearing_tolerance_rad > 0.0 && limits.pulse_advance_m > 0.0 &&
           limits.max_oblique_rad > 0.0 && limits.max_oblique_rad < M_PI_2 &&
           limits.min_range_m >= 0.0 &&
           limits.min_range_m < limits.target_range_m - limits.range_tolerance_m;
}

ApproachCommand planApproach(double range_m, double bearing_rad, const ApproachLimits& limits)
{
    ApproachCommand command;
    if (!limitsAreUsable(limits))
    {
        return command;
    }

    command.remaining_m = range_m - limits.target_range_m;

    // Checked before anything else. Every other branch assumes there is still room to walk into.
    if (range_m < limits.min_range_m)
    {
        command.move = ApproachMove::kOvershot;
        return command;
    }

    const double align      = wrap(bearing_rad - limits.target_bearing_rad);
    const bool   on_bearing = std::abs(align) <= limits.bearing_tolerance_rad;
    const bool   in_range   = std::abs(command.remaining_m) <= limits.range_tolerance_m;

    if (in_range && on_bearing)
    {
        command.move = ApproachMove::kDone;
        return command;
    }

    // In range but pointing wrong: turn only. Stepping here would leave the window it is
    // already inside.
    if (in_range)
    {
        command.move     = ApproachMove::kTurn;
        command.turn_rad = align;
        return command;
    }

    // Past the window on the near side but not yet under the robot. Reverse does not exist, so
    // there is nothing to plan.
    if (command.remaining_m < 0.0)
    {
        command.move = ApproachMove::kOvershot;
        return command;
    }

    // A pulse advances quantum*cos(theta) along the heading. Solve for the theta that lands on
    // the window's centre, and cap it: at large theta the sideways excursion grows toward a
    // whole pulse for an advance of nearly nothing.
    if (command.remaining_m < limits.pulse_advance_m)
    {
        const double ratio  = std::clamp(command.remaining_m / limits.pulse_advance_m, 0.0, 1.0);
        command.oblique_rad = std::min(std::acos(ratio), limits.max_oblique_rad);
    }

    // Lean the oblique toward the side the object already sits on. Both signs advance equally,
    // and this one walks the robot around the object rather than away from it.
    const double lean         = std::copysign(command.oblique_rad, align != 0.0 ? align : 1.0);
    const double desired_turn = wrap(align + lean);

    if (std::abs(desired_turn) > limits.bearing_tolerance_rad)
    {
        command.move     = ApproachMove::kTurn;
        command.turn_rad = desired_turn;
        return command;
    }

    command.move = ApproachMove::kStep;
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
