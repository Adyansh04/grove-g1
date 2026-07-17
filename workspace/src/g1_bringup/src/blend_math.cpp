#include "g1_bringup/blend_math.hpp"

#include <algorithm>

namespace g1_bringup
{

double stepEffectiveWeight(
    double previous_effective_weight, double raw_weight, bool arm_sdk_stale,
    double timeout_ramp_down_s, double dt_s)
{
    const double target   = arm_sdk_stale ? 0.0 : std::clamp(raw_weight, 0.0, 1.0);
    const double max_step = dt_s / timeout_ramp_down_s;

    double next = previous_effective_weight;
    if (target > next)
    {
        next = std::min(target, next + max_step);
    }
    else if (target < next)
    {
        next = std::max(target, next - max_step);
    }
    return std::clamp(next, 0.0, 1.0);
}

}  // namespace g1_bringup
