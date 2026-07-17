#ifndef G1_BRINGUP__BLEND_MATH_HPP_
#define G1_BRINGUP__BLEND_MATH_HPP_

#include <algorithm>

namespace g1_bringup
{

// Linear blend between the captured hold value and the commanded value,
// weight clamped to [0, 1] -- used identically for arm q, kp, and kd (see
// arm_sdk_sim_bridge_node's README section for the emulated motion-service
// contract). Header-only: trivial and called on every motor slot, every
// tick.
inline double blend(double hold_value, double commanded_value, double weight)
{
    weight = std::clamp(weight, 0.0, 1.0);
    return hold_value * (1.0 - weight) + commanded_value * weight;
}

// Bridge-side staleness policy for the /arm_sdk blend weight -- this is
// BRIDGE policy, not vendor semantics (see README: the real motion service's
// behavior on a silent publisher at weight 1 is unverified, a hardware
// re-validation item). Slews the effective weight toward `raw_weight` when
// `arm_sdk_stale` is false, or toward 0 when true, at a fixed rate of
// 1 / timeout_ramp_down_s per second either way -- so a staleness episode
// decays smoothly and a fresh message afterwards resumes from wherever the
// weight currently sits rather than snapping to it.
double stepEffectiveWeight(
    double previous_effective_weight, double raw_weight, bool arm_sdk_stale,
    double timeout_ramp_down_s, double dt_s);

}  // namespace g1_bringup

#endif  // G1_BRINGUP__BLEND_MATH_HPP_
