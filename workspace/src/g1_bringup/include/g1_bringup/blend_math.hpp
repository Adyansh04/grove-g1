#ifndef G1_BRINGUP__BLEND_MATH_HPP_
#define G1_BRINGUP__BLEND_MATH_HPP_

#include <algorithm>
#include <array>
#include <cstddef>

#include "unitree_hg/msg/low_cmd.hpp"

namespace g1_bringup
{

// Motor index layout shared with g1_hardware_interface and Unitree's own G1
// examples: legs 0-11, waist 12-14, arms 15-28, weight slot 29 (see
// unitree_ros2's example/src/include/g1/g1.hpp, G1Arm7JointIndex).
inline constexpr int kNumLegMotors  = 12;
inline constexpr int kFirstArmMotor = 15;
inline constexpr int kNumArmMotors  = 14;
// 29 total, matching the sim's G1 MJCF (29-DoF, no hands -- confirmed in the
// milestone-1 spike).
inline constexpr int         kNumBodyMotors    = kFirstArmMotor + kNumArmMotors;
inline constexpr std::size_t kWeightMotorIndex = 29;

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

// Assembles a full-body /lowcmd from the frozen hold pose and the latest
// /arm_sdk command: legs (0-11) + waist (12-14) stiff-held at `hold_q`'s
// value with the given per-group gains, arms (kFirstArmMotor..) blended
// between `hold_q` and the commanded arm targets at `weight` via blend()
// (on q, kp, and kd alike), and the weight slot echoing `weight` back out.
// mode/mode_pr/mode_machine are left at zero -- see arm_sdk_sim_bridge_node's
// README section for why. Free function (not a member) so the slot/gain
// assembly is unit-testable without a live node or DDS, mirroring
// g1_hardware_interface's assembleLowCmd().
unitree_hg::msg::LowCmd assembleSimLowCmd(
    const std::array<double, kNumBodyMotors>& hold_q,
    const std::array<double, kNumArmMotors>&  arm_cmd_q,
    const std::array<double, kNumArmMotors>&  arm_cmd_kp,
    const std::array<double, kNumArmMotors>& arm_cmd_kd, double weight, double leg_kp,
    double leg_kd, double waist_kp, double waist_kd, double arm_hold_kp, double arm_hold_kd);

}  // namespace g1_bringup

#endif  // G1_BRINGUP__BLEND_MATH_HPP_
