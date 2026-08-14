#ifndef G1_BRINGUP__BLEND_MATH_HPP_
#define G1_BRINGUP__BLEND_MATH_HPP_

/**
 * @file blend_math.hpp
 * @brief Motor index layout and blend-weight math for the arm_sdk sim bridge's /lowcmd assembly.
 */

#include <algorithm>
#include <array>
#include <cstddef>

#include "unitree_hg/msg/low_cmd.hpp"

namespace g1_motion_service_sim
{

/**
 * @brief Motor index layout shared with g1_hardware_interface and Unitree's own G1 examples.
 *
 * Legs 0-11, waist 12-14, arms 15-28, weight slot 29 (see unitree_ros2's
 * example/src/include/g1/g1.hpp, G1Arm7JointIndex).
 */
inline constexpr int kNumLegMotors  = 12;
inline constexpr int kFirstArmMotor = 15;
inline constexpr int kNumArmMotors  = 14;
/**
 * @brief 29 total, matching the sim's G1 MJCF.
 *
 * 29-DoF, no hands.
 */
inline constexpr int         kNumBodyMotors    = kFirstArmMotor + kNumArmMotors;
inline constexpr std::size_t kWeightMotorIndex = 29;

/**
 * @brief Linear blend between the captured hold value and the commanded value.
 *
 * Weight is clamped to [0, 1] -- used identically for arm q, kp, and kd (see
 * motion_service_sim_node's README section for the emulated motion-service
 * contract). Header-only: trivial and called on every motor slot, every
 * tick.
 *
 * @param hold_value       Captured hold value.
 * @param commanded_value  Commanded value.
 * @param weight           Blend weight, clamped to [0, 1].
 * @return The blended value.
 */
inline double blend(double hold_value, double commanded_value, double weight) noexcept
{
    weight = std::clamp(weight, 0.0, 1.0);
    return hold_value * (1.0 - weight) + commanded_value * weight;
}

/**
 * @brief Bridge-side staleness policy for the /arm_sdk blend weight.
 *
 * This is BRIDGE policy, not vendor semantics (see README: the real motion
 * service's behavior on a silent publisher at weight 1 is unverified, a
 * hardware re-validation item). Slews the effective weight toward
 * `raw_weight` when `arm_sdk_stale` is false, or toward 0 when true, at a
 * fixed rate of 1 / timeout_ramp_down_s per second either way -- so a
 * staleness episode decays smoothly and a fresh message afterwards resumes
 * from wherever the weight currently sits rather than snapping to it.
 *
 * @param previous_effective_weight  Effective weight from the previous tick.
 * @param raw_weight                 Latest raw /arm_sdk blend weight.
 * @param arm_sdk_stale              Whether the /arm_sdk publisher is currently stale.
 * @param timeout_ramp_down_s        Ramp-down time constant, in seconds.
 * @param dt_s                       Elapsed time since the previous tick, in seconds.
 * @return The new effective weight for this tick.
 */
[[nodiscard]] double stepEffectiveWeight(
    double previous_effective_weight, double raw_weight, bool arm_sdk_stale,
    double timeout_ramp_down_s, double dt_s) noexcept;

/**
 * @brief Assembles a full-body /lowcmd from the frozen hold pose and the latest /arm_sdk command.
 *
 * Legs (0-11) + waist (12-14) take `lower_q` with per-joint `lower_kp`/`lower_kd` -- either the
 * frozen hold pose at stiff-hold gains, or the walking policy's targets at its own gains; the
 * caller picks, so the leg-authority decision stays visible at the call site instead of hiding in
 * here. Arms (kFirstArmMotor..) are blended between
 * `hold_q` and the commanded arm targets at `weight` via blend() (on q, kp,
 * and kd alike), and the weight slot echoes `weight` back out.
 * mode/mode_pr/mode_machine are left at zero -- see motion_service_sim_node's
 * README section for why. Free function (not a member) so the slot/gain
 * assembly is unit-testable without a live node or DDS, mirroring
 * g1_hardware_interface's assembleLowCmd().
 *
 * @param hold_q       Frozen hold pose, supplying the arm slots' blend-toward value.
 * @param lower_q      Position targets for the legs and waist.
 * @param lower_kp     Per-joint position gains for the legs and waist.
 * @param lower_kd     Per-joint velocity gains for the legs and waist.
 * @param arm_cmd_q    Commanded arm joint positions from /arm_sdk.
 * @param arm_cmd_kp   Commanded arm position gains from /arm_sdk.
 * @param arm_cmd_kd   Commanded arm velocity gains from /arm_sdk.
 * @param weight       Arm blend weight passed to blend() for q, kp, and kd.
 * @param arm_hold_kp  Hold-side position gain blended for the arm motors.
 * @param arm_hold_kd  Hold-side velocity gain blended for the arm motors.
 * @return The assembled /lowcmd message.
 */
unitree_hg::msg::LowCmd assembleSimLowCmd(
    const std::array<double, kNumBodyMotors>& hold_q,
    const std::array<double, kFirstArmMotor>& lower_q,
    const std::array<double, kFirstArmMotor>& lower_kp,
    const std::array<double, kFirstArmMotor>& lower_kd,
    const std::array<double, kNumArmMotors>&  arm_cmd_q,
    const std::array<double, kNumArmMotors>&  arm_cmd_kp,
    const std::array<double, kNumArmMotors>& arm_cmd_kd, double weight, double arm_hold_kp,
    double arm_hold_kd) noexcept;

}  // namespace g1_motion_service_sim

#endif  // G1_BRINGUP__BLEND_MATH_HPP_
