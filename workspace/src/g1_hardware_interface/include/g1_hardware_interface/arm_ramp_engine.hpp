#ifndef G1_HARDWARE_INTERFACE__ARM_RAMP_ENGINE_HPP_
#define G1_HARDWARE_INTERFACE__ARM_RAMP_ENGINE_HPP_

/**
 * @file arm_ramp_engine.hpp
 * @brief Pure, ROS-free ramp/slew engine for the G1 arm_sdk blend weight and joint targets.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace g1_hardware_interface
{

/**
 * @brief Number of arm joints exported by the G1ArmSdkSystem.
 *
 * The G1ArmSdkSystem exports exactly the 14 arm joints (7 per arm: shoulder
 * pitch/roll/yaw, elbow, wrist roll/pitch/yaw). The waist rides along on /arm_sdk but is
 * held rather than ramped, so it is not this engine's concern; legs and hands stay with
 * the onboard controller.
 */
inline constexpr std::size_t kNumArmJoints = 14;

/**
 * @brief Phases of the single writer-authority state machine.
 *
 * kInactive means "not commanding" (write() self-gates on it); the other
 * three all imply active publication with the blend weight moving toward a
 * mode-specific target (1 for kActive, 0 otherwise) at a mode-specific rate.
 */
enum class BlendMode : std::uint8_t
{
    kInactive,
    kActive,
    kRampDown,
    kEmergencyRampDown,
};

/**
 * @brief Tunable parameters for ArmRampEngine's blend-weight ramps and position slew.
 */
struct RampConfig
{
    double blend_ramp_up_s{ 0.0 };
    double blend_ramp_down_s{ 0.0 };
    double emergency_ramp_down_s{ 0.0 };
    double max_joint_velocity_rad_s{ 0.0 };
    /// Reference tick period used only to bound a pathologically large dt_s
    /// passed to step() (e.g. a stalled control loop) before it's integrated
    /// -- see kMaxDtNominalPeriodMultiple. Sourced from command_publish_rate,
    /// the only period-like <param> this pure engine has any visibility into
    /// (controller_manager's own update_rate, which is what actually paces
    /// step()'s real per-tick dt, lives in controller_manager's YAML and
    /// never reaches a hardware plugin) -- a conservative proxy, not a claim
    /// that dt is normally this large.
    double nominal_period_s{ 0.0 };
};

/**
 * @brief Bound on how many nominal_period_s a single step() tick's dt may span.
 *
 * Caps a single step() tick's dt at this many nominal_period_s before
 * integrating the weight ramp and position slew, so neither can blow
 * through arbitrarily far in one tick on a pathologically large or stalled
 * period. Exposed (not just an implementation constant) so tests can pin
 * the exact bound.
 */
inline constexpr double kMaxDtNominalPeriodMultiple = 3.0;

/**
 * @brief Ramp and velocity slew calculation engine for arm blend weights and joint targets.
 */
class ArmRampEngine
{
public:
    /**
     * @brief Constructs the engine with the given ramp and slew configuration.
     * @param config  Ramp and slew configuration to use for subsequent step() calls.
     */
    explicit ArmRampEngine(const RampConfig& config);

    /**
     * @brief Hold-in-place seed, called by on_activate with the just-measured joint positions.
     *
     * Ensures the first published target matches reality exactly (no snap),
     * and zeroes the blend weight so the onboard controller isn't handed
     * anything until the ramp-up has actually started.
     * @param measured_positions  Just-measured joint positions to seed the published targets with.
     */
    void seedFromMeasured(const std::array<double, kNumArmJoints>& measured_positions) noexcept;

    /**
     * @brief One control-loop tick: advances the blend weight and slews the published targets.
     *
     * Advances the blend weight toward the target implied by `mode` (1 for
     * kActive, 0 for kRampDown/kEmergencyRampDown, at
     * blend_ramp_up_s/blend_ramp_down_s/emergency_ramp_down_s respectively)
     * and slews every joint's published target toward `commanded_positions`
     * by at most max_joint_velocity_rad_s * dt_s, starting from wherever the
     * target currently is -- never a jump. `mode` is never kInactive in
     * practice (callers self-gate before reaching this call); it's treated
     * the same as kRampDown here as a defensive fallback rather than left
     * unhandled.
     *
     * dt_s <= 0 (e.g. a system-clock step-back -- callers aren't guaranteed a
     * monotonic source) holds rather than integrates: returns the current
     * weight/positions unchanged instead of snapping, and avoids clamp()'s UB
     * on an inverted [-max_step, max_step] range from a negative dt_s. A
     * dt_s larger than kMaxDtNominalPeriodMultiple * nominal_period_s (e.g. a
     * stalled control loop) is clamped to that bound first, so neither the
     * weight ramp nor the position slew can blow arbitrarily far through in
     * one tick.
     * @param mode  Requested blend-mode phase for this tick; kInactive falls back to
     *   kRampDown here (see above).
     * @param commanded_positions  Per-joint targets to slew the published positions toward.
     * @param dt_s  Elapsed time for this tick; see above for the handling of non-positive or
     *   pathologically large values.
     * @return The resulting blend weight.
     */
    double step(
        BlendMode mode, const std::array<double, kNumArmJoints>& commanded_positions,
        double dt_s) noexcept;

    /**
     * @brief Current blend weight in [0, 1]; set by seedFromMeasured(), advanced by step().
     * @return The current blend weight.
     */
    double weight() const noexcept { return weight_; }
    /**
     * @brief Currently published per-joint targets, last set by seedFromMeasured() or step().
     * @return The currently published per-joint targets.
     */
    const std::array<double, kNumArmJoints>& publishedPositions() const noexcept
    {
        return published_positions_;
    }

private:
    double rampDurationFor(BlendMode mode) const noexcept;

    RampConfig                        config_;
    double                            weight_{ 0.0 };
    std::array<double, kNumArmJoints> published_positions_{};
};

/**
 * @brief Validates a joint->motor_index map.
 *
 * Every entry must be unique and fall inside the arm range [15, 28] from
 * Unitree's G1Arm7JointIndex (legs 0-11, waist 12-14, arms 15-28 -- see
 * g1_description/config/arm_sdk_params.yaml).
 * @param motor_index  Joint-index-to-motor-index map to validate.
 * @return Empty string if valid, else a human-readable reason for on_init to log.
 */
std::string validateMotorIndexMap(const std::array<int, kNumArmJoints>& motor_index);

/**
 * @brief Resolves the mode write() should actually drive this tick.
 *
 * Takes the mode currently requested (via the shared lifecycle atomic) and
 * whether LowState is stale. Staleness can only escalate a requested
 * kActive to kEmergencyRampDown; a request that's already ramping down is
 * left alone (already shutting down at least as fast). Pure and
 * deterministic, so calling it repeatedly with the same stale reading never
 * re-escalates or oscillates -- staleness trips the escalation exactly once
 * per activation.
 * @param requested_mode  Mode currently requested via the shared lifecycle atomic.
 * @param lowstate_stale  Whether the most recent LowState reading is stale.
 * @return The mode write() should actually drive this tick.
 */
BlendMode resolveEffectiveMode(BlendMode requested_mode, bool lowstate_stale) noexcept;

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__ARM_RAMP_ENGINE_HPP_
