/**
 * @file arm_ramp_engine.cpp
 * @brief Arm blend-weight/position ramping, plus motor-index validation and mode-safety helpers.
 */

#include "g1_hardware_interface/arm_ramp_engine.hpp"

#include <algorithm>
#include <set>

namespace g1_hardware_interface
{

namespace
{
/**
 * @brief Inclusive motor-index bounds for the G1 arm joints.
 *
 * G1Arm7JointIndex (unitree_ros2's example/src/include/g1/g1.hpp): legs
 * occupy 0-11, waist 12-14, arms 15-28.
 */
constexpr int kMinArmMotorIndex = 15;
constexpr int kMaxArmMotorIndex = 28;
}  // namespace

ArmRampEngine::ArmRampEngine(const RampConfig& config)
  : config_(config)
{}

void ArmRampEngine::seedFromMeasured(const std::array<double, kNumArmJoints>& measured_positions)
{
    published_positions_ = measured_positions;
    weight_              = 0.0;
}

double ArmRampEngine::rampDurationFor(BlendMode mode) const
{
    /*
     * Listed exhaustively (no default) so a future BlendMode addition trips
     * -Wswitch instead of silently falling through.
     */
    switch (mode)
    {
        case BlendMode::kActive:
            return config_.blend_ramp_up_s;
        case BlendMode::kEmergencyRampDown:
            return config_.emergency_ramp_down_s;
        case BlendMode::kRampDown:
            return config_.blend_ramp_down_s;
        case BlendMode::kInactive:
            return config_.blend_ramp_down_s;
    }
    return config_.blend_ramp_down_s;  // unreachable; keeps -Wreturn-type quiet
}

double ArmRampEngine::step(
    BlendMode mode, const std::array<double, kNumArmJoints>& commanded_positions, double dt_s)
{
    if (dt_s <= 0.0)
    {
        /*
         * Hold rather than integrate: a zero/negative tick has nothing
         * meaningful to advance, and letting it through would either snap
         * weight_ straight to target_weight (the `else` branch below) or,
         * for a negative dt_s, hand clamp() an inverted [-max_step,
         * max_step] range, which is UB.
         */
        return weight_;
    }
    /*
     * Guards a pathologically large dt_s (e.g. a stalled control loop)
     * proportionally to whatever cadence the caller configured, rather than
     * a fixed wall-clock constant. nominal_period_s <= 0.0 means "not
     * configured" (e.g. a RampConfig built without it) -- skip the clamp
     * rather than collapsing every dt_s to zero.
     */
    if (config_.nominal_period_s > 0.0)
    {
        dt_s = std::min(dt_s, kMaxDtNominalPeriodMultiple * config_.nominal_period_s);
    }

    /*
     * Target is purely a function of the mode passed in *this* tick, so a
     * ramp-down requested mid-ramp-up just changes direction from wherever
     * the weight currently sits -- monotonic by construction, no special
     * case needed for "deactivate while ramping up".
     */
    const double target_weight   = (mode == BlendMode::kActive) ? 1.0 : 0.0;
    const double ramp_duration_s = rampDurationFor(mode);
    if (ramp_duration_s > 0.0)
    {
        const double max_step = dt_s / ramp_duration_s;
        weight_ += std::clamp(target_weight - weight_, -max_step, max_step);
    }
    else
    {
        /*
         * Degenerate config: on_init already rejects a non-positive ramp
         * duration, so this is a defensive fallback, not the normal path.
         */
        weight_ = target_weight;
    }
    weight_ = std::clamp(weight_, 0.0, 1.0);

    const double max_delta = config_.max_joint_velocity_rad_s * dt_s;
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        const double diff = commanded_positions[i] - published_positions_[i];
        published_positions_[i] += std::clamp(diff, -max_delta, max_delta);
    }

    return weight_;
}

std::string validateMotorIndexMap(const std::array<int, kNumArmJoints>& motor_index)
{
    std::set<int> seen;
    for (const int index : motor_index)
    {
        if (index < kMinArmMotorIndex || index > kMaxArmMotorIndex)
        {
            return "motor_index " + std::to_string(index) + " is outside the arm range [" +
                   std::to_string(kMinArmMotorIndex) + ", " + std::to_string(kMaxArmMotorIndex) +
                   "]";
        }
        if (!seen.insert(index).second)
        {
            return "duplicate motor_index " + std::to_string(index) + " across arm joints";
        }
    }
    return "";
}

BlendMode resolveEffectiveMode(BlendMode requested_mode, bool lowstate_stale) noexcept
{
    if (requested_mode == BlendMode::kActive && lowstate_stale)
    {
        return BlendMode::kEmergencyRampDown;
    }
    return requested_mode;
}

}  // namespace g1_hardware_interface
