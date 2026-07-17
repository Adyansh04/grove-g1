#include "g1_hardware_interface/arm_ramp_engine.hpp"

#include <algorithm>
#include <set>

namespace g1_hardware_interface
{

namespace
{
// G1Arm7JointIndex (unitree_ros2's example/src/include/g1/g1.hpp): legs
// occupy 0-11, waist 12-14, arms 15-28.
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
    // Listed exhaustively (no default) so a future BlendMode addition trips
    // -Wswitch instead of silently falling through.
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
    // Target is purely a function of the mode passed in *this* tick, so a
    // ramp-down requested mid-ramp-up just changes direction from wherever
    // the weight currently sits -- monotonic by construction, no special
    // case needed for "deactivate while ramping up".
    const double target_weight   = (mode == BlendMode::kActive) ? 1.0 : 0.0;
    const double ramp_duration_s = rampDurationFor(mode);
    if (ramp_duration_s > 0.0 && dt_s > 0.0)
    {
        const double max_step = dt_s / ramp_duration_s;
        weight_ += std::clamp(target_weight - weight_, -max_step, max_step);
    }
    else
    {
        // Degenerate config (dt <= 0, e.g. the very first tick) or a
        // ramp_duration_s of 0 -- on_init already rejects non-positive ramp
        // params, so this is a defensive fallback, not the normal path.
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
