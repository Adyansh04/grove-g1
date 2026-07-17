#ifndef G1_HARDWARE_INTERFACE__ARM_RAMP_ENGINE_HPP_
#define G1_HARDWARE_INTERFACE__ARM_RAMP_ENGINE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace g1_hardware_interface
{

// The G1ArmSdkSystem exports exactly the 14 arm joints (7 per arm: shoulder
// pitch/roll/yaw, elbow, wrist roll/pitch/yaw); legs/waist/hands stay with
// the onboard controller.
inline constexpr std::size_t kNumArmJoints = 14;

// Phases of the single writer-authority state machine. kInactive means "not
// commanding" (write() self-gates on it); the other three all imply active
// publication with the blend weight moving toward a mode-specific target (1
// for kActive, 0 otherwise) at a mode-specific rate.
enum class BlendMode : std::uint8_t
{
    kInactive,
    kActive,
    kRampDown,
    kEmergencyRampDown,
};

struct RampConfig
{
    double blend_ramp_up_s{ 0.0 };
    double blend_ramp_down_s{ 0.0 };
    double emergency_ramp_down_s{ 0.0 };
    double max_joint_velocity_rad_s{ 0.0 };
};

// Pure, ROS-free ramp/slew logic for the arm_sdk blend weight and per-joint
// command targets -- the safety-critical unit-testable surface. Not
// thread-safe by design: exactly one thread drives it at a time (see
// G1ArmSdkSystem's writer-token contract for how that's enforced across the
// RT and lifecycle threads).
class ArmRampEngine
{
public:
    explicit ArmRampEngine(const RampConfig& config);

    // Hold-in-place seed: on_activate calls this with the just-measured joint
    // positions so the first published target matches reality exactly (no
    // snap), and zeroes the blend weight so the onboard controller isn't
    // handed anything until the ramp-up has actually started.
    void seedFromMeasured(const std::array<double, kNumArmJoints>& measured_positions);

    // One control-loop tick. Advances the blend weight toward the target
    // implied by `mode` (1 for kActive, 0 for kRampDown/kEmergencyRampDown, at
    // blend_ramp_up_s/blend_ramp_down_s/emergency_ramp_down_s respectively)
    // and slews every joint's published target toward `commanded_positions`
    // by at most max_joint_velocity_rad_s * dt_s, starting from wherever the
    // target currently is -- never a jump. `mode` is never kInactive in
    // practice (callers self-gate before reaching this call); it's treated
    // the same as kRampDown here as a defensive fallback rather than left
    // unhandled. Returns the resulting weight.
    double
    step(BlendMode mode, const std::array<double, kNumArmJoints>& commanded_positions, double dt_s);

    double                                   weight() const noexcept { return weight_; }
    const std::array<double, kNumArmJoints>& publishedPositions() const noexcept
    {
        return published_positions_;
    }

private:
    double rampDurationFor(BlendMode mode) const;

    RampConfig                        config_;
    double                            weight_{ 0.0 };
    std::array<double, kNumArmJoints> published_positions_{};
};

// Validates a joint->motor_index map: every entry must be unique and fall
// inside the arm range [15, 28] from Unitree's G1Arm7JointIndex (legs 0-11,
// waist 12-14, arms 15-28 -- see g1_description/config/arm_sdk_params.yaml).
// Returns an empty string if valid, else a human-readable reason for on_init
// to log.
std::string validateMotorIndexMap(const std::array<int, kNumArmJoints>& motor_index);

// Resolves the mode write() should actually drive this tick, given the mode
// currently requested (via the shared lifecycle atomic) and whether
// LowState is stale. Staleness can only escalate a requested kActive to
// kEmergencyRampDown; a request that's already ramping down is left alone
// (already shutting down at least as fast). Pure and deterministic, so
// calling it repeatedly with the same stale reading never re-escalates or
// oscillates -- staleness trips the escalation exactly once per activation.
BlendMode resolveEffectiveMode(BlendMode requested_mode, bool lowstate_stale) noexcept;

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__ARM_RAMP_ENGINE_HPP_
