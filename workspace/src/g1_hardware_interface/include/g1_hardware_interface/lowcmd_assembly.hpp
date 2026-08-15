#ifndef G1_HARDWARE_INTERFACE__LOWCMD_ASSEMBLY_HPP_
#define G1_HARDWARE_INTERFACE__LOWCMD_ASSEMBLY_HPP_

/**
 * @file lowcmd_assembly.hpp
 * @brief Per-motor LowCmd packing for the full-body rt/lowcmd path, kept free of rclcpp so the
 *        mode table can be asserted without a live hardware component.
 */

#include <cstddef>
#include <cstdint>

#include "unitree_hg/msg/low_cmd.hpp"

namespace g1_hardware_interface
{

/// The body motors rt/lowcmd owns: legs 0-11, waist 12-14, arms 15-28.
inline constexpr std::size_t kNumBodyMotors = 29;

/**
 * @brief What a joint's claimed interfaces mean for the firmware's control law,
 * `tau = tau_ff + kp * (q - q_meas) + kd * (dq - dq_meas)`.
 */
enum class JointControlMode : std::uint8_t
{
    /// Unclaimed: motor disabled, every field zero. Also where a finished release ramp lands.
    kDisabled,
    /// Position tracked at the joint's configured fallback gains, no feedforward torque.
    kPositionOnly,
    /// Torque with damping: kp is forced to zero so the position term cannot fight the effort.
    kEffort,
    /// The policy mode -- the controller owns q, dq, tau, kp and kd on every tick.
    kImpedance,
};

/// Which command interfaces a controller currently holds on one joint.
struct InterfaceClaims
{
    bool position = false;
    bool velocity = false;
    bool effort   = false;
    /// kp and kd together; either one alone does not define an impedance.
    bool impedance = false;
};

/**
 * @brief Maps claimed interfaces onto the mode write() acts on.
 *
 * Impedance wins because it is the only claim carrying its own gains. A velocity-only claim
 * resolves to kDisabled: with kp and kd both zero the firmware law has no term left to act on,
 * so there is no such mode on this hardware.
 */
[[nodiscard]] JointControlMode resolveJointMode(const InterfaceClaims& claims) noexcept;

/// One joint's commanded values, as written by whichever controller holds its interfaces.
struct JointCommand
{
    double position = 0.0;
    double velocity = 0.0;
    double effort   = 0.0;
    double kp       = 0.0;
    double kd       = 0.0;
};

/// Gains used in kPositionOnly, where the controller supplies no kp/kd of its own.
struct PositionOnlyGains
{
    double kp = 0.0;
    double kd = 0.0;
};

/**
 * @brief Fills one motor_cmd slot for `mode`.
 *
 * @param measured_position Read only in kEffort, where q must sit on the measurement so the
 *                          position term contributes nothing while tau does the work.
 */
void fillMotorCmd(
    unitree_hg::msg::MotorCmd& motor, JointControlMode mode, const JointCommand& command,
    const PositionOnlyGains& fallback, double measured_position);

/**
 * @brief Fills one motor_cmd slot for the release ramp: hold position at fading stiffness with
 *        fixed damping, so authority hands back as a controlled sag rather than a drop.
 *
 * @param hold_position   Where the joint was when the release began, not the live measurement --
 *                        tracking the fall would drive the joint down with it.
 * @param kp_at_release   The joint's stiffness on the last commanded tick.
 * @param stiffness_scale 1.0 at the start of the ramp, 0.0 at its end.
 * @param release_kd      Damping held flat across the whole ramp, so it survives kp reaching zero.
 */
void fillReleaseCmd(
    unitree_hg::msg::MotorCmd& motor, double hold_position, double kp_at_release,
    double stiffness_scale, double release_kd);

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__LOWCMD_ASSEMBLY_HPP_
