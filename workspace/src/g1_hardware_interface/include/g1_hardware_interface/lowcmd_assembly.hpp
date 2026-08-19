#ifndef G1_HARDWARE_INTERFACE__LOWCMD_ASSEMBLY_HPP_
#define G1_HARDWARE_INTERFACE__LOWCMD_ASSEMBLY_HPP_

/**
 * @file lowcmd_assembly.hpp
 * @brief Per-motor LowCmd packing for rt/lowcmd, on unitree_sdk2's own DDS structs. Split out
 *        from the component so the mode table and the checksum are assertable without one.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <unitree/idl/hg/LowCmd_.hpp>

namespace g1_hardware_interface
{

/// The body motors rt/lowcmd owns: legs 0-11, waist 12-14, arms 15-28.
inline constexpr std::size_t kNumBodyMotors = 29;

/**
 * @brief Per-joint branch of the firmware law `tau = tau_ff + kp*(q - q_meas) + kd*(dq - dq_meas)`.
 *
 * Mirrors the upstream motor-command fill, so controllers written against it behave identically
 * here.
 */
enum class JointControlMode : std::uint8_t
{
    /// Unclaimed: motor off. Holding is a controller's job, see G1FreezeController.
    kDisabled,
    kPositionOnly,
    /// kp forced to 0 so the position term cannot fight the commanded torque.
    kEffort,
    /// The policy mode: controller owns q, dq, tau, kp and kd every tick.
    kImpedance,
};

/**
 * @brief Which command interfaces a controller currently holds on one joint.
 */
struct InterfaceClaims
{
    bool position = false;
    bool velocity = false;
    bool effort   = false;
    /// kp and kd together; either alone does not define an impedance.
    bool impedance = false;
};

/**
 * @brief Maps claimed command interfaces onto the mode write() acts on.
 *
 * Impedance outranks the rest, being the only claim carrying its own gains. Velocity alone
 * resolves to kDisabled: with kp and kd zero the firmware law has no term left to act on.
 *
 * @param claims Interfaces a controller currently holds on one joint.
 * @return The branch fillMotorCmd should take for that joint.
 */
[[nodiscard]] JointControlMode resolveJointMode(const InterfaceClaims& claims) noexcept;

/**
 * @brief One joint's commanded values, from whichever controller holds its interfaces.
 */
struct JointCommand
{
    double position = 0.0;
    double velocity = 0.0;
    double effort   = 0.0;
    double kp       = 0.0;
    double kd       = 0.0;
};

/**
 * @brief Gains used in kPositionOnly, where the controller supplies none of its own.
 */
struct PositionOnlyGains
{
    double kp = 0.0;
    double kd = 0.0;
};

/**
 * @brief Fills one motor_cmd slot for `mode`.
 *
 * @param motor             Slot filled in place.
 * @param mode              Which branch of the firmware law to command.
 * @param command           Values written by the claiming controller.
 * @param fallback          Gains applied in kPositionOnly.
 * @param measured_position Read only in kEffort, where q sits on the measurement so the position
 *                          term contributes nothing.
 */
void fillMotorCmd(
    unitree_hg::msg::dds_::MotorCmd_& motor, JointControlMode mode, const JointCommand& command,
    const PositionOnlyGains& fallback, double measured_position);

/**
 * @brief Fills one motor_cmd slot for the release ramp: fading stiffness, fixed damping.
 *
 * @param motor           Slot filled in place.
 * @param hold_position   Where the joint was when the release began; tracking the live
 *                        measurement would drive the joint down with the fall.
 * @param kp_at_release   The joint's stiffness on its last commanded tick.
 * @param stiffness_scale 1.0 at the start of the ramp, 0.0 at its end.
 * @param release_kd      Damping held flat across the ramp, so it survives kp reaching zero.
 */
void fillReleaseCmd(
    unitree_hg::msg::dds_::MotorCmd_& motor, double hold_position, double kp_at_release,
    double stiffness_scale, double release_kd);

/**
 * @brief Checksums `cmd` in place over every byte but its own crc field, which the firmware
 *        requires on every frame.
 *
 * @param cmd LowCmd whose crc field is overwritten.
 * @note bit_cast rather than a uint32_t* cast, which GCC 13 optimises fields out of at -O2.
 */
void computeLowCmdCrc(unitree_hg::msg::dds_::LowCmd_& cmd);

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__LOWCMD_ASSEMBLY_HPP_
