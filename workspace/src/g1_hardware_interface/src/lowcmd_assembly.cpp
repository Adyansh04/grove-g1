/**
 * @file lowcmd_assembly.cpp
 * @brief Mode resolution and per-motor LowCmd packing for the rt/lowcmd path.
 */

#include "g1_hardware_interface/lowcmd_assembly.hpp"

namespace g1_hardware_interface
{

namespace
{
/// MotorCmd.mode: 1 enables the motor, 0 leaves it unpowered. Unitree's hg low-level examples.
constexpr std::uint8_t kMotorEnabled  = 1;
constexpr std::uint8_t kMotorDisabled = 0;
}  // namespace

JointControlMode resolveJointMode(const InterfaceClaims& claims) noexcept
{
    if (claims.impedance)
    {
        return JointControlMode::kImpedance;
    }
    if (claims.effort)
    {
        return JointControlMode::kEffort;
    }
    if (claims.position)
    {
        return JointControlMode::kPositionOnly;
    }
    return JointControlMode::kDisabled;
}

void fillMotorCmd(
    unitree_hg::msg::MotorCmd& motor, JointControlMode mode, const JointCommand& command,
    const PositionOnlyGains& fallback, double measured_position)
{
    switch (mode)
    {
        case JointControlMode::kImpedance:
            motor.mode = kMotorEnabled;
            motor.q    = static_cast<float>(command.position);
            motor.dq   = static_cast<float>(command.velocity);
            motor.tau  = static_cast<float>(command.effort);
            motor.kp   = static_cast<float>(command.kp);
            motor.kd   = static_cast<float>(command.kd);
            break;

        case JointControlMode::kEffort:
            motor.mode = kMotorEnabled;
            motor.q    = static_cast<float>(measured_position);
            motor.dq   = static_cast<float>(command.velocity);
            motor.tau  = static_cast<float>(command.effort);
            motor.kp   = 0.0F;
            motor.kd   = static_cast<float>(command.kd);
            break;

        case JointControlMode::kPositionOnly:
            motor.mode = kMotorEnabled;
            motor.q    = static_cast<float>(command.position);
            motor.dq   = 0.0F;
            motor.tau  = 0.0F;
            motor.kp   = static_cast<float>(fallback.kp);
            motor.kd   = static_cast<float>(fallback.kd);
            break;

        case JointControlMode::kDisabled:
            motor.mode = kMotorDisabled;
            motor.q    = 0.0F;
            motor.dq   = 0.0F;
            motor.tau  = 0.0F;
            motor.kp   = 0.0F;
            motor.kd   = 0.0F;
            break;
    }
}

void fillReleaseCmd(
    unitree_hg::msg::MotorCmd& motor, double hold_position, double kp_at_release,
    double stiffness_scale, double release_kd)
{
    motor.mode = kMotorEnabled;
    motor.q    = static_cast<float>(hold_position);
    motor.dq   = 0.0F;
    motor.tau  = 0.0F;
    motor.kp   = static_cast<float>(kp_at_release * stiffness_scale);
    motor.kd   = static_cast<float>(release_kd);
}

}  // namespace g1_hardware_interface
