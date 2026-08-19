/**
 * @file lowcmd_assembly.cpp
 * @brief Mode resolution, per-motor LowCmd packing and the checksum for the rt/lowcmd path.
 */

#include "g1_hardware_interface/lowcmd_assembly.hpp"

#include <bit>
#include <cmath>
#include <type_traits>

#include "g1_hardware_interface/motor_crc_hg.hpp"

namespace g1_hardware_interface
{

namespace
{
/// MotorCmd.mode: 1 enables the motor, 0 leaves it unpowered. Unitree's hg low-level examples.
constexpr std::uint8_t kMotorEnabled  = 1;
constexpr std::uint8_t kMotorDisabled = 0;

using LowCmd = unitree_hg::msg::dds_::LowCmd_;

static_assert(
    sizeof(LowCmd) == 1004 && sizeof(LowCmd) % sizeof(std::uint32_t) == 0,
    "LowCmd_ must keep the wire layout the firmware checksums, in whole 32-bit words");
static_assert(
    std::is_trivially_copyable_v<LowCmd>, "bit_cast to a word array needs trivial copyability");
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
    unitree_hg::msg::dds_::MotorCmd_& motor, JointControlMode mode, const JointCommand& command,
    const PositionOnlyGains& fallback, double measured_position)
{
    // The only funnel between a controller and rt/lowcmd: every controller, both arm states, the
    // freeze paths and the release ramp come through here. A non-finite value from any of them
    // would otherwise reach the motors as a float cast that is undefined for anything outside
    // float range, so the joint goes unpowered instead: garbage on the wire is worse than a
    // joint that stops being driven, and the caller finds out from the joint not moving.
    if (!std::isfinite(command.position) || !std::isfinite(command.velocity) ||
        !std::isfinite(command.effort) || !std::isfinite(command.kp) ||
        !std::isfinite(command.kd) || !std::isfinite(measured_position))
    {
        mode = JointControlMode::kDisabled;
    }

    switch (mode)
    {
        case JointControlMode::kImpedance:
            motor.mode() = kMotorEnabled;
            motor.q()    = static_cast<float>(command.position);
            motor.dq()   = static_cast<float>(command.velocity);
            motor.tau()  = static_cast<float>(command.effort);
            motor.kp()   = static_cast<float>(command.kp);
            motor.kd()   = static_cast<float>(command.kd);
            break;

        case JointControlMode::kEffort:
            motor.mode() = kMotorEnabled;
            motor.q()    = static_cast<float>(measured_position);
            motor.dq()   = static_cast<float>(command.velocity);
            motor.tau()  = static_cast<float>(command.effort);
            motor.kp()   = 0.0F;
            motor.kd()   = static_cast<float>(command.kd);
            break;

        case JointControlMode::kPositionOnly:
            motor.mode() = kMotorEnabled;
            motor.q()    = static_cast<float>(command.position);
            motor.dq()   = 0.0F;
            motor.tau()  = 0.0F;
            motor.kp()   = static_cast<float>(fallback.kp);
            motor.kd()   = static_cast<float>(fallback.kd);
            break;

        case JointControlMode::kDisabled:
            motor.mode() = kMotorDisabled;
            motor.q()    = 0.0F;
            motor.dq()   = 0.0F;
            motor.tau()  = 0.0F;
            motor.kp()   = 0.0F;
            motor.kd()   = 0.0F;
            break;
    }
}

void fillReleaseCmd(
    unitree_hg::msg::dds_::MotorCmd_& motor, double hold_position, double kp_at_release,
    double stiffness_scale, double release_kd)
{
    motor.mode() = kMotorEnabled;
    motor.q()    = static_cast<float>(hold_position);
    motor.dq()   = 0.0F;
    motor.tau()  = 0.0F;
    motor.kp()   = static_cast<float>(kp_at_release * stiffness_scale);
    motor.kd()   = static_cast<float>(release_kd);
}

void computeLowCmdCrc(unitree_hg::msg::dds_::LowCmd_& cmd)
{
    cmd.crc()        = 0;
    const auto words = std::bit_cast<std::array<std::uint32_t, sizeof(LowCmd) / 4>>(cmd);
    cmd.crc() = vendored::crc32Core(words.data(), static_cast<std::uint32_t>(words.size()) - 1);
}

}  // namespace g1_hardware_interface
