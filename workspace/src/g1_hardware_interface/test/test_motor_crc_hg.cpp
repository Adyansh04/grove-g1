/**
 * @file test_motor_crc_hg.cpp
 * @brief Coverage tests for the LowCmd checksum the firmware validates.
 *
 * The bit loop is vendored and unmodified, so these do not re-derive it. What they pin down is
 * the part that can silently rot: which bytes the CRC actually covers, and whether the raw
 * mirror struct's padding is deterministic.
 */
#include <gmock/gmock.h>

#include <cstdint>
#include <functional>

#include "g1_hardware_interface/motor_crc_hg.hpp"
#include "unitree_hg/msg/low_cmd.hpp"

namespace g1_hardware_interface::vendored
{
namespace
{

/// A non-trivial baseline: an all-zero message makes too many mistakes look identical.
unitree_hg::msg::LowCmd baseline()
{
    unitree_hg::msg::LowCmd cmd{};
    cmd.mode_pr      = 0;
    cmd.mode_machine = 5;
    for (std::size_t slot = 0; slot < cmd.motor_cmd.size(); ++slot)
    {
        auto& motor = cmd.motor_cmd[slot];
        motor.mode  = 1;
        motor.q     = 0.01F * static_cast<float>(slot);
        motor.dq    = 0.0F;
        motor.tau   = 0.0F;
        motor.kp    = 40.0F;
        motor.kd    = 1.0F;
    }
    return cmd;
}

std::uint32_t crcOf(unitree_hg::msg::LowCmd cmd)
{
    computeLowCmdCrc(cmd);
    return cmd.crc;
}

/// Asserts a field is inside the checksummed range: perturb it, the CRC must move.
void expectCovered(const char* what, const std::function<void(unitree_hg::msg::LowCmd&)>& perturb)
{
    unitree_hg::msg::LowCmd perturbed = baseline();
    perturb(perturbed);
    EXPECT_NE(crcOf(perturbed), crcOf(baseline())) << what << " is outside the checksummed range";
}

TEST(MotorCrcHg, EveryLowCmdHeaderFieldIsCovered)
{
    expectCovered("mode_pr", [](auto& cmd) { cmd.mode_pr = 1; });
    expectCovered("mode_machine", [](auto& cmd) { cmd.mode_machine = 4; });
    expectCovered("reserve", [](auto& cmd) { cmd.reserve[0] = 0xABCDEF01; });
}

TEST(MotorCrcHg, EveryMotorCmdFieldIsCovered)
{
    expectCovered("motor mode", [](auto& cmd) { cmd.motor_cmd[0].mode = 0; });
    expectCovered("motor q", [](auto& cmd) { cmd.motor_cmd[0].q = 1.25F; });
    expectCovered("motor dq", [](auto& cmd) { cmd.motor_cmd[0].dq = 1.25F; });
    expectCovered("motor tau", [](auto& cmd) { cmd.motor_cmd[0].tau = 1.25F; });
    expectCovered("motor kp", [](auto& cmd) { cmd.motor_cmd[0].kp = 55.0F; });
    expectCovered("motor kd", [](auto& cmd) { cmd.motor_cmd[0].kd = 5.0F; });
    expectCovered("motor reserve", [](auto& cmd) { cmd.motor_cmd[0].reserve = 7; });
}

TEST(MotorCrcHg, TheLastMotorSlotIsCovered)
{
    // The covered length is derived from sizeof, so an off-by-one would drop the tail of the
    // array. Slot 29 is the arm_sdk weight and 34 is the end of the message.
    expectCovered("motor_cmd[29]", [](auto& cmd) { cmd.motor_cmd[29].q = 1.0F; });
    expectCovered("motor_cmd[34]", [](auto& cmd) { cmd.motor_cmd[34].q = 1.0F; });
}

TEST(MotorCrcHg, TheChecksumFieldItselfIsExcluded)
{
    unitree_hg::msg::LowCmd stale = baseline();
    stale.crc                     = 0xDEADBEEF;
    EXPECT_EQ(crcOf(stale), crcOf(baseline()));
}

TEST(MotorCrcHg, IdenticalContentChecksumsIdenticallyAcrossMessages)
{
    // The CRC runs over a raw mirror struct including its alignment padding, so a message built
    // field by field must agree with one built any other way. Divergence here means uninitialised
    // padding is reaching the wire.
    unitree_hg::msg::LowCmd built_in_reverse{};
    built_in_reverse.mode_machine = 5;
    built_in_reverse.mode_pr      = 0;
    for (std::size_t i = built_in_reverse.motor_cmd.size(); i > 0; --i)
    {
        auto& motor = built_in_reverse.motor_cmd[i - 1];
        motor.kd    = 1.0F;
        motor.kp    = 40.0F;
        motor.q     = 0.01F * static_cast<float>(i - 1);
        motor.mode  = 1;
    }

    EXPECT_EQ(crcOf(built_in_reverse), crcOf(baseline()));
}

TEST(MotorCrcHg, ChecksumIsWrittenBackOntoTheMessage)
{
    unitree_hg::msg::LowCmd cmd = baseline();
    cmd.crc                     = 0;
    computeLowCmdCrc(cmd);
    EXPECT_NE(cmd.crc, 0U);
}

}  // namespace
}  // namespace g1_hardware_interface::vendored
