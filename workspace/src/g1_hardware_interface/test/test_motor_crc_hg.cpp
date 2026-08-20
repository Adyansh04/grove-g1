/**
 * @file test_motor_crc_hg.cpp
 * @brief Which bytes the LowCmd checksum covers, and whether the mirror struct's padding is
 *        deterministic. The bit loop itself is vendored, so it is not re-derived here.
 */
#include <gmock/gmock.h>

#include <cstdint>
#include <functional>

#include "g1_hardware_interface/lowcmd_assembly.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"

namespace g1_hardware_interface
{
namespace
{

using LowCmd = unitree_hg::msg::dds_::LowCmd_;

/// Non-trivial on purpose: an all-zero message makes too many mistakes look identical.
LowCmd baseline()
{
    LowCmd cmd{};
    cmd.mode_pr()      = 0;
    cmd.mode_machine() = 5;
    for (std::size_t slot = 0; slot < cmd.motor_cmd().size(); ++slot)
    {
        auto& motor  = cmd.motor_cmd()[slot];
        motor.mode() = 1;
        motor.q()    = 0.01F * static_cast<float>(slot);
        motor.dq()   = 0.0F;
        motor.tau()  = 0.0F;
        motor.kp()   = 40.0F;
        motor.kd()   = 1.0F;
    }
    return cmd;
}

std::uint32_t crcOf(LowCmd cmd)
{
    computeLowCmdCrc(cmd);
    return cmd.crc();
}

/// Asserts a field is inside the checksummed range: perturb it, the CRC must move.
void expectCovered(const char* what, const std::function<void(LowCmd&)>& perturb)
{
    LowCmd perturbed = baseline();
    perturb(perturbed);
    EXPECT_NE(crcOf(perturbed), crcOf(baseline())) << what << " is outside the checksummed range";
}

TEST(MotorCrcHg, EveryLowCmdHeaderFieldIsCovered)
{
    // mode_pr and mode_machine are the two the strict-aliasing bug silently dropped, and
    // mode_machine carries 5 on this robot, so every frame would have been rejected.
    expectCovered("mode_pr", [](auto& cmd) { cmd.mode_pr() = 1; });
    expectCovered("mode_machine", [](auto& cmd) { cmd.mode_machine() = 4; });
    expectCovered("reserve", [](auto& cmd) { cmd.reserve()[0] = 0xABCDEF01; });
}

TEST(MotorCrcHg, EveryMotorCmdFieldIsCovered)
{
    expectCovered("motor mode", [](auto& cmd) { cmd.motor_cmd()[0].mode() = 0; });
    expectCovered("motor q", [](auto& cmd) { cmd.motor_cmd()[0].q() = 1.25F; });
    expectCovered("motor dq", [](auto& cmd) { cmd.motor_cmd()[0].dq() = 1.25F; });
    expectCovered("motor tau", [](auto& cmd) { cmd.motor_cmd()[0].tau() = 1.25F; });
    expectCovered("motor kp", [](auto& cmd) { cmd.motor_cmd()[0].kp() = 55.0F; });
    expectCovered("motor kd", [](auto& cmd) { cmd.motor_cmd()[0].kd() = 5.0F; });
    expectCovered("motor reserve", [](auto& cmd) { cmd.motor_cmd()[0].reserve() = 7; });
}

TEST(MotorCrcHg, TheCoveredRangeReachesTheWordBeforeTheChecksum)
{
    // The covered length comes from sizeof, so an off-by-one drops the tail. reserve[3] is the
    // last word before crc, and it is the only perturbation that catches a length one word short.
    expectCovered("motor_cmd[29]", [](auto& cmd) { cmd.motor_cmd()[29].q() = 1.0F; });
    expectCovered("motor_cmd[34]", [](auto& cmd) { cmd.motor_cmd()[34].q() = 1.0F; });
    expectCovered("reserve[3]", [](auto& cmd) { cmd.reserve()[3] = 0xABCDEF01; });
}

TEST(MotorCrcHg, AStaleChecksumCannotLeakIntoTheSum)
{
    // Pins the zeroing at the top of computeLowCmdCrc, not the covered length: with crc zeroed
    // first, a leftover value cannot reach the sum whatever the length is.
    LowCmd stale = baseline();
    stale.crc()  = 0xDEADBEEF;
    EXPECT_EQ(crcOf(stale), crcOf(baseline()));
}

TEST(MotorCrcHg, IdenticalContentChecksumsIdenticallyAcrossMessages)
{
    // The CRC covers the struct's alignment padding too, so divergence here means
    // uninitialised padding is reaching the wire.
    LowCmd built_in_reverse{};
    built_in_reverse.mode_machine() = 5;
    built_in_reverse.mode_pr()      = 0;
    for (std::size_t i = built_in_reverse.motor_cmd().size(); i > 0; --i)
    {
        auto& motor  = built_in_reverse.motor_cmd()[i - 1];
        motor.kd()   = 1.0F;
        motor.kp()   = 40.0F;
        motor.q()    = 0.01F * static_cast<float>(i - 1);
        motor.mode() = 1;
    }

    EXPECT_EQ(crcOf(built_in_reverse), crcOf(baseline()));
}

TEST(MotorCrcHg, ChecksumIsWrittenBackOntoTheMessage)
{
    LowCmd cmd = baseline();
    cmd.crc()  = 0;
    computeLowCmdCrc(cmd);
    EXPECT_NE(cmd.crc(), 0U);
}

}  // namespace
}  // namespace g1_hardware_interface
