/**
 * @file test_assemble_low_cmd.cpp
 * @brief Unit tests for assembleLowCmd() -- arm and waist slot writes, the weight slot, and the
 *        slots that must stay untouched.
 */
#include <gmock/gmock.h>

#include <array>

#include "g1_hardware_interface/g1_arm_sdk_system.hpp"
#include "unitree_hg/msg/low_cmd.hpp"

namespace g1_hardware_interface
{
namespace
{

std::array<double, kNumWaistJoints> waistHold() { return { 0.05, -0.02, 0.11 }; }

std::array<int, kNumArmJoints> realMotorIndexMap()
{
    std::array<int, kNumArmJoints> indices{};
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        indices[i] = 15 + static_cast<int>(i);
    }
    return indices;
}

TEST(AssembleLowCmd, NonArmNonWeightSlotsStayZeroed)
{
    unitree_hg::msg::LowCmd           cmd{};
    const auto                        motor_index = realMotorIndexMap();
    std::array<double, kNumArmJoints> position{};
    std::array<double, kNumArmJoints> kp{};
    std::array<double, kNumArmJoints> kd{};
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        position[i] = 0.1 * static_cast<double>(i);
        kp[i]       = 40.0;
        kd[i]       = 1.0;
    }

    assembleLowCmd(cmd, motor_index, position, kp, kd, 0.7F, waistHold(), 160.0, 4.0);

    for (std::size_t slot = 0; slot < cmd.motor_cmd.size(); ++slot)
    {
        const bool is_arm_slot    = slot >= 15 && slot <= 28;
        const bool is_waist_slot  = slot >= 12 && slot <= 14;
        const bool is_weight_slot = slot == kWeightMotorIndex;
        if (is_arm_slot || is_waist_slot || is_weight_slot)
        {
            continue;
        }
        const auto& motor = cmd.motor_cmd[slot];
        EXPECT_EQ(motor.mode, 0U) << "slot " << slot;
        EXPECT_FLOAT_EQ(motor.q, 0.0F) << "slot " << slot;
        EXPECT_FLOAT_EQ(motor.dq, 0.0F) << "slot " << slot;
        EXPECT_FLOAT_EQ(motor.tau, 0.0F) << "slot " << slot;
        EXPECT_FLOAT_EQ(motor.kp, 0.0F) << "slot " << slot;
        EXPECT_FLOAT_EQ(motor.kd, 0.0F) << "slot " << slot;
    }
}

TEST(AssembleLowCmd, ArmSlotsGetPositionAndGains)
{
    unitree_hg::msg::LowCmd           cmd{};
    const auto                        motor_index = realMotorIndexMap();
    std::array<double, kNumArmJoints> position{};
    std::array<double, kNumArmJoints> kp{};
    std::array<double, kNumArmJoints> kd{};
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        position[i] = 0.1 * static_cast<double>(i) - 0.5;
        kp[i]       = 25.0 + static_cast<double>(i);
        kd[i]       = 1.0;
    }

    assembleLowCmd(cmd, motor_index, position, kp, kd, 0.3F, waistHold(), 160.0, 4.0);

    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        const auto& motor = cmd.motor_cmd[static_cast<std::size_t>(motor_index[i])];
        EXPECT_FLOAT_EQ(motor.q, static_cast<float>(position[i]));
        EXPECT_FLOAT_EQ(motor.dq, 0.0F);
        EXPECT_FLOAT_EQ(motor.tau, 0.0F);
        EXPECT_FLOAT_EQ(motor.kp, static_cast<float>(kp[i]));
        EXPECT_FLOAT_EQ(motor.kd, static_cast<float>(kd[i]));
        // Mirrors Unitree's own arm_sdk example: it never sets a motor's `mode`.
        EXPECT_EQ(motor.mode, 0U);
    }
}

TEST(AssembleLowCmd, WaistSlotsHoldTheirLatchedPositionAtWaistGains)
{
    // Left out of the command entirely until now, which sent the waist kp=kd=0 while the blend
    // weight was up -- a torso with no stiffness under arm load. Unitree's own arm_sdk example
    // commands these three alongside the arms.
    unitree_hg::msg::LowCmd                 cmd{};
    const auto                              motor_index = realMotorIndexMap();
    const std::array<double, kNumArmJoints> position{};
    const std::array<double, kNumArmJoints> kp{};
    const std::array<double, kNumArmJoints> kd{};
    const auto                              hold = waistHold();

    assembleLowCmd(cmd, motor_index, position, kp, kd, 1.0F, hold, 160.0, 4.0);

    for (std::size_t i = 0; i < kNumWaistJoints; ++i)
    {
        const auto& motor = cmd.motor_cmd[static_cast<std::size_t>(kWaistMotorIndex[i])];
        EXPECT_FLOAT_EQ(motor.q, static_cast<float>(hold[i]));
        EXPECT_FLOAT_EQ(motor.dq, 0.0F);
        EXPECT_FLOAT_EQ(motor.tau, 0.0F);
        EXPECT_FLOAT_EQ(motor.kp, 160.0F);
        EXPECT_FLOAT_EQ(motor.kd, 4.0F);
        EXPECT_EQ(motor.mode, 0U);
    }
}

TEST(AssembleLowCmd, WaistSlotsAreTheThreeUnitreeNames)
{
    // 12, 13, 14 = WAIST_YAW, WAIST_ROLL, WAIST_PITCH in G1Arm7JointIndex. Asserted rather than
    // trusted: getting this wrong writes stiff gains onto a leg.
    EXPECT_EQ(kWaistMotorIndex[0], 12);
    EXPECT_EQ(kWaistMotorIndex[1], 13);
    EXPECT_EQ(kWaistMotorIndex[2], 14);
}

TEST(AssembleLowCmd, WeightSlotIsPlacedAtMotorCmd29)
{
    unitree_hg::msg::LowCmd                 cmd{};
    const auto                              motor_index = realMotorIndexMap();
    const std::array<double, kNumArmJoints> position{};
    const std::array<double, kNumArmJoints> kp{};
    const std::array<double, kNumArmJoints> kd{};

    assembleLowCmd(cmd, motor_index, position, kp, kd, 0.42F, waistHold(), 160.0, 4.0);

    ASSERT_EQ(kWeightMotorIndex, 29U);
    EXPECT_FLOAT_EQ(cmd.motor_cmd[29].q, 0.42F);
}

TEST(AssembleLowCmd, NeverTouchesModePrOrModeMachine)
{
    unitree_hg::msg::LowCmd                 cmd{};
    const auto                              motor_index = realMotorIndexMap();
    const std::array<double, kNumArmJoints> position{};
    const std::array<double, kNumArmJoints> kp{};
    const std::array<double, kNumArmJoints> kd{};

    assembleLowCmd(cmd, motor_index, position, kp, kd, 1.0F, waistHold(), 160.0, 4.0);

    EXPECT_EQ(cmd.mode_pr, 0U);
    EXPECT_EQ(cmd.mode_machine, 0U);
}

}  // namespace
}  // namespace g1_hardware_interface
