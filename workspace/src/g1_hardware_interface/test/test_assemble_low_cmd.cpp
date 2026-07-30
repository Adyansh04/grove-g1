/**
 * @file test_assemble_low_cmd.cpp
 * @brief Unit tests for assembleLowCmd() -- arm slot writes, the weight slot, and untouched slots.
 */
#include <gmock/gmock.h>

#include <array>

#include "g1_hardware_interface/g1_arm_sdk_system.hpp"
#include "unitree_hg/msg/low_cmd.hpp"

namespace g1_hardware_interface
{
namespace
{

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

    assembleLowCmd(cmd, motor_index, position, kp, kd, 0.7F);

    for (std::size_t slot = 0; slot < cmd.motor_cmd.size(); ++slot)
    {
        const bool is_arm_slot    = slot >= 15 && slot <= 28;
        const bool is_weight_slot = slot == kWeightMotorIndex;
        if (is_arm_slot || is_weight_slot)
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

    assembleLowCmd(cmd, motor_index, position, kp, kd, 0.3F);

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

TEST(AssembleLowCmd, WeightSlotIsPlacedAtMotorCmd29)
{
    unitree_hg::msg::LowCmd                 cmd{};
    const auto                              motor_index = realMotorIndexMap();
    const std::array<double, kNumArmJoints> position{};
    const std::array<double, kNumArmJoints> kp{};
    const std::array<double, kNumArmJoints> kd{};

    assembleLowCmd(cmd, motor_index, position, kp, kd, 0.42F);

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

    assembleLowCmd(cmd, motor_index, position, kp, kd, 1.0F);

    EXPECT_EQ(cmd.mode_pr, 0U);
    EXPECT_EQ(cmd.mode_machine, 0U);
}

}  // namespace
}  // namespace g1_hardware_interface
