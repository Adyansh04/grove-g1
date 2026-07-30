/**
 * @file test_assemble_sim_low_cmd.cpp
 * @brief Unit tests for assembleSimLowCmd() -- per-slot hold/blend gains and the weight echo.
 */
#include <gmock/gmock.h>

#include <array>

#include "g1_bringup/blend_math.hpp"

namespace g1_bringup
{
namespace
{

constexpr double kLegKp     = 100.0;
constexpr double kLegKd     = 1.0;
constexpr double kWaistKp   = 50.0;
constexpr double kWaistKd   = 1.0;
constexpr double kArmHoldKp = 40.0;
constexpr double kArmHoldKd = 1.0;

std::array<double, kNumBodyMotors> makeHoldQ()
{
    std::array<double, kNumBodyMotors> hold_q{};
    for (std::size_t i = 0; i < hold_q.size(); ++i)
    {
        hold_q[i] = 0.01 * static_cast<double>(i);
    }
    return hold_q;
}

std::array<double, kNumArmMotors> makeArmCmdQ()
{
    std::array<double, kNumArmMotors> arm_cmd_q{};
    for (std::size_t i = 0; i < arm_cmd_q.size(); ++i)
    {
        arm_cmd_q[i] = 1.0 + 0.1 * static_cast<double>(i);
    }
    return arm_cmd_q;
}

TEST(AssembleSimLowCmd, LegSlotsHoldAtHoldQWithLegGains)
{
    const auto                              hold_q = makeHoldQ();
    const std::array<double, kNumArmMotors> arm_cmd_q{};
    const std::array<double, kNumArmMotors> arm_cmd_kp{};
    const std::array<double, kNumArmMotors> arm_cmd_kd{};

    const auto cmd = assembleSimLowCmd(
        hold_q,
        arm_cmd_q,
        arm_cmd_kp,
        arm_cmd_kd,
        /*weight=*/0.0,
        kLegKp,
        kLegKd,
        kWaistKp,
        kWaistKd,
        kArmHoldKp,
        kArmHoldKd);

    for (int i = 0; i < kNumLegMotors; ++i)
    {
        const auto& motor = cmd.motor_cmd[static_cast<std::size_t>(i)];
        EXPECT_FLOAT_EQ(motor.q, static_cast<float>(hold_q[static_cast<std::size_t>(i)]))
            << "slot " << i;
        EXPECT_FLOAT_EQ(motor.dq, 0.0F) << "slot " << i;
        EXPECT_FLOAT_EQ(motor.tau, 0.0F) << "slot " << i;
        EXPECT_FLOAT_EQ(motor.kp, static_cast<float>(kLegKp)) << "slot " << i;
        EXPECT_FLOAT_EQ(motor.kd, static_cast<float>(kLegKd)) << "slot " << i;
    }
}

TEST(AssembleSimLowCmd, WaistSlotsHoldAtHoldQWithWaistGains)
{
    const auto                              hold_q = makeHoldQ();
    const std::array<double, kNumArmMotors> arm_cmd_q{};
    const std::array<double, kNumArmMotors> arm_cmd_kp{};
    const std::array<double, kNumArmMotors> arm_cmd_kd{};

    const auto cmd = assembleSimLowCmd(
        hold_q,
        arm_cmd_q,
        arm_cmd_kp,
        arm_cmd_kd,
        /*weight=*/0.0,
        kLegKp,
        kLegKd,
        kWaistKp,
        kWaistKd,
        kArmHoldKp,
        kArmHoldKd);

    for (int i = kNumLegMotors; i < kFirstArmMotor; ++i)
    {
        const auto& motor = cmd.motor_cmd[static_cast<std::size_t>(i)];
        EXPECT_FLOAT_EQ(motor.q, static_cast<float>(hold_q[static_cast<std::size_t>(i)]))
            << "slot " << i;
        EXPECT_FLOAT_EQ(motor.dq, 0.0F) << "slot " << i;
        EXPECT_FLOAT_EQ(motor.tau, 0.0F) << "slot " << i;
        EXPECT_FLOAT_EQ(motor.kp, static_cast<float>(kWaistKp)) << "slot " << i;
        EXPECT_FLOAT_EQ(motor.kd, static_cast<float>(kWaistKd)) << "slot " << i;
    }
}

TEST(AssembleSimLowCmd, ArmSlotsBlendHoldAndCommandedByWeight)
{
    const auto                        hold_q    = makeHoldQ();
    const auto                        arm_cmd_q = makeArmCmdQ();
    std::array<double, kNumArmMotors> arm_cmd_kp{};
    std::array<double, kNumArmMotors> arm_cmd_kd{};
    arm_cmd_kp.fill(80.0);
    arm_cmd_kd.fill(2.0);
    constexpr double kWeight = 0.25;

    const auto cmd = assembleSimLowCmd(
        hold_q,
        arm_cmd_q,
        arm_cmd_kp,
        arm_cmd_kd,
        kWeight,
        kLegKp,
        kLegKd,
        kWaistKp,
        kWaistKd,
        kArmHoldKp,
        kArmHoldKd);

    for (int i = 0; i < kNumArmMotors; ++i)
    {
        const auto   idx         = static_cast<std::size_t>(i);
        const int    motor_index = kFirstArmMotor + i;
        const auto&  motor       = cmd.motor_cmd[static_cast<std::size_t>(motor_index)];
        const double expected_q =
            blend(hold_q[static_cast<std::size_t>(motor_index)], arm_cmd_q[idx], kWeight);
        const double expected_kp = blend(kArmHoldKp, arm_cmd_kp[idx], kWeight);
        const double expected_kd = blend(kArmHoldKd, arm_cmd_kd[idx], kWeight);

        EXPECT_FLOAT_EQ(motor.q, static_cast<float>(expected_q)) << "arm slot " << motor_index;
        EXPECT_FLOAT_EQ(motor.dq, 0.0F) << "arm slot " << motor_index;
        EXPECT_FLOAT_EQ(motor.tau, 0.0F) << "arm slot " << motor_index;
        EXPECT_FLOAT_EQ(motor.kp, static_cast<float>(expected_kp)) << "arm slot " << motor_index;
        EXPECT_FLOAT_EQ(motor.kd, static_cast<float>(expected_kd)) << "arm slot " << motor_index;
    }
}

TEST(AssembleSimLowCmd, WeightSlotEchoesEffectiveWeight)
{
    const auto                              hold_q = makeHoldQ();
    const std::array<double, kNumArmMotors> arm_cmd_q{};
    const std::array<double, kNumArmMotors> arm_cmd_kp{};
    const std::array<double, kNumArmMotors> arm_cmd_kd{};
    constexpr double                        kWeight = 0.42;

    const auto cmd = assembleSimLowCmd(
        hold_q,
        arm_cmd_q,
        arm_cmd_kp,
        arm_cmd_kd,
        kWeight,
        kLegKp,
        kLegKd,
        kWaistKp,
        kWaistKd,
        kArmHoldKp,
        kArmHoldKd);

    ASSERT_EQ(kWeightMotorIndex, 29U);
    EXPECT_FLOAT_EQ(cmd.motor_cmd[29].q, static_cast<float>(kWeight));
}

}  // namespace
}  // namespace g1_bringup
