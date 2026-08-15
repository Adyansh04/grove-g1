/**
 * @file test_lowcmd_assembly.cpp
 * @brief Unit tests for the rt/lowcmd mode table and per-motor packing.
 */
#include <gmock/gmock.h>

#include "g1_hardware_interface/lowcmd_assembly.hpp"
#include "unitree_hg/msg/low_cmd.hpp"

namespace g1_hardware_interface
{
namespace
{

constexpr PositionOnlyGains kFallback{ 10.0, 1.0 };

JointCommand commandFixture() { return JointCommand{ 0.25, 1.5, -3.0, 80.0, 2.0 }; }

TEST(ResolveJointMode, ImpedanceWinsOverEveryOtherClaim)
{
    // kp+kd is the only claim that carries its own gains, so it outranks the rest.
    EXPECT_EQ(
        resolveJointMode(InterfaceClaims{ true, true, true, true }),
        JointControlMode::kImpedance);
    EXPECT_EQ(
        resolveJointMode(InterfaceClaims{ false, false, false, true }),
        JointControlMode::kImpedance);
}

TEST(ResolveJointMode, EffortOutranksPosition)
{
    EXPECT_EQ(
        resolveJointMode(InterfaceClaims{ true, false, true, false }),
        JointControlMode::kEffort);
}

TEST(ResolveJointMode, PositionAloneIsPositionOnly)
{
    EXPECT_EQ(
        resolveJointMode(InterfaceClaims{ true, false, false, false }),
        JointControlMode::kPositionOnly);
}

TEST(ResolveJointMode, VelocityAloneIsDisabled)
{
    // There is no velocity-only mode on this hardware: with kp and kd both zero the firmware's
    // law has nothing left to act on, so claiming velocity alone must not look like control.
    EXPECT_EQ(
        resolveJointMode(InterfaceClaims{ false, true, false, false }),
        JointControlMode::kDisabled);
}

TEST(ResolveJointMode, NoClaimsIsDisabled)
{
    EXPECT_EQ(resolveJointMode(InterfaceClaims{}), JointControlMode::kDisabled);
}

TEST(FillMotorCmd, ImpedancePassesEveryCommandedFieldThrough)
{
    unitree_hg::msg::MotorCmd motor{};
    fillMotorCmd(motor, JointControlMode::kImpedance, commandFixture(), kFallback, 99.0);

    EXPECT_EQ(motor.mode, 1U);
    EXPECT_FLOAT_EQ(motor.q, 0.25F);
    EXPECT_FLOAT_EQ(motor.dq, 1.5F);
    EXPECT_FLOAT_EQ(motor.tau, -3.0F);
    EXPECT_FLOAT_EQ(motor.kp, 80.0F);
    EXPECT_FLOAT_EQ(motor.kd, 2.0F);
}

TEST(FillMotorCmd, EffortPinsPositionToTheMeasurementAndZeroesStiffness)
{
    unitree_hg::msg::MotorCmd motor{};
    fillMotorCmd(motor, JointControlMode::kEffort, commandFixture(), kFallback, 99.0);

    // q must sit on the measurement, not the stale position command: kp is zero here, but a
    // later gain change would otherwise turn a forgotten setpoint into a lurch.
    EXPECT_FLOAT_EQ(motor.q, 99.0F);
    EXPECT_FLOAT_EQ(motor.kp, 0.0F);
    EXPECT_FLOAT_EQ(motor.tau, -3.0F);
    EXPECT_FLOAT_EQ(motor.kd, 2.0F);
}

TEST(FillMotorCmd, PositionOnlyUsesFallbackGainsAndNoFeedforward)
{
    unitree_hg::msg::MotorCmd motor{};
    fillMotorCmd(motor, JointControlMode::kPositionOnly, commandFixture(), kFallback, 99.0);

    EXPECT_FLOAT_EQ(motor.q, 0.25F);
    EXPECT_FLOAT_EQ(motor.dq, 0.0F);
    EXPECT_FLOAT_EQ(motor.tau, 0.0F);
    EXPECT_FLOAT_EQ(motor.kp, 10.0F);
    EXPECT_FLOAT_EQ(motor.kd, 1.0F);
}

TEST(FillMotorCmd, DisabledZeroesEverythingIncludingTheModeByte)
{
    unitree_hg::msg::MotorCmd motor{};
    // Pre-load the slot so the test fails if the branch leaves a previous tick's values behind.
    fillMotorCmd(motor, JointControlMode::kImpedance, commandFixture(), kFallback, 99.0);
    fillMotorCmd(motor, JointControlMode::kDisabled, commandFixture(), kFallback, 99.0);

    EXPECT_EQ(motor.mode, 0U);
    EXPECT_FLOAT_EQ(motor.q, 0.0F);
    EXPECT_FLOAT_EQ(motor.dq, 0.0F);
    EXPECT_FLOAT_EQ(motor.tau, 0.0F);
    EXPECT_FLOAT_EQ(motor.kp, 0.0F);
    EXPECT_FLOAT_EQ(motor.kd, 0.0F);
}

TEST(FillReleaseCmd, StiffnessFadesWhileDampingAndHoldPositionStay)
{
    unitree_hg::msg::MotorCmd start{};
    fillReleaseCmd(start, 0.4, 80.0, 1.0, 3.0);
    EXPECT_FLOAT_EQ(start.q, 0.4F);
    EXPECT_FLOAT_EQ(start.kp, 80.0F);
    EXPECT_FLOAT_EQ(start.kd, 3.0F);

    unitree_hg::msg::MotorCmd midway{};
    fillReleaseCmd(midway, 0.4, 80.0, 0.5, 3.0);
    EXPECT_FLOAT_EQ(midway.kp, 40.0F);

    // Damping must outlive the stiffness, otherwise the last tick of the ramp is a free drop.
    unitree_hg::msg::MotorCmd finish{};
    fillReleaseCmd(finish, 0.4, 80.0, 0.0, 3.0);
    EXPECT_FLOAT_EQ(finish.kp, 0.0F);
    EXPECT_FLOAT_EQ(finish.kd, 3.0F);
    EXPECT_FLOAT_EQ(finish.q, 0.4F);
    EXPECT_EQ(finish.mode, 1U);
}

TEST(FillReleaseCmd, NeverCommandsTorque)
{
    unitree_hg::msg::MotorCmd motor{};
    fillReleaseCmd(motor, 0.4, 80.0, 0.7, 3.0);
    EXPECT_FLOAT_EQ(motor.tau, 0.0F);
    EXPECT_FLOAT_EQ(motor.dq, 0.0F);
}

}  // namespace
}  // namespace g1_hardware_interface
