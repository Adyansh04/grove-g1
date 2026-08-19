/**
 * @file test_lowcmd_assembly.cpp
 * @brief Unit tests for the rt/lowcmd mode table and per-motor packing.
 */
#include <gmock/gmock.h>

#include <cstring>

#include "g1_hardware_interface/lowcmd_assembly.hpp"

namespace g1_hardware_interface
{
namespace
{

constexpr PositionOnlyGains kFallback{ 10.0, 1.0 };

JointCommand commandFixture() { return JointCommand{ 0.25, 1.5, -3.0, 80.0, 2.0 }; }

TEST(ResolveJointMode, ImpedanceWinsOverEveryOtherClaim)
{
    // kp+kd is the only claim carrying its own gains, so it outranks the rest.
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
    // No velocity-only mode exists on this hardware, so the claim must not look like control.
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
    unitree_hg::msg::dds_::MotorCmd_ motor{};
    fillMotorCmd(motor, JointControlMode::kImpedance, commandFixture(), kFallback, 99.0);

    EXPECT_EQ(motor.mode(), 1U);
    EXPECT_FLOAT_EQ(motor.q(), 0.25F);
    EXPECT_FLOAT_EQ(motor.dq(), 1.5F);
    EXPECT_FLOAT_EQ(motor.tau(), -3.0F);
    EXPECT_FLOAT_EQ(motor.kp(), 80.0F);
    EXPECT_FLOAT_EQ(motor.kd(), 2.0F);
}

TEST(FillMotorCmd, EffortPinsPositionToTheMeasurementAndZeroesStiffness)
{
    unitree_hg::msg::dds_::MotorCmd_ motor{};
    fillMotorCmd(motor, JointControlMode::kEffort, commandFixture(), kFallback, 99.0);

    // A fresh MotorCmd_ is already mode 0, so without this a branch that forgot to enable the
    // motor would pass every other assertion here while leaving the joint unpowered.
    EXPECT_EQ(motor.mode(), 1);
    // q sits on the measurement, not a stale setpoint a later gain change could turn into a lurch.
    EXPECT_FLOAT_EQ(motor.q(), 99.0F);
    EXPECT_FLOAT_EQ(motor.kp(), 0.0F);
    EXPECT_FLOAT_EQ(motor.tau(), -3.0F);
    EXPECT_FLOAT_EQ(motor.kd(), 2.0F);
}

TEST(FillMotorCmd, PositionOnlyUsesFallbackGainsAndNoFeedforward)
{
    unitree_hg::msg::dds_::MotorCmd_ motor{};
    fillMotorCmd(motor, JointControlMode::kPositionOnly, commandFixture(), kFallback, 99.0);

    EXPECT_EQ(motor.mode(), 1);
    EXPECT_FLOAT_EQ(motor.q(), 0.25F);
    EXPECT_FLOAT_EQ(motor.dq(), 0.0F);
    EXPECT_FLOAT_EQ(motor.tau(), 0.0F);
    EXPECT_FLOAT_EQ(motor.kp(), 10.0F);
    EXPECT_FLOAT_EQ(motor.kd(), 1.0F);
}

TEST(FillMotorCmd, DisabledZeroesEverythingIncludingTheModeByte)
{
    unitree_hg::msg::dds_::MotorCmd_ motor{};
    // Pre-loaded, so the test fails if the branch leaves a previous tick's values behind.
    fillMotorCmd(motor, JointControlMode::kImpedance, commandFixture(), kFallback, 99.0);
    fillMotorCmd(motor, JointControlMode::kDisabled, commandFixture(), kFallback, 99.0);

    EXPECT_EQ(motor.mode(), 0U);
    EXPECT_FLOAT_EQ(motor.q(), 0.0F);
    EXPECT_FLOAT_EQ(motor.dq(), 0.0F);
    EXPECT_FLOAT_EQ(motor.tau(), 0.0F);
    EXPECT_FLOAT_EQ(motor.kp(), 0.0F);
    EXPECT_FLOAT_EQ(motor.kd(), 0.0F);
}

TEST(FillReleaseCmd, StiffnessFadesWhileDampingAndHoldPositionStay)
{
    unitree_hg::msg::dds_::MotorCmd_ start{};
    fillReleaseCmd(start, 0.4, 80.0, 1.0, 3.0);
    EXPECT_FLOAT_EQ(start.q(), 0.4F);
    EXPECT_FLOAT_EQ(start.kp(), 80.0F);
    EXPECT_FLOAT_EQ(start.kd(), 3.0F);

    unitree_hg::msg::dds_::MotorCmd_ midway{};
    fillReleaseCmd(midway, 0.4, 80.0, 0.5, 3.0);
    EXPECT_FLOAT_EQ(midway.kp(), 40.0F);

    // Damping outlives the stiffness, otherwise the ramp's last tick is a free drop.
    unitree_hg::msg::dds_::MotorCmd_ finish{};
    fillReleaseCmd(finish, 0.4, 80.0, 0.0, 3.0);
    EXPECT_FLOAT_EQ(finish.kp(), 0.0F);
    EXPECT_FLOAT_EQ(finish.kd(), 3.0F);
    EXPECT_FLOAT_EQ(finish.q(), 0.4F);
    EXPECT_EQ(finish.mode(), 1U);
}

TEST(FillReleaseCmd, NeverCommandsTorque)
{
    unitree_hg::msg::dds_::MotorCmd_ motor{};
    fillReleaseCmd(motor, 0.4, 80.0, 0.7, 3.0);
    EXPECT_FLOAT_EQ(motor.tau(), 0.0F);
    EXPECT_FLOAT_EQ(motor.dq(), 0.0F);
}

}  // namespace
}  // namespace g1_hardware_interface
