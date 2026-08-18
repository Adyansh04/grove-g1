/**
 * @file test_wire_contract.cpp
 * @brief The parts of the Dex3 wire format that are easy to get wrong and expensive to discover
 * on hardware: the packed mode byte, the joint order, and what a command frame carries.
 */

#include <gmock/gmock.h>

#include <pluginlib/class_loader.hpp>

#include "g1_hand_interface/g1_dex3_system.hpp"
#include "hardware_interface/system_interface.hpp"

using g1_hand_interface::kJointSuffixes;
using g1_hand_interface::kNumHandJoints;
using g1_hand_interface::kStatusFoc;
using g1_hand_interface::kStatusLock;
using g1_hand_interface::packHandMotor;
using g1_hand_interface::packMode;

TEST(Dex3WireContract, ModePacksIdStatusAndTimeoutIntoTheirOwnFields)
{
    // id in bits 0-3, status in 4-6, timeout in 7. Getting the shift wrong produces a byte
    // the motor still accepts, so nothing complains and the wrong finger moves.
    EXPECT_EQ(packMode(0, kStatusLock, false), 0x00);
    EXPECT_EQ(packMode(3, kStatusFoc, false), 0x13);
    EXPECT_EQ(packMode(6, kStatusFoc, true), 0x96);
    EXPECT_EQ(packMode(6, kStatusLock, true), 0x86);
}

TEST(Dex3WireContract, EveryMotorCarriesItsOwnIndex)
{
    // id 15 is the broadcast address, so an index that overflowed the nibble would command
    // all seven motors at once with one finger's target.
    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        EXPECT_EQ(packMode(i, kStatusFoc, false) & 0x0F, i);
    }
}

TEST(Dex3WireContract, JointOrderIsThumbThenMiddleThenIndex)
{
    // Unitree's own Dex3_1_Right_JointIndex enum lists index before middle, contradicting
    // their documented order. It is inert in their code, but transcribing it here would
    // close the wrong fingers, so this order is pinned rather than trusted.
    EXPECT_THAT(
        kJointSuffixes,
        ::testing::ElementsAre(
            ::testing::StrEq("thumb_0"),
            ::testing::StrEq("thumb_1"),
            ::testing::StrEq("thumb_2"),
            ::testing::StrEq("middle_0"),
            ::testing::StrEq("middle_1"),
            ::testing::StrEq("index_0"),
            ::testing::StrEq("index_1")));
}

TEST(Dex3WireContract, ReleaseLocksTheMotorAtItsMeasuredPositionWithNoStiffness)
{
    // The release frame is what a deactivate leaves on the wire. Non-zero gains here would
    // leave the fingers driving toward a target nothing is updating any more.
    unitree_hg::msg::dds_::MotorCmd_ motor{};
    packHandMotor(motor, 2, false, 0.42, 1.5, 0.2);

    EXPECT_EQ(motor.mode(), packMode(2, kStatusLock, true));
    EXPECT_FLOAT_EQ(motor.q(), 0.42F);
    EXPECT_FLOAT_EQ(motor.kp(), 0.0F);
    EXPECT_FLOAT_EQ(motor.kd(), 0.0F);
}

TEST(Dex3WireContract, DrivenCarriesTheGainsAndLeavesTheTimeoutDisarmed)
{
    // Arming the timeout while driving would stop the fingers a second after any hiccup in the
    // control loop, so this pins the flag as much as the gains.
    unitree_hg::msg::dds_::MotorCmd_ motor{};
    packHandMotor(motor, 5, true, -0.7, 1.5, 0.2);

    EXPECT_EQ(motor.mode(), packMode(5, kStatusFoc, false));
    EXPECT_FLOAT_EQ(motor.q(), -0.7F);
    EXPECT_FLOAT_EQ(motor.kp(), 1.5F);
    EXPECT_FLOAT_EQ(motor.kd(), 0.2F);
    EXPECT_FLOAT_EQ(motor.dq(), 0.0F);
    EXPECT_FLOAT_EQ(motor.tau(), 0.0F);
}

/**
 * @brief Confirms g1_hand_interface/G1Dex3System is discoverable through pluginlib's
 * ament-index lookup, the same path controller_manager uses, rather than merely compiling.
 */
TEST(G1Dex3SystemPluginlib, DiscoversAndInstantiates)
{
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
        "hardware_interface",
        "hardware_interface::SystemInterface");

    ASSERT_TRUE(loader.isClassAvailable("g1_hand_interface/G1Dex3System"));

    auto instance = loader.createUniqueInstance("g1_hand_interface/G1Dex3System");
    ASSERT_NE(instance, nullptr);
}
