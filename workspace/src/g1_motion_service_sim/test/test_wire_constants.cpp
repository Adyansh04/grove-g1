/**
 * @file test_wire_constants.cpp
 * @brief Pins the Unitree wire-layout constants this package duplicates from
 * g1_hardware_interface.
 *
 * Both packages independently encode the same vendor facts: the weight slot lives at motor
 * index 29, and there are 14 arm joints. They are duplicated rather than shared on purpose --
 * blend_math links only unitree_hg, and pulling in g1_hardware_interface to reach a constant
 * would drag the ros2_control plugin's headers into a pure math library.
 *
 * The cost of that choice is drift, so it is checked here instead. Same arrangement as
 * g1_navigation's test_gait_coupling, which exists because two numbers in different packages
 * had already silently disagreed once. This is a test target, so the heavier include stays
 * out of the shipped library.
 */
#include <gmock/gmock.h>

#include <cstddef>

#include "g1_hardware_interface/arm_ramp_engine.hpp"
#include "g1_hardware_interface/g1_arm_sdk_system.hpp"
#include "g1_motion_service_sim/blend_math.hpp"

namespace
{

// Compile-time, so a mismatch fails the build rather than waiting for the suite to run.
static_assert(
    g1_motion_service_sim::kWeightMotorIndex == g1_hardware_interface::kWeightMotorIndex,
    "the /lowcmd weight slot index disagrees between g1_motion_service_sim and "
    "g1_hardware_interface -- one of them is writing the weight to the wrong motor");

static_assert(
    static_cast<std::size_t>(g1_motion_service_sim::kNumArmMotors) ==
        g1_hardware_interface::kNumArmJoints,
    "the arm joint count disagrees between g1_motion_service_sim (kNumArmMotors) and "
    "g1_hardware_interface (kNumArmJoints)");

TEST(WireConstants, TheWeightSlotIndexAgreesAcrossPackages)
{
    EXPECT_EQ(g1_motion_service_sim::kWeightMotorIndex, g1_hardware_interface::kWeightMotorIndex);
}

TEST(WireConstants, TheArmJointCountAgreesAcrossPackages)
{
    // Deliberately compared through a cast: the two spellings also disagree on type, int here
    // and std::size_t there, which is the part most likely to bite in a mixed comparison.
    EXPECT_EQ(
        static_cast<std::size_t>(g1_motion_service_sim::kNumArmMotors),
        g1_hardware_interface::kNumArmJoints);
}

TEST(WireConstants, TheArmSliceEndsWhereTheWeightSlotBegins)
{
    // Not a duplication check but the invariant that makes both constants meaningful: arms
    // occupy [kFirstArmMotor, kFirstArmMotor + kNumArmMotors) and the weight slot sits just
    // past them. If this ever fails, the blend is writing over the weight or vice versa.
    EXPECT_EQ(
        static_cast<std::size_t>(g1_motion_service_sim::kNumBodyMotors),
        g1_motion_service_sim::kWeightMotorIndex);
}

}  // namespace
