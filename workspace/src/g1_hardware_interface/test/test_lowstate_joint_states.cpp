/**
 * @file test_lowstate_joint_states.cpp
 * @brief The LowState -> JointState mapping the robot's TF tree depends on.
 *
 * Motor index IS the index into the name table, so a permutation here puts a knee angle on a
 * waist joint and moves every frame above the torso. g1_description's test_motor_order covers
 * the other half: that this table still agrees with the simulator's.
 */

#include <gmock/gmock.h>

#include "g1_hardware_interface/lowstate_joint_states.hpp"

namespace
{
using g1_hardware_interface::fillLowerBodyJointState;
using g1_hardware_interface::initLowerBodyJointState;
using g1_hardware_interface::kLowerMotorJointNames;
using g1_hardware_interface::kNumLowerMotors;

unitree_hg::msg::LowState makeLowState()
{
    unitree_hg::msg::LowState state;
    for (std::size_t i = 0; i < state.motor_state.size(); ++i)
    {
        state.motor_state[i].q  = static_cast<float>(i) * 0.01F;
        state.motor_state[i].dq = static_cast<float>(i) * -0.1F;
    }
    return state;
}
}  // namespace

TEST(LowStateJointStates, NamesAreTheLowerBodyInDdsMotorOrder)
{
    sensor_msgs::msg::JointState msg;
    initLowerBodyJointState(msg);

    ASSERT_EQ(msg.name.size(), kNumLowerMotors);
    ASSERT_EQ(msg.position.size(), kNumLowerMotors);
    ASSERT_EQ(msg.velocity.size(), kNumLowerMotors);

    // The three that matter for the sensor chain: pelvis -> torso_link crosses exactly these,
    // at exactly these motor indices.
    EXPECT_EQ(msg.name[12], "waist_yaw_joint");
    EXPECT_EQ(msg.name[13], "waist_roll_joint");
    EXPECT_EQ(msg.name[14], "waist_pitch_joint");
    EXPECT_EQ(msg.name[0], "left_hip_pitch_joint");
    EXPECT_EQ(msg.name[6], "right_hip_pitch_joint");
    EXPECT_THAT(msg.name, ::testing::Each(::testing::Not(::testing::HasSubstr("shoulder"))));
}

TEST(LowStateJointStates, PositionsComeOffTheMatchingMotorSlot)
{
    sensor_msgs::msg::JointState msg;
    initLowerBodyJointState(msg);

    fillLowerBodyJointState(makeLowState(), msg);
    for (std::size_t i = 0; i < kNumLowerMotors; ++i)
    {
        EXPECT_NEAR(msg.position[i], static_cast<double>(i) * 0.01, 1e-6)
            << kLowerMotorJointNames[i];
        EXPECT_NEAR(msg.velocity[i], static_cast<double>(i) * -0.1, 1e-6)
            << kLowerMotorJointNames[i];
    }
}

TEST(LowStateJointStates, StopsAtTheWaistAndDoesNotReachTheArms)
{
    sensor_msgs::msg::JointState msg;
    initLowerBodyJointState(msg);
    fillLowerBodyJointState(makeLowState(), msg);

    // Motor 15 is left_shoulder_pitch, which joint_state_broadcaster owns. Publishing it here
    // too would have two sources writing one joint at different rates.
    EXPECT_EQ(msg.position.size(), kNumLowerMotors);
    EXPECT_NEAR(msg.position.back(), 14 * 0.01, 1e-6);
}
