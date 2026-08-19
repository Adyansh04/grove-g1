/**
 * @file test_agile_policy.cpp
 * @brief Loads the installed policy, pinning the ONNX contract and the joint tables together.
 */

#include <gmock/gmock.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <set>

#include "g1_controllers/agile_policy.hpp"

namespace
{

using g1_controllers::agileActionIndex;
using g1_controllers::agileObsIndex;
using g1_controllers::AgilePolicy;
using g1_controllers::kAgileActionJointNames;
using g1_controllers::kAgileObsJointNames;
using g1_controllers::kNumActJoints;
using g1_controllers::kNumObsJoints;
using g1_controllers::PolicyAction;
using g1_controllers::PolicyObservation;

std::string policyPath()
{
    return ament_index_cpp::get_package_share_directory("g1_controllers") +
           "/policy/unitree_g1_velocity_e2e.onnx";
}

/// Nominal standing pose, in the observation ordering. Matches the descriptor's default_joint_pos.
PolicyObservation standingObservation()
{
    PolicyObservation obs;
    obs.joint_position.at(*agileObsIndex("left_hip_pitch_joint"))    = -0.1F;
    obs.joint_position.at(*agileObsIndex("right_hip_pitch_joint"))   = -0.1F;
    obs.joint_position.at(*agileObsIndex("left_knee_joint"))         = 0.3F;
    obs.joint_position.at(*agileObsIndex("right_knee_joint"))        = 0.3F;
    obs.joint_position.at(*agileObsIndex("left_ankle_pitch_joint"))  = -0.2F;
    obs.joint_position.at(*agileObsIndex("right_ankle_pitch_joint")) = -0.2F;
    return obs;
}

TEST(AgileJointTables, ObservationAndActionOrderingsAreDistinct)
{
    // waist_yaw is observed but never commanded, which is what leaves it free for MoveIt.
    EXPECT_TRUE(agileObsIndex("waist_yaw_joint").has_value());
    EXPECT_FALSE(agileActionIndex("waist_yaw_joint").has_value());

    // Same joint, different slot in each ordering: the two tables are not interchangeable.
    EXPECT_EQ(agileObsIndex("waist_roll_joint"), 5U);
    EXPECT_EQ(agileActionIndex("waist_roll_joint"), 4U);
}

TEST(AgileJointTables, EveryActionJointIsAlsoObserved)
{
    for (const auto& name : kAgileActionJointNames)
    {
        EXPECT_TRUE(agileObsIndex(name).has_value()) << name << " is commanded but not observed";
    }
}

TEST(AgileJointTables, NamesAreUniqueAndComplete)
{
    const std::set<std::string> obs(kAgileObsJointNames.begin(), kAgileObsJointNames.end());
    const std::set<std::string> act(kAgileActionJointNames.begin(), kAgileActionJointNames.end());
    EXPECT_EQ(obs.size(), kNumObsJoints);
    EXPECT_EQ(act.size(), kNumActJoints);
}

TEST(AgileJointTables, UnknownJointHasNoIndex)
{
    EXPECT_FALSE(agileObsIndex("left_hand_thumb_0_joint").has_value());
    EXPECT_FALSE(agileActionIndex("not_a_joint").has_value());
}

TEST(AgileJointTables, ArmsAreNeverCommanded)
{
    for (const auto& name : kAgileActionJointNames)
    {
        EXPECT_EQ(name.find("shoulder"), std::string::npos) << name;
        EXPECT_EQ(name.find("elbow"), std::string::npos) << name;
        EXPECT_EQ(name.find("wrist"), std::string::npos) << name;
    }
}

TEST(AgilePolicyModel, RejectsAModelThatIsNotThePolicy)
{
    EXPECT_THROW(AgilePolicy("/nonexistent/policy.onnx"), Ort::Exception);
}

TEST(AgilePolicyModel, StandingObservationProducesTheNominalPose)
{
    AgilePolicy policy(policyPath());

    PolicyAction action;
    ASSERT_TRUE(policy.run(standingObservation(), action));

    // The graph applies its own scale and offset, so outputs are absolute radians near the
    // nominal stance rather than raw actions around zero.
    EXPECT_NEAR(action.joint_position.at(*agileActionIndex("left_knee_joint")), 0.3F, 0.25F);
    EXPECT_NEAR(action.joint_position.at(*agileActionIndex("left_hip_pitch_joint")), -0.1F, 0.25F);

    // Gains come out of the graph; these are the descriptor's stiffness values for those joints.
    EXPECT_NEAR(action.kp.at(*agileActionIndex("left_knee_joint")), 200.0F, 1.0F);
    EXPECT_NEAR(action.kp.at(*agileActionIndex("waist_roll_joint")), 300.0F, 1.0F);
    EXPECT_NEAR(action.kd.at(*agileActionIndex("left_ankle_roll_joint")), 0.1F, 0.01F);
}

TEST(AgilePolicyModel, HistoryMakesRepeatedTicksDiffer)
{
    AgilePolicy policy(policyPath());
    const auto  obs = standingObservation();

    PolicyAction first;
    PolicyAction second;
    ASSERT_TRUE(policy.run(obs, first));
    ASSERT_TRUE(policy.run(obs, second));

    // Identical input, different output: proof the fed-back history is actually reaching the graph.
    EXPECT_NE(first.joint_position, second.joint_position);
}

TEST(AgilePolicyModel, ResetReturnsToTheColdStartOutput)
{
    AgilePolicy policy(policyPath());
    const auto  obs = standingObservation();

    PolicyAction cold;
    ASSERT_TRUE(policy.run(obs, cold));
    PolicyAction warm;
    ASSERT_TRUE(policy.run(obs, warm));
    ASSERT_NE(cold.joint_position, warm.joint_position);

    policy.reset();
    PolicyAction after_reset;
    ASSERT_TRUE(policy.run(obs, after_reset));
    EXPECT_EQ(cold.joint_position, after_reset.joint_position);
}

TEST(AgilePolicyModel, OutputsStayFiniteUnderAToppledObservation)
{
    AgilePolicy policy(policyPath());

    // Lying on its side with the gyro saturated: out of distribution, but must not emit NaN.
    PolicyObservation obs = standingObservation();
    obs.root_quat_wxyz    = { 0.707F, 0.707F, 0.0F, 0.0F };
    obs.root_ang_vel_b    = { 8.0F, -8.0F, 4.0F };
    obs.velocity_command  = { 1.0F, -1.0F, 2.0F };

    PolicyAction action;
    ASSERT_TRUE(policy.run(obs, action));
    for (std::size_t i = 0; i < kNumActJoints; ++i)
    {
        EXPECT_TRUE(std::isfinite(action.joint_position.at(i))) << kAgileActionJointNames.at(i);
        EXPECT_TRUE(std::isfinite(action.kp.at(i)));
        EXPECT_TRUE(std::isfinite(action.kd.at(i)));
    }
}

}  // namespace
