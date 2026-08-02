/**
 * @file test_walk_policy.cpp
 * @brief Pins the walking policy's wire contract: joint order, observation layout, action mapping,
 * and the velocity latch's dead-man -- all without a live node, DDS, or the sim.
 */
#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <vector>

#include "g1_bringup/walk_policy.hpp"

namespace g1_bringup
{
namespace
{

using namespace std::chrono_literals;
using ::testing::FloatEq;

PolicyConfig makeConfig()
{
    PolicyConfig config;
    // Distinct per-joint values so an off-by-one in any index arithmetic shows up as a wrong
    // number rather than coincidentally matching its neighbour.
    for (std::size_t i = 0; i < kNumBodyMotors; ++i)
    {
        config.default_joint_pos[i] = 0.01 * static_cast<double>(i);
        config.action_scales[i]     = 0.1 + 0.001 * static_cast<double>(i);
    }
    config.max_velocity              = { 1.0, 0.8, 2.0 };
    config.gait_initiation_threshold = { 0.4, 0.5, 1.5 };
    config.velocity_duration_max_s   = 2.0;
    return config;
}

std::vector<std::string> ddsOrderAsVector()
{
    return std::vector<std::string>(kDdsMotorOrder.begin(), kDdsMotorOrder.end());
}

// --- joint order -----------------------------------------------------------------------------

TEST(WalkPolicyJointOrder, DdsMotorOrderMatchesTheVendorTable)
{
    // Spot-checks the group boundaries from unitree_mujoco's g1_joint_index_dds.md 29DOF table:
    // a permutation inside a group would still land every gain on the wrong joint.
    ASSERT_EQ(kDdsMotorOrder.size(), 29U);
    EXPECT_STREQ(kDdsMotorOrder[0], "left_hip_pitch_joint");
    EXPECT_STREQ(kDdsMotorOrder[5], "left_ankle_roll_joint");
    EXPECT_STREQ(kDdsMotorOrder[6], "right_hip_pitch_joint");
    EXPECT_STREQ(kDdsMotorOrder[11], "right_ankle_roll_joint");
    EXPECT_STREQ(kDdsMotorOrder[12], "waist_yaw_joint");
    EXPECT_STREQ(kDdsMotorOrder[14], "waist_pitch_joint");
    EXPECT_STREQ(kDdsMotorOrder[kFirstArmMotor], "left_shoulder_pitch_joint");
    EXPECT_STREQ(kDdsMotorOrder[22], "right_shoulder_pitch_joint");
    EXPECT_STREQ(kDdsMotorOrder[28], "right_wrist_yaw_joint");
}

TEST(WalkPolicyJointOrder, LowerMotorsAreLegsPlusWaist)
{
    EXPECT_EQ(kNumLowerMotors, 15);
    EXPECT_EQ(kNumLowerMotors, kFirstArmMotor);
}

TEST(WalkPolicyJointOrder, MatchingOrderIsAccepted)
{
    EXPECT_EQ(checkJointOrder(ddsOrderAsVector()), "");
}

TEST(WalkPolicyJointOrder, SwappedNamesAreRejectedWithTheOffendingIndex)
{
    auto names = ddsOrderAsVector();
    std::swap(names[22], names[15]);  // the exact class of mistake the reference package made
    const auto problem = checkJointOrder(names);
    EXPECT_THAT(problem, ::testing::HasSubstr("joint_names[15]"));
    EXPECT_THAT(problem, ::testing::HasSubstr("right_shoulder_pitch_joint"));
}

TEST(WalkPolicyJointOrder, WrongLengthIsRejected)
{
    auto names = ddsOrderAsVector();
    names.pop_back();
    EXPECT_THAT(checkJointOrder(names), ::testing::HasSubstr("expected 29"));
}

// --- observation layout ----------------------------------------------------------------------

TEST(WalkPolicyObservation, SectionOffsetsFormTheTrainedLayout)
{
    EXPECT_EQ(kObsBaseLinVel, 0U);
    EXPECT_EQ(kObsBaseAngVel, 3U);
    EXPECT_EQ(kObsGravity, 6U);
    EXPECT_EQ(kObsJointPos, 9U);
    EXPECT_EQ(kObsJointVel, 38U);
    EXPECT_EQ(kObsLastAction, 67U);
    EXPECT_EQ(kObsCommand, 96U);
    EXPECT_EQ(kObsDim, 99U);
}

TEST(WalkPolicyObservation, JointPosIsRelativeToDefaultAndJointVelIsNot)
{
    const auto   config = makeConfig();
    PolicyInputs inputs;
    for (std::size_t i = 0; i < kNumBodyMotors; ++i)
    {
        inputs.joint_pos[i] = config.default_joint_pos[i] + 0.25;
        inputs.joint_vel[i] = 0.5 * static_cast<double>(i);
    }
    const auto obs = assembleObservation(inputs, config, {}, { 0.0, 0.0, 0.0 });

    for (std::size_t i = 0; i < kNumBodyMotors; ++i)
    {
        EXPECT_NEAR(obs[kObsJointPos + i], 0.25F, 1e-5F) << "joint_pos[" << i << "]";
        EXPECT_NEAR(obs[kObsJointVel + i], 0.5F * static_cast<float>(i), 1e-5F)
            << "joint_vel[" << i << "]";
    }
}

TEST(WalkPolicyObservation, IsNotNormalised)
{
    // The exported graph starts with Sub(obs_mean) then Div(obs_std), so the observation must
    // reach the tensor verbatim. Normalising here would apply it twice and silently wreck the
    // policy -- model_config.json's obs_mean/obs_std are duplicates of the baked-in constants.
    const auto   config = makeConfig();
    PolicyInputs inputs;
    inputs.base_ang_vel_body = { 7.5, -3.25, 0.125 };
    const auto obs           = assembleObservation(inputs, config, {}, { 0.0, 0.0, 0.0 });

    EXPECT_THAT(obs[kObsBaseAngVel + 0], FloatEq(7.5F));
    EXPECT_THAT(obs[kObsBaseAngVel + 1], FloatEq(-3.25F));
    EXPECT_THAT(obs[kObsBaseAngVel + 2], FloatEq(0.125F));
}

TEST(WalkPolicyObservation, CommandAndLastActionAreCopiedThrough)
{
    const auto                    config = makeConfig();
    std::array<float, kActionDim> last_action{};
    for (std::size_t i = 0; i < kActionDim; ++i)
    {
        last_action[i] = 0.05F * static_cast<float>(i);
    }
    const auto obs = assembleObservation(PolicyInputs{}, config, last_action, { 0.6, -0.2, 1.1 });

    for (std::size_t i = 0; i < kActionDim; ++i)
    {
        EXPECT_THAT(obs[kObsLastAction + i], FloatEq(0.05F * static_cast<float>(i)));
    }
    EXPECT_THAT(obs[kObsCommand + 0], FloatEq(0.6F));
    EXPECT_THAT(obs[kObsCommand + 1], FloatEq(-0.2F));
    EXPECT_THAT(obs[kObsCommand + 2], FloatEq(1.1F));
}

TEST(WalkPolicyObservation, UprightBaseProjectsGravityStraightDown)
{
    const auto obs = assembleObservation(PolicyInputs{}, makeConfig(), {}, { 0.0, 0.0, 0.0 });
    EXPECT_NEAR(obs[kObsGravity + 0], 0.0F, 1e-6F);
    EXPECT_NEAR(obs[kObsGravity + 1], 0.0F, 1e-6F);
    EXPECT_NEAR(obs[kObsGravity + 2], -1.0F, 1e-6F);
}

TEST(WalkPolicyObservation, PitchedBaseTiltsProjectedGravity)
{
    // 90 deg pitch about +Y: world -Z should read as +X in the base frame.
    const double half = M_PI / 4.0;
    PolicyInputs inputs;
    inputs.base_quat = { std::cos(half), 0.0, std::sin(half), 0.0 };
    const auto obs   = assembleObservation(inputs, makeConfig(), {}, { 0.0, 0.0, 0.0 });

    EXPECT_NEAR(obs[kObsGravity + 0], 1.0F, 1e-5F);
    EXPECT_NEAR(obs[kObsGravity + 2], 0.0F, 1e-5F);
}

TEST(WalkPolicyObservation, BaseLinearVelocityIsRotatedIntoTheBaseFrame)
{
    // /sportmodestate reports world-frame velocity; the policy expects it base-relative. Yawed
    // 90 deg about +Z, a robot moving along world +X is moving along its own -Y.
    const double half = M_PI / 4.0;
    PolicyInputs inputs;
    inputs.base_quat          = { std::cos(half), 0.0, 0.0, std::sin(half) };
    inputs.base_lin_vel_world = { 1.0, 0.0, 0.0 };
    const auto obs            = assembleObservation(inputs, makeConfig(), {}, { 0.0, 0.0, 0.0 });

    EXPECT_NEAR(obs[kObsBaseLinVel + 0], 0.0F, 1e-5F);
    EXPECT_NEAR(obs[kObsBaseLinVel + 1], -1.0F, 1e-5F);
}

TEST(WalkPolicyObservation, IdentityQuaternionLeavesVectorsUnchanged)
{
    const auto out = rotateWorldToBase({ 1.0, 0.0, 0.0, 0.0 }, { 0.3, -0.7, 2.0 });
    EXPECT_NEAR(out[0], 0.3, 1e-9);
    EXPECT_NEAR(out[1], -0.7, 1e-9);
    EXPECT_NEAR(out[2], 2.0, 1e-9);
}

// --- action mapping --------------------------------------------------------------------------

TEST(WalkPolicyAction, MapsToDefaultPlusScaledAction)
{
    const auto                    config = makeConfig();
    std::array<float, kActionDim> action{};
    action.fill(2.0F);
    const auto targets = actionToJointTargets(action, config);

    for (std::size_t i = 0; i < kNumBodyMotors; ++i)
    {
        EXPECT_NEAR(targets[i], config.default_joint_pos[i] + 2.0 * config.action_scales[i], 1e-9)
            << "motor " << i;
    }
}

TEST(WalkPolicyAction, ZeroActionHoldsTheDefaultPosture)
{
    const auto config  = makeConfig();
    const auto targets = actionToJointTargets({}, config);
    for (std::size_t i = 0; i < kNumBodyMotors; ++i)
    {
        EXPECT_NEAR(targets[i], config.default_joint_pos[i], 1e-9) << "motor " << i;
    }
}

// --- velocity clamping, threshold, and the dead-man ---------------------------------------------

TEST(WalkPolicyVelocity, IsClampedPerAxis)
{
    const auto config  = makeConfig();
    const auto clamped = clampVelocity(5.0, -5.0, 9.0, config);
    EXPECT_DOUBLE_EQ(clamped[0], 1.0);
    EXPECT_DOUBLE_EQ(clamped[1], -0.8);
    EXPECT_DOUBLE_EQ(clamped[2], 2.0);
}

TEST(WalkPolicyVelocity, InRangeCommandsPassThroughUnchanged)
{
    const auto config  = makeConfig();
    const auto clamped = clampVelocity(0.6, -0.2, 1.1, config);
    EXPECT_DOUBLE_EQ(clamped[0], 0.6);
    EXPECT_DOUBLE_EQ(clamped[1], -0.2);
    EXPECT_DOUBLE_EQ(clamped[2], 1.1);
}

TEST(WalkPolicyVelocity, BelowThresholdCommandIsDetectedButNotModified)
{
    const auto config = makeConfig();
    EXPECT_TRUE(isBelowGaitThreshold({ 0.2, 0.0, 0.0 }, config));
    EXPECT_FALSE(isBelowGaitThreshold({ 0.45, 0.0, 0.0 }, config));
    EXPECT_FALSE(isBelowGaitThreshold({ 0.0, 0.0, -1.6 }, config));

    // The advisory check must never scale a command up -- clamping is the only transform applied.
    const auto clamped = clampVelocity(0.2, 0.0, 0.0, config);
    EXPECT_DOUBLE_EQ(clamped[0], 0.2);
}

TEST(WalkPolicyVelocity, LatchExpiresAfterTheRequestedDuration)
{
    const auto config = makeConfig();
    const auto t0     = std::chrono::steady_clock::time_point{} + 10s;
    const auto latch  = latchVelocity({ 0.6, 0.0, 0.0 }, 1.0, t0, config);

    EXPECT_DOUBLE_EQ(activeCommand(latch, t0)[0], 0.6);
    EXPECT_DOUBLE_EQ(activeCommand(latch, t0 + 999ms)[0], 0.6);
    EXPECT_DOUBLE_EQ(activeCommand(latch, t0 + 1001ms)[0], 0.0)
        << "the vendor's duration field is the dead-man -- a silent bridge must stop the robot";
}

TEST(WalkPolicyVelocity, DurationIsClampedSoAContinuousLatchCannotStick)
{
    const auto config = makeConfig();
    const auto t0     = std::chrono::steady_clock::time_point{} + 10s;
    // 864000 s is the vendor's "continuous" value, which this stack never sends and must never honour.
    const auto latch = latchVelocity({ 0.6, 0.0, 0.0 }, 864000.0, t0, config);

    EXPECT_DOUBLE_EQ(activeCommand(latch, t0 + 1900ms)[0], 0.6);
    EXPECT_DOUBLE_EQ(activeCommand(latch, t0 + 2100ms)[0], 0.0);
}

TEST(WalkPolicyVelocity, NoLatchMeansZeroCommand)
{
    const auto zero = activeCommand(std::nullopt, std::chrono::steady_clock::now());
    EXPECT_DOUBLE_EQ(zero[0], 0.0);
    EXPECT_DOUBLE_EQ(zero[1], 0.0);
    EXPECT_DOUBLE_EQ(zero[2], 0.0);
}

}  // namespace
}  // namespace g1_bringup
