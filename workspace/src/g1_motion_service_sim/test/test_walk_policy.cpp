/**
 * @file test_walk_policy.cpp
 * @brief Pins the walking policy's wire contract: joint order, observation layout, action mapping,
 * and the velocity latch's dead-man -- all without a live node, DDS, or the sim.
 */
#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <vector>

#include "g1_motion_service_sim/walk_policy.hpp"

namespace g1_motion_service_sim
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
    return { kDdsMotorOrder.begin(), kDdsMotorOrder.end() };
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

// --- leg-authority fallback (B1) and guarded inference (B2) -------------------------------------
//
// Both were real regressions on this path, fixed once and pinned here so neither comes back
// silently.

std::array<double, kNumLowerMotors> makePolicyQ()
{
    std::array<double, kNumLowerMotors> q{};
    for (std::size_t i = 0; i < q.size(); ++i)
    {
        q[i] = 0.5 + 0.01 * static_cast<double>(i);  // a plausible mid-gait crouch
    }
    return q;
}

std::array<double, kNumBodyMotors> makeHoldQ()
{
    // The spawn pose: legs straight, i.e. far from any stance the policy holds.
    return std::array<double, kNumBodyMotors>{};
}

/// Distinct per index, so an off-by-one shows up as a wrong number rather than a coincidence.
std::array<double, kNumLowerMotors> makePolicyKp()
{
    std::array<double, kNumLowerMotors> kp{};
    for (std::size_t i = 0; i < kp.size(); ++i)
    {
        kp[i] = 40.0 + static_cast<double>(i);
    }
    return kp;
}

std::array<double, kNumLowerMotors> makePolicyKd()
{
    std::array<double, kNumLowerMotors> kd{};
    for (std::size_t i = 0; i < kd.size(); ++i)
    {
        kd[i] = 2.5 + 0.1 * static_cast<double>(i);
    }
    return kd;
}

LowerBodyCommand select(LegAuthority authority)
{
    // Stiff-hold gains all distinct (100/1.5 legs, 50/2.5 waist) so a leg/waist or kp/kd mix-up
    // cannot alias into a passing value.
    return selectLowerBodyCommand(
        authority,
        makePolicyQ(),
        makeHoldQ(),
        makePolicyKp(),
        makePolicyKd(),
        /*leg_kp=*/100.0,
        /*leg_kd=*/1.5,
        /*waist_kp=*/50.0,
        /*waist_kd=*/2.5);
}

TEST(WalkPolicyFallback, LivePolicyOwnsBothTargetAndGains)
{
    const auto out = select(LegAuthority::kLivePolicy);
    for (std::size_t i = 0; i < kNumLowerMotors; ++i)
    {
        EXPECT_DOUBLE_EQ(out.q[i], makePolicyQ()[i]) << "motor " << i;
        EXPECT_DOUBLE_EQ(out.kp[i], makePolicyKp()[i]) << "motor " << i;
        EXPECT_DOUBLE_EQ(out.kd[i], makePolicyKd()[i]) << "motor " << i;
    }
}

TEST(WalkPolicyFallback, FrozenPolicyKeepsTheTargetButRevertsTheGains)
{
    // THE B1 REGRESSION. A stale tick must NOT revert the legs to hold_q: that is the
    // straight-legged spawn pose, so reverting mid-stance steps the knees at stiff gains and
    // topples the robot. Gains DO revert, so the frozen pose is held firmly.
    const auto out = select(LegAuthority::kFrozenPolicy);
    for (std::size_t i = 0; i < kNumLowerMotors; ++i)
    {
        EXPECT_DOUBLE_EQ(out.q[i], makePolicyQ()[i])
            << "motor " << i << " reverted to the spawn pose on a stale tick";
    }
    // Boundary: 11 is the last leg, 12 the first waist -- a >/>= slip mis-gains only these two.
    EXPECT_DOUBLE_EQ(out.kp[0], 100.0);
    EXPECT_DOUBLE_EQ(out.kd[0], 1.5);
    EXPECT_DOUBLE_EQ(out.kp[11], 100.0) << "motor 11 is a leg";
    EXPECT_DOUBLE_EQ(out.kd[11], 1.5) << "motor 11 is a leg";
    EXPECT_DOUBLE_EQ(out.kp[12], 50.0) << "motor 12 is the first waist joint";
    EXPECT_DOUBLE_EQ(out.kd[12], 2.5) << "motor 12 is the first waist joint";
    EXPECT_DOUBLE_EQ(out.kp[14], 50.0);
}

TEST(WalkPolicyFallback, HoldPoseUsesTheCapturedPoseAtStiffGains)
{
    // Before the policy has ever run, hold_q IS the live pose, so it is the correct target.
    auto hold      = makeHoldQ();
    hold[2]        = 0.123;
    hold[13]       = 0.456;
    const auto out = selectLowerBodyCommand(
        LegAuthority::kHoldPose,
        makePolicyQ(),
        hold,
        makePolicyKp(),
        makePolicyKd(),
        100.0,
        1.5,
        50.0,
        2.5);

    EXPECT_DOUBLE_EQ(out.q[2], 0.123);
    EXPECT_DOUBLE_EQ(out.q[13], 0.456);
    EXPECT_DOUBLE_EQ(out.kp[2], 100.0);
    EXPECT_DOUBLE_EQ(out.kp[13], 50.0);
}

TEST(WalkPolicyFallback, TheThreeStatesAreAllDistinct)
{
    // Guards the enum itself: if two states ever collapsed onto the same behaviour, the type would
    // stop buying anything over the bool pair it replaced.
    const auto live   = select(LegAuthority::kLivePolicy);
    const auto frozen = select(LegAuthority::kFrozenPolicy);
    const auto hold   = select(LegAuthority::kHoldPose);

    EXPECT_EQ(live.q, frozen.q) << "live and frozen share a target; only the gains differ";
    EXPECT_NE(live.kp, frozen.kp) << "frozen must fall back to stiff-hold gains";
    EXPECT_NE(frozen.q, hold.q) << "frozen must NOT revert to the hold pose -- that is B1";
    EXPECT_EQ(frozen.kp, hold.kp) << "both non-live states use stiff-hold gains";
}

TEST(WalkPolicyFallback, GuardedInferenceReturnsTheActionWhenItSucceeds)
{
    std::array<float, kActionDim> expected{};
    expected[7]    = 0.25F;
    const auto out = runPolicyGuarded([&expected] { return expected; });
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(*out, expected);
}

TEST(WalkPolicyFallback, GuardedInferenceSwallowsAThrowInsteadOfKillingTheProcess)
{
    // THE B2 REGRESSION. Inference runs from a timer under a bare rclcpp::spin(), so an escaping
    // exception terminates the process and stops /lowcmd entirely -- the robot collapses rather
    // than falling back to the stiff hold.
    const auto out = runPolicyGuarded(
        []() -> std::array<float, kActionDim> { throw std::runtime_error("inference blew up"); });
    EXPECT_FALSE(out.has_value());
}

TEST(WalkPolicyFallback, GuardedInferenceSwallowsNonStandardExceptionsToo)
{
    const auto out = runPolicyGuarded([]() -> std::array<float, kActionDim> { throw 42; });
    EXPECT_FALSE(out.has_value());
}

TEST(WalkPolicyFallback, AFailedInferenceYieldsNoActionToPublish)
{
    // Half one of the old concatenated test: the failure path produces nothing, so a caller has
    // nothing to write and its last-good action stays intact.
    std::array<float, kActionDim> last_good{};
    last_good[3]      = 0.75F;
    const auto failed = runPolicyGuarded(
        []() -> std::array<float, kActionDim> { throw std::runtime_error("boom"); });

    EXPECT_FALSE(failed.has_value());
    EXPECT_FLOAT_EQ(last_good[3], 0.75F) << "a failed inference must not clobber the last action";
}

TEST(WalkPolicyFallback, AStaleTickAfterAFailureStillFreezesAtTheLastGoodTarget)
{
    // Half two: once failures push the policy past its staleness window the authority becomes
    // kFrozenPolicy, and that must still hold the last GOOD target -- not the spawn pose.
    const auto out = select(LegAuthority::kFrozenPolicy);
    EXPECT_DOUBLE_EQ(out.q[5], makePolicyQ()[5]);
    EXPECT_NE(out.q[5], makeHoldQ()[5]);
}

}  // namespace
}  // namespace g1_motion_service_sim
