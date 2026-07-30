/**
 * @file test_walk_policy.cpp
 * @brief Pins the walking policy's observation layout and scales against the vendor source.
 */

#include <cmath>

#include "g1_bringup/walk_policy.hpp"
#include "gmock/gmock.h"

namespace
{
using g1_bringup::actionToJointTargets;
using g1_bringup::assembleObservation;
using g1_bringup::gaitPhase;
using g1_bringup::gravityOrientation;
using g1_bringup::kNumPolicyJoints;
using g1_bringup::kNumPolicyObs;
using g1_bringup::WalkPolicyConfig;

constexpr double kEps = 1e-9;

std::array<double, kNumPolicyJoints> filled(double v)
{
    std::array<double, kNumPolicyJoints> a{};
    a.fill(v);
    return a;
}

TEST(WalkPolicy, GravityOrientationIsMinusZForAnUprightBase)
{
    // Identity quaternion: gravity points straight down the base's -z.
    const auto g = gravityOrientation({ 1.0, 0.0, 0.0, 0.0 });
    EXPECT_NEAR(g[0], 0.0, kEps);
    EXPECT_NEAR(g[1], 0.0, kEps);
    EXPECT_NEAR(g[2], -1.0, kEps);
}

TEST(WalkPolicy, GravityOrientationTiltsWithPitch)
{
    // 90 deg pitch: gravity rotates fully into +x.
    const double s = std::sin(M_PI / 4.0);
    const double c = std::cos(M_PI / 4.0);
    const auto   g = gravityOrientation({ c, 0.0, s, 0.0 });
    EXPECT_NEAR(g[0], 1.0, 1e-9);
    EXPECT_NEAR(g[2], 0.0, 1e-9);
}

TEST(WalkPolicy, ObservationLayoutMatchesTheVendorOrdering)
{
    WalkPolicyConfig config{};
    const auto       obs = assembleObservation(
        config,
        { 0.4, -0.8, 1.2 },
        { 1.0, 0.0, 0.0, 0.0 },
        filled(0.5),
        filled(2.0),
        filled(0.25),
        { 0.3, -0.1, 0.5 },
        0.0);

    ASSERT_EQ(obs.size(), kNumPolicyObs);
    ASSERT_EQ(kNumPolicyObs, 47U);

    // [0:3] angular velocity * ang_vel_scale
    EXPECT_NEAR(obs[0], 0.4 * 0.25, kEps);
    EXPECT_NEAR(obs[2], 1.2 * 0.25, kEps);
    // [3:6] gravity orientation, upright
    EXPECT_NEAR(obs[5], -1.0, kEps);
    // [6:9] command * cmd_scale
    EXPECT_NEAR(obs[6], 0.3 * 2.0, kEps);
    EXPECT_NEAR(obs[7], -0.1 * 2.0, kEps);
    EXPECT_NEAR(obs[8], 0.5 * 0.25, kEps);
    // [9:21] (q - default) * dof_pos_scale
    EXPECT_NEAR(obs[9], 0.5 - config.default_angles[0], kEps);
    // [21:33] dq * dof_vel_scale
    EXPECT_NEAR(obs[21], 2.0 * 0.05, kEps);
    // [33:45] previous action, passed through unscaled
    EXPECT_NEAR(obs[33], 0.25, kEps);
    // [45:47] gait phase
    EXPECT_NEAR(obs[45], 0.0, kEps);
    EXPECT_NEAR(obs[46], 1.0, kEps);
}

TEST(WalkPolicy, CommandIsClampedToMaxCmd)
{
    WalkPolicyConfig config{};
    const auto       obs = assembleObservation(
        config,
        { 0.0, 0.0, 0.0 },
        { 1.0, 0.0, 0.0, 0.0 },
        filled(0.0),
        filled(0.0),
        filled(0.0),
        { 99.0, -99.0, 99.0 },
        0.0);

    // Clamped to max_cmd first, then scaled -- an unclamped command must never
    // reach the policy as an out-of-distribution observation.
    EXPECT_NEAR(obs[6], config.max_cmd[0] * config.cmd_scale[0], kEps);
    EXPECT_NEAR(obs[7], -config.max_cmd[1] * config.cmd_scale[1], kEps);
    EXPECT_NEAR(obs[8], config.max_cmd[2] * config.cmd_scale[2], kEps);
}

TEST(WalkPolicy, GaitPhaseWrapsOverThePeriod)
{
    WalkPolicyConfig config{};
    EXPECT_NEAR(gaitPhase(config, 0.0), 0.0, kEps);
    EXPECT_NEAR(gaitPhase(config, 0.4), 0.5, kEps);
    EXPECT_NEAR(gaitPhase(config, 0.8), 0.0, kEps);
    EXPECT_NEAR(gaitPhase(config, 1.2), 0.5, kEps);
}

TEST(WalkPolicy, ActionDecodesAsDefaultPlusScaledAction)
{
    WalkPolicyConfig config{};
    const auto       targets = actionToJointTargets(config, filled(2.0));
    for (std::size_t i = 0; i < kNumPolicyJoints; ++i)
    {
        EXPECT_NEAR(targets[i], config.default_angles[i] + (2.0 * config.action_scale), kEps);
    }
}

TEST(WalkPolicy, ZeroActionHoldsTheDefaultPosture)
{
    WalkPolicyConfig config{};
    const auto       targets = actionToJointTargets(config, filled(0.0));
    for (std::size_t i = 0; i < kNumPolicyJoints; ++i)
    {
        EXPECT_NEAR(targets[i], config.default_angles[i], kEps);
    }
}
}  // namespace
