/**
 * @file test_safety_blend.cpp
 * @brief The blend and slew arithmetic, tested without a controller_manager harness.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <limits>

#include "g1_controllers/g1_safety_controller.hpp"

namespace
{

using g1_controllers::blendAndSlew;

constexpr double kDt = 0.005;  // 200 Hz

TEST(BlendAndSlew, ZeroBlendHoldsTheActivationPose)
{
    // The policy is commanding something far away, but at ratio 0 none of it gets through.
    EXPECT_DOUBLE_EQ(blendAndSlew(0.3, 1.5, 0.0, 0.3, -1.0, kDt), 0.3);
}

TEST(BlendAndSlew, FullBlendFollowsTheCommandWhenUnclamped)
{
    EXPECT_DOUBLE_EQ(blendAndSlew(0.3, 1.5, 1.0, 0.3, -1.0, kDt), 1.5);
}

TEST(BlendAndSlew, HalfBlendSitsHalfway)
{
    EXPECT_DOUBLE_EQ(blendAndSlew(0.0, 1.0, 0.5, 0.0, -1.0, kDt), 0.5);
}

TEST(BlendAndSlew, NonPositiveMaxVelocityLeavesTheJointUnclamped)
{
    // The whole lower body is configured like this on purpose.
    EXPECT_DOUBLE_EQ(blendAndSlew(0.0, 10.0, 1.0, 0.0, 0.0, kDt), 10.0);
    EXPECT_DOUBLE_EQ(blendAndSlew(0.0, 10.0, 1.0, 0.0, -5.0, kDt), 10.0);
}

TEST(BlendAndSlew, VelocityClampBoundsOneTicksTravel)
{
    // 2 rad/s over a 5 ms tick is 0.01 rad, whatever the command asks for.
    EXPECT_DOUBLE_EQ(blendAndSlew(0.0, 10.0, 1.0, 0.0, 2.0, kDt), 0.01);
    EXPECT_DOUBLE_EQ(blendAndSlew(0.0, -10.0, 1.0, 0.0, 2.0, kDt), -0.01);
}

TEST(BlendAndSlew, ClampSurvivesABlendRatioStep)
{
    // The property that makes `ros2 param set blend_ratio 1.0` safe mid-run: even a 0-to-1 jump
    // cannot move the joint faster than its ceiling.
    const double out = blendAndSlew(0.0, 2.0, 1.0, 0.0, 1.0, kDt);
    EXPECT_DOUBLE_EQ(out, 1.0 * kDt);
}

TEST(BlendAndSlew, RepeatedTicksConvergeWithoutOvershoot)
{
    double integrated = 0.0;
    for (int i = 0; i < 1000; ++i)
    {
        integrated = blendAndSlew(0.0, 0.5, 1.0, integrated, 1.0, kDt);
    }
    EXPECT_NEAR(integrated, 0.5, 1e-9);

    // Once there it stays there rather than oscillating around the target.
    EXPECT_DOUBLE_EQ(blendAndSlew(0.0, 0.5, 1.0, integrated, 1.0, kDt), 0.5);
}

TEST(BlendAndSlew, ApproachesFromAboveToo)
{
    double integrated = 1.0;
    for (int i = 0; i < 1000; ++i)
    {
        integrated = blendAndSlew(1.0, 0.2, 1.0, integrated, 1.0, kDt);
    }
    EXPECT_NEAR(integrated, 0.2, 1e-9);
}

TEST(BlendAndSlew, RampingTheBlendMovesTheTargetGradually)
{
    // What activation actually looks like: ratio climbing at 1.0/s while the joint tracks it.
    double integrated = 0.3;
    double ratio      = 0.0;
    for (int i = 0; i < 200; ++i)  // 1 s at 200 Hz
    {
        ratio      = std::min(1.0, ratio + (1.0 * kDt));
        integrated = blendAndSlew(0.3, 0.9, ratio, integrated, -1.0, kDt);
    }
    EXPECT_NEAR(ratio, 1.0, 1e-9);
    EXPECT_NEAR(integrated, 0.9, 1e-9);
}

TEST(BlendAndSlew, OutputIsFiniteForAFiniteCommand)
{
    EXPECT_TRUE(std::isfinite(blendAndSlew(0.0, 1e6, 1.0, 0.0, 1.0, kDt)));
    EXPECT_TRUE(std::isfinite(blendAndSlew(0.0, 1e6, 1.0, 0.0, -1.0, kDt)));
}

TEST(BlendAndSlew, AnInfiniteCommandPoisonsTheIntegratorEvenAtBlendRatioZero)
{
    // Documents why the caller filters non-finite references rather than relying on the blend.
    // At ratio 0 the blend is meant to ignore the command entirely, but `0.0 * inf` is NaN, so
    // even "ignore the policy" does not survive one infinite reference.
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(std::isfinite(blendAndSlew(0.5, inf, 0.0, 0.5, -1.0, kDt)));
    EXPECT_FALSE(std::isfinite(blendAndSlew(0.5, inf, 1.0, 0.5, -1.0, kDt)));

    // And it latches: once integrated is non-finite, a well-behaved command cannot recover it.
    const double poisoned = blendAndSlew(0.5, inf, 1.0, 0.5, -1.0, kDt);
    EXPECT_FALSE(std::isfinite(blendAndSlew(0.5, 0.5, 1.0, poisoned, -1.0, kDt)));
}

}  // namespace
