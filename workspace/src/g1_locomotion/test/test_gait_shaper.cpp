/**
 * @file test_gait_shaper.cpp
 * @brief Unit tests for GaitShaper: the deadband, primitive exclusivity, the signed-forward
 * asymmetry that blocks reverse, and the never-amplifies invariant.
 */
#include <gmock/gmock.h>

#include <cmath>
#include <limits>

#include "g1_locomotion/gait_shaper.hpp"

namespace g1_locomotion
{
namespace
{

// config/g1_gait_shaper.yaml
constexpr double kFwdEngage = 0.45;
constexpr double kYawEngage = 1.20;
constexpr double kYawClamp  = 1.57;

GaitShaper makeShaper()
{
    return GaitShaper(GaitShaper::Config{ kFwdEngage, kYawEngage, kYawClamp });
}

::testing::AssertionResult isStop(const GaitShaper::Command& c)
{
    if (c.vx == 0.0 && c.vy == 0.0 && c.vyaw == 0.0)
    {
        return ::testing::AssertionSuccess();
    }
    return ::testing::AssertionFailure()
           << "expected a stop, got (" << c.vx << ", " << c.vy << ", " << c.vyaw << ")";
}

TEST(GaitShaper, SubThresholdBecomesAStop)
{
    const auto shaper = makeShaper();
    // The whole dead zone: the policy produces no motion for any of these, so passing them
    // through would leave the robot standing while the planner believes it is driving.
    for (double vx : { 0.0, 0.05, 0.2, 0.35, 0.44 })
    {
        EXPECT_TRUE(isStop(shaper.shape({ vx, 0.0, 0.0 }))) << "vx " << vx;
    }
    for (double vyaw : { 0.0, 0.1, 0.6, 1.0, 1.19 })
    {
        EXPECT_TRUE(isStop(shaper.shape({ 0.0, 0.0, vyaw }))) << "vyaw " << vyaw;
        EXPECT_TRUE(isStop(shaper.shape({ 0.0, 0.0, -vyaw }))) << "vyaw " << -vyaw;
    }
}

TEST(GaitShaper, EngagesInclusivelyAtTheThreshold)
{
    // Pinned deliberately. Nothing hinges on it -- the thresholds sit between the measured
    // "no motion at or below" and "steps from" values -- but the choice should be a decision
    // rather than an accident.
    const auto shaper = makeShaper();
    EXPECT_DOUBLE_EQ(shaper.shape({ kFwdEngage, 0.0, 0.0 }).vx, kFwdEngage);
    EXPECT_TRUE(isStop(shaper.shape({ std::nextafter(kFwdEngage, 0.0), 0.0, 0.0 })));
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 0.0, kYawEngage }).vyaw, kYawEngage);
    EXPECT_TRUE(isStop(shaper.shape({ 0.0, 0.0, std::nextafter(kYawEngage, 0.0) })));
}

TEST(GaitShaper, DrivesStraightAboveTheForwardThreshold)
{
    const auto shaper = makeShaper();
    const auto out    = shaper.shape({ 0.6, 0.0, 0.0 });
    EXPECT_DOUBLE_EQ(out.vx, 0.6) << "an achievable forward command passes through unchanged";
    EXPECT_DOUBLE_EQ(out.vy, 0.0);
    EXPECT_DOUBLE_EQ(out.vyaw, 0.0);
}

TEST(GaitShaper, TurnsInPlaceAndKeepsTheSign)
{
    const auto shaper = makeShaper();
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 0.0, 1.4 }).vyaw, 1.4);
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 0.0, -1.4 }).vyaw, -1.4) << "turning either way is proven";
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 0.0, 5.0 }).vyaw, kYawClamp) << "clamped, not passed";
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 0.0, -5.0 }).vyaw, -kYawClamp);
}

TEST(GaitShaper, YawWinsSoTheOutputIsNeverACombinedCommand)
{
    // The measured worst case: a commanded (0.50, 0, 0.50) produced (0.337, 0.299, 0.390) --
    // a third of a metre per second of lateral nobody asked for. The two primitives are never
    // mixed on the output.
    const auto shaper = makeShaper();
    for (double vx : { 0.5, 0.6, 1.0 })
    {
        for (double vyaw : { 1.2, 1.5, 2.0 })
        {
            const auto out = shaper.shape({ vx, 0.0, vyaw });
            EXPECT_DOUBLE_EQ(out.vx, 0.0) << "vx " << vx << " vyaw " << vyaw;
            EXPECT_GT(std::abs(out.vyaw), 0.0);
        }
    }
}

TEST(GaitShaper, AlwaysDropsLateral)
{
    const auto shaper = makeShaper();
    // Whichever branch is taken, and whatever was asked for.
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.6, 0.5, 0.0 }).vy, 0.0);
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 0.5, 1.5 }).vy, 0.0);
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 0.5, 0.0 }).vy, 0.0) << "lateral alone is not a primitive";
}

TEST(GaitShaper, NegativeForwardIsAlwaysAStopAtAnyMagnitude)
{
    // The backstop that makes reverse recovery behaviours harmless. Reverse measures -0.247 m/s
    // for a commanded -0.60 and exactly 0.000 for -0.40, so a planner's usual backup speeds sit
    // inside the dead zone -- but this holds even for a hand-edited tree asking for -5.
    const auto shaper = makeShaper();
    for (double vx : { -0.01, -0.1, -0.45, -0.6, -1.0, -5.0 })
    {
        EXPECT_TRUE(isStop(shaper.shape({ vx, 0.0, 0.0 }))) << "vx " << vx;
    }
}

TEST(GaitShaper, NonFiniteInputFallsThroughToAStop)
{
    const auto   shaper = makeShaper();
    const double nan    = std::numeric_limits<double>::quiet_NaN();
    const double inf    = std::numeric_limits<double>::infinity();

    EXPECT_TRUE(isStop(shaper.shape({ nan, 0.0, 0.0 }))) << "NaN fails both comparisons";
    EXPECT_TRUE(isStop(shaper.shape({ 0.0, 0.0, nan })));
    EXPECT_TRUE(isStop(shaper.shape({ nan, nan, nan })));
    EXPECT_TRUE(isStop(shaper.shape({ -inf, 0.0, 0.0 })));
    // An infinite yaw is still a turn, but a bounded one.
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 0.0, inf }).vyaw, kYawClamp);
    EXPECT_DOUBLE_EQ(shaper.shape({ inf, 0.0, 0.0 }).vx, inf)
        << "an infinite vx is the caller's bug, not ours to amplify";
}

TEST(GaitShaper, NeverAmplifiesAnyAxis)
{
    // THE safety property: every output is the input unchanged, clamped smaller, or zero.
    // Turning a small command into a large motion is exactly what this stack's control-mode
    // rules exist to prevent, so it is swept rather than spot-checked.
    const auto shaper = makeShaper();
    for (double vx = -2.0; vx <= 2.0; vx += 0.05)
    {
        for (double vyaw = -2.0; vyaw <= 2.0; vyaw += 0.05)
        {
            for (double vy : { -0.5, 0.0, 0.5 })
            {
                const GaitShaper::Command in{ vx, vy, vyaw };
                const GaitShaper::Command out = shaper.shape(in);
                EXPECT_LE(std::abs(out.vx), std::abs(in.vx) + 1e-12) << "vx " << vx;
                EXPECT_LE(std::abs(out.vy), std::abs(in.vy) + 1e-12) << "vy " << vy;
                EXPECT_LE(std::abs(out.vyaw), std::abs(in.vyaw) + 1e-12) << "vyaw " << vyaw;
            }
        }
    }
}

TEST(GaitShaper, NeverFlipsASign)
{
    // Weaker than the magnitude invariant but independent of it: reversing a command would
    // also be "not amplifying", and would be just as wrong.
    const auto shaper = makeShaper();
    for (double vyaw = -2.0; vyaw <= 2.0; vyaw += 0.05)
    {
        const auto out = shaper.shape({ 0.0, 0.0, vyaw });
        if (out.vyaw != 0.0)
        {
            EXPECT_GT(out.vyaw * vyaw, 0.0) << "vyaw " << vyaw;
        }
    }
}

}  // namespace
}  // namespace g1_locomotion
