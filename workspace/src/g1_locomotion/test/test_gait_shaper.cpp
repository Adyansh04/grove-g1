/**
 * @file test_gait_shaper.cpp
 * @brief Unit tests for GaitShaper: the deadband, primitive exclusivity, the signed-forward
 * asymmetry that blocks reverse, and the never-amplifies invariant.
 */
#include <gmock/gmock.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "g1_locomotion/gait_shaper.hpp"

namespace g1_locomotion
{
namespace
{

// config/g1_gait_shaper.yaml
constexpr double kFwdEngage = 0.45;
constexpr double kRevEngage = 0.55;
constexpr double kYawEngage = 1.20;
constexpr double kYawClamp  = 1.57;
constexpr double kLatEngage = 0.50;
constexpr double kLatClamp  = 0.50;

GaitShaper makeShaper()
{
    return GaitShaper(
        GaitShaper::Config{ kFwdEngage, kRevEngage, kYawEngage, kYawClamp, kLatEngage, kLatClamp });
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

TEST(GaitShaper, StrafeSurvivesOnlyWhenItIsTheWholeCommand)
{
    const auto shaper = makeShaper();
    // Lateral is a primitive now, but the LAST one tested, so anything carrying forward or yaw
    // as well still collapses to that. The measured combined-command response is why: a
    // commanded (0.50, 0, 0.50) produced (0.337, 0.299, 0.390).
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.6, 0.5, 0.0 }).vy, 0.0) << "forward wins";
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 0.5, 1.5 }).vy, 0.0) << "yaw wins";

    const auto out = shaper.shape({ 0.0, kLatEngage, 0.0 });
    EXPECT_DOUBLE_EQ(out.vy, kLatEngage) << "lateral alone IS a primitive";
    EXPECT_DOUBLE_EQ(out.vx, 0.0);
    EXPECT_DOUBLE_EQ(out.vyaw, 0.0);
}

TEST(GaitShaper, LateralBelowTheStepPointIsAStop)
{
    const auto shaper = makeShaper();
    // Measured: no lateral motion at or below 0.30 m/s, steps from 0.50. Anything under the
    // threshold has to become a stop rather than a command the gait silently ignores.
    for (double vy : { 0.0, 0.1, 0.3, 0.49 })
    {
        EXPECT_TRUE(isStop(shaper.shape({ 0.0, vy, 0.0 }))) << "vy " << vy;
        EXPECT_TRUE(isStop(shaper.shape({ 0.0, -vy, 0.0 }))) << "vy " << -vy;
    }
}

TEST(GaitShaper, StrafesEitherWayAndClamps)
{
    const auto shaper = makeShaper();
    // Unlike forward, lateral compares on MAGNITUDE. The signed forward comparison exists to
    // block a reverse lurch; nothing suggests the two strafe directions differ.
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 0.5, 0.0 }).vy, 0.5);
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, -0.5, 0.0 }).vy, -0.5);
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, 2.0, 0.0 }).vy, kLatClamp) << "clamped, not passed";
    EXPECT_DOUBLE_EQ(shaper.shape({ 0.0, -2.0, 0.0 }).vy, -kLatClamp);
}

TEST(GaitShaper, ReverseEngagesOnlyPastItsOwnHigherThreshold)
{
    const auto shaper = makeShaper();
    // Reverse exists on this policy but only well past where a planner asks for it: -0.60
    // measures -0.247 m/s and -0.40 measures exactly zero. A deliberate -0.60 gets through while
    // Nav2's 0.025..0.15 m/s backup speeds stay blocked -- the same backstop against a
    // misconfigured recovery behaviour lurching backwards.
    EXPECT_DOUBLE_EQ(shaper.shape({ -0.60, 0.0, 0.0 }).vx, -0.60);
    for (double vx : { -0.02, -0.15, -0.30, -0.40, -0.54 })
    {
        EXPECT_TRUE(isStop(shaper.shape({ vx, 0.0, 0.0 }))) << "vx " << vx;
    }
}

TEST(GaitShaper, ReverseIsNeverAmplifiedPastWhatWasAsked)
{
    // Reverse is allowed now, so the invariant worth pinning is the same one forward has: what
    // comes out is what went in, or a stop. Never something larger.
    const auto shaper = makeShaper();
    for (double vx : { -0.6, -1.0, -5.0 })
    {
        const auto out = shaper.shape({ vx, 0.0, 0.0 });
        EXPECT_LE(std::abs(out.vx), std::abs(vx) + 1e-12) << "vx " << vx;
        EXPECT_LE(out.vx, 0.0) << "a reverse command must not come back forward";
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
    for (int ix = 0; - 2.0 + ix * 0.05 <= 2.0; ++ix)
    {
        const double vx = -2.0 + ix * 0.05;
        for (int iyaw = 0; - 2.0 + iyaw * 0.05 <= 2.0; ++iyaw)
        {
            const double vyaw = -2.0 + iyaw * 0.05;
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
    for (int i = 0; - 2.0 + i * 0.05 <= 2.0; ++i)
    {
        const double vyaw = -2.0 + i * 0.05;
        const auto   out  = shaper.shape({ 0.0, 0.0, vyaw });
        if (out.vyaw != 0.0)
        {
            EXPECT_GT(out.vyaw * vyaw, 0.0) << "vyaw " << vyaw;
        }
    }
}

// The constructor's own contract. These matter because shape() relies on it: std::clamp is
// undefined when its bounds are inverted, which a negative yaw_clamp produces. Validating in
// the node that reads the YAML was not enough -- it left every other caller, this test file
// included, free to hand the class a config its body cannot handle.

TEST(GaitShaperConfig, RejectsANegativeYawClamp)
{
    EXPECT_THROW(
        GaitShaper(GaitShaper::Config{ kFwdEngage, kRevEngage, kYawEngage, -1.0 }),
        std::invalid_argument);
}

TEST(GaitShaperConfig, RejectsANegativeForwardEngage)
{
    // A negative fwd_engage makes `in.vx >= fwd_engage` true for reverse commands, which is the
    // one thing the signed comparison exists to prevent.
    EXPECT_THROW(
        GaitShaper(GaitShaper::Config{ -0.1, kRevEngage, kYawEngage, kYawClamp }),
        std::invalid_argument);
}

TEST(GaitShaperConfig, RejectsANonPositiveYawEngage)
{
    EXPECT_THROW(
        GaitShaper(GaitShaper::Config{ kFwdEngage, kRevEngage, 0.0, kYawClamp }),
        std::invalid_argument);
    EXPECT_THROW(
        GaitShaper(GaitShaper::Config{ kFwdEngage, kRevEngage, -1.0, kYawClamp }),
        std::invalid_argument);
}

TEST(GaitShaperConfig, RejectsAYawClampBelowYawEngage)
{
    // Structurally invalid, not merely badly tuned: every turn that clears yaw_engage is then
    // clamped back under it, so the turn-in-place motion becomes unreachable while the shaper
    // still reports it as a turn.
    EXPECT_THROW(
        GaitShaper(GaitShaper::Config{ kFwdEngage, kRevEngage, 1.20, 0.5 }),
        std::invalid_argument);
    // Same trap on the lateral axis: a clamp under the engage threshold accepts a strafe and
    // then clamps it back below the value that accepted it, so the robot never steps sideways.
    EXPECT_THROW(
        GaitShaper(GaitShaper::Config{ kFwdEngage, kRevEngage, kYawEngage, kYawClamp, 0.50, 0.30 }),
        std::invalid_argument);
    EXPECT_THROW(
        GaitShaper(
            GaitShaper::Config{ kFwdEngage, kRevEngage, kYawEngage, kYawClamp, 0.0, kLatClamp }),
        std::invalid_argument);
}

TEST(GaitShaperConfig, AcceptsTheShippedConfigAndTheBoundaryCase)
{
    EXPECT_NO_THROW(GaitShaper(GaitShaper::Config{ kFwdEngage, kRevEngage, kYawEngage, kYawClamp }));
    // yaw_clamp == yaw_engage is the tightest legal config, and zero forward engage is legal:
    // it means every non-negative vx passes through, which is a deadband of nothing, not an
    // inverted bound.
    EXPECT_NO_THROW(GaitShaper(GaitShaper::Config{ 0.0, kRevEngage, kYawEngage, kYawEngage }));
}

}  // namespace
}  // namespace g1_locomotion
