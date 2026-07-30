/**
 * @file test_blend_math.cpp
 * @brief Unit tests for blend() and the stepEffectiveWeight() staleness ramp/resume policy.
 */
#include <gmock/gmock.h>

#include "g1_bringup/blend_math.hpp"

namespace g1_bringup
{
namespace
{

constexpr double kTimeoutRampDownS = 1.0;
/**
 * @brief 100 Hz-ish tick, matches the sim's own /arm_sdk rate.
 */
constexpr double kDt      = 0.01;
constexpr double kEpsilon = 1e-9;

// -------------------------------------------------------------------------
// blend()
// -------------------------------------------------------------------------

TEST(Blend, ZeroWeightReturnsHoldValue) { EXPECT_DOUBLE_EQ(blend(1.5, -3.0, 0.0), 1.5); }

TEST(Blend, FullWeightReturnsCommandedValue) { EXPECT_DOUBLE_EQ(blend(1.5, -3.0, 1.0), -3.0); }

TEST(Blend, MidWeightIsLinearInterpolation) { EXPECT_DOUBLE_EQ(blend(0.0, 10.0, 0.25), 2.5); }

TEST(Blend, WeightIsClampedToUnitRange)
{
    EXPECT_DOUBLE_EQ(blend(0.0, 10.0, -5.0), 0.0);
    EXPECT_DOUBLE_EQ(blend(0.0, 10.0, 5.0), 10.0);
}

// -------------------------------------------------------------------------
// stepEffectiveWeight() -- the staleness decay/resume policy
// -------------------------------------------------------------------------

TEST(StepEffectiveWeight, TracksRawWeightWhenFresh)
{
    double weight = 0.0;
    for (int i = 0; i < static_cast<int>(1.0 / kDt) + 1; ++i)
    {
        weight = stepEffectiveWeight(weight, 1.0, /*arm_sdk_stale=*/false, kTimeoutRampDownS, kDt);
    }
    EXPECT_NEAR(weight, 1.0, 1e-6);
}

TEST(StepEffectiveWeight, DecaysMonotonicallyToZeroWhenStale)
{
    double weight   = 1.0;
    double previous = weight;
    for (int i = 0; i < static_cast<int>(kTimeoutRampDownS / kDt); ++i)
    {
        weight = stepEffectiveWeight(weight, 1.0, /*arm_sdk_stale=*/true, kTimeoutRampDownS, kDt);
        EXPECT_LE(weight, previous + kEpsilon) << "weight increased on tick " << i;
        previous = weight;
    }
    EXPECT_NEAR(weight, 0.0, 1e-6);
}

TEST(StepEffectiveWeight, DecayRateMatchesTimeoutRampDownS)
{
    // Reaches 0 in roughly timeout_ramp_down_s seconds, not some other rate.
    double    weight         = 1.0;
    int       ticks_taken    = 0;
    const int expected_ticks = static_cast<int>(kTimeoutRampDownS / kDt);
    while (weight > 0.0 && ticks_taken < expected_ticks + 5)
    {
        weight = stepEffectiveWeight(weight, 1.0, /*arm_sdk_stale=*/true, kTimeoutRampDownS, kDt);
        ++ticks_taken;
    }
    EXPECT_NEAR(ticks_taken, expected_ticks, 1);
}

TEST(StepEffectiveWeight, ResumeAfterStalenessContinuesFromCurrentValueWithoutSnapping)
{
    // Decay partway through a staleness episode.
    double weight = 1.0;
    for (int i = 0; i < static_cast<int>(kTimeoutRampDownS / kDt / 2); ++i)
    {
        weight = stepEffectiveWeight(weight, 1.0, /*arm_sdk_stale=*/true, kTimeoutRampDownS, kDt);
    }
    const double weight_at_resume = weight;
    ASSERT_GT(weight_at_resume, 0.0);
    ASSERT_LT(weight_at_resume, 1.0);

    /*
     * A fresh message with raw weight 1.0 arrives: the very next tick must
     * move toward 1.0 from weight_at_resume, never jump straight to it.
     */
    const double next =
        stepEffectiveWeight(weight, 1.0, /*arm_sdk_stale=*/false, kTimeoutRampDownS, kDt);
    EXPECT_GT(next, weight_at_resume);
    EXPECT_LT(next, 1.0);
    EXPECT_NEAR(next - weight_at_resume, kDt / kTimeoutRampDownS, 1e-9);
}

TEST(StepEffectiveWeight, NeverReceivedTreatedAsStaleStaysAtZero)
{
    /*
     * The bridge feeds raw_weight=0.0 with arm_sdk_stale=true before the
     * first /arm_sdk message ever arrives -- confirm that's a stable
     * fixed point, not just a coincidental zero.
     */
    double weight = 0.0;
    for (int i = 0; i < 100; ++i)
    {
        weight = stepEffectiveWeight(weight, 0.0, /*arm_sdk_stale=*/true, kTimeoutRampDownS, kDt);
    }
    EXPECT_DOUBLE_EQ(weight, 0.0);
}

TEST(StepEffectiveWeight, ResultIsAlwaysClampedToUnitRange)
{
    const double weight = stepEffectiveWeight(
        /*previous_effective_weight=*/0.0,
        /*raw_weight=*/5.0,
        false,
        kTimeoutRampDownS,
        1000.0);
    EXPECT_LE(weight, 1.0);
    EXPECT_GE(weight, 0.0);
}

}  // namespace
}  // namespace g1_bringup
