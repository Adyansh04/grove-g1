/**
 * @file test_arm_ramp_engine.cpp
 * @brief Unit tests for ArmRampEngine's ramp/slew/dt-hardening behavior and
 *        its helper functions.
 */

#include <gmock/gmock.h>

#include <array>
#include <cmath>

#include "g1_hardware_interface/arm_ramp_engine.hpp"

namespace g1_hardware_interface
{
namespace
{

constexpr double kBlendRampUpS       = 2.0;
constexpr double kBlendRampDownS     = 2.0;
constexpr double kEmergencyRampDownS = 0.5;
constexpr double kMaxJointVelocity   = 1.0;
/**
 * @brief 100 Hz, matches command_publish_rate's default.
 */
constexpr double kDt = 0.01;
/**
 * @brief Same rate -- see RampConfig::nominal_period_s.
 */
constexpr double kNominalPeriodS = kDt;
constexpr double kEpsilon        = 1e-9;

RampConfig makeConfig()
{
    return RampConfig{ kBlendRampUpS,
                       kBlendRampDownS,
                       kEmergencyRampDownS,
                       kMaxJointVelocity,
                       kNominalPeriodS };
}

std::array<double, kNumArmJoints> zeroPositions()
{
    std::array<double, kNumArmJoints> positions{};
    positions.fill(0.0);
    return positions;
}

// -------------------------------------------------------------------------
// seedFromMeasured
// -------------------------------------------------------------------------

TEST(ArmRampEngine, SeedFromMeasuredSetsPublishedPositionsAndZeroesWeight)
{
    ArmRampEngine                     engine(makeConfig());
    std::array<double, kNumArmJoints> measured{};
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        measured[i] = static_cast<double>(i) * 0.1 - 0.5;
    }

    engine.seedFromMeasured(measured);

    EXPECT_DOUBLE_EQ(engine.weight(), 0.0);
    EXPECT_EQ(engine.publishedPositions(), measured);
}

// -------------------------------------------------------------------------
// Weight monotonicity and slope bounds
// -------------------------------------------------------------------------

TEST(ArmRampEngine, WeightRampsUpMonotonicallyAndReachesOneInExpectedTicks)
{
    ArmRampEngine engine(makeConfig());
    engine.seedFromMeasured(zeroPositions());
    const auto commanded = zeroPositions();

    /*
     * Per-tick slope bound (not just monotonicity + a completion deadline):
     * catches a regression that ramps up faster than blend_ramp_up_s
     * allows even if it still happens to finish inside the tick budget
     * below.
     */
    const double max_slope      = kDt / kBlendRampUpS;
    const int    expected_ticks = static_cast<int>(kBlendRampUpS / kDt);
    int          ticks_taken    = 0;
    double       previous       = engine.weight();
    while (engine.weight() < 1.0 && ticks_taken < expected_ticks + 5)
    {
        const double weight = engine.step(BlendMode::kActive, commanded, kDt);
        EXPECT_GE(weight, previous - kEpsilon) << "weight decreased on tick " << ticks_taken;
        EXPECT_LE(weight - previous, max_slope + kEpsilon)
            << "weight ramped up faster than blend_ramp_up_s allows on tick " << ticks_taken;
        previous = weight;
        ++ticks_taken;
    }

    EXPECT_NEAR(engine.weight(), 1.0, 1e-6);
    /*
     * Mirrors the emergency-ramp test's duration pin: a snap-to-1.0 would
     * finish in a handful of ticks and trip this, not just the slope check
     * above.
     */
    EXPECT_NEAR(ticks_taken, expected_ticks, 1);
}

TEST(ArmRampEngine, WeightRampsDownMonotonicallyAndReachesZeroInExpectedTicks)
{
    ArmRampEngine engine(makeConfig());
    engine.seedFromMeasured(zeroPositions());
    const auto commanded = zeroPositions();

    // Get to weight == 1 first.
    for (int i = 0; i < static_cast<int>(kBlendRampUpS / kDt) + 1; ++i)
    {
        engine.step(BlendMode::kActive, commanded, kDt);
    }
    ASSERT_NEAR(engine.weight(), 1.0, 1e-6);

    const double max_slope      = kDt / kBlendRampDownS;
    const int    expected_ticks = static_cast<int>(kBlendRampDownS / kDt);
    int          ticks_taken    = 0;
    double       previous       = engine.weight();
    while (engine.weight() > 0.0 && ticks_taken < expected_ticks + 5)
    {
        const double weight = engine.step(BlendMode::kRampDown, commanded, kDt);
        EXPECT_LE(weight, previous + kEpsilon) << "weight increased on tick " << ticks_taken;
        EXPECT_LE(previous - weight, max_slope + kEpsilon)
            << "weight ramped down faster than blend_ramp_down_s allows on tick " << ticks_taken;
        previous = weight;
        ++ticks_taken;
    }

    EXPECT_NEAR(engine.weight(), 0.0, 1e-6);
    EXPECT_NEAR(ticks_taken, expected_ticks, 1);
}

TEST(ArmRampEngine, RampDownTriggeredMidRampUpStaysMonotonicDownwardToZero)
{
    ArmRampEngine engine(makeConfig());
    engine.seedFromMeasured(zeroPositions());
    const auto commanded = zeroPositions();

    // Ramp up partway (not to completion).
    for (int i = 0; i < static_cast<int>(kBlendRampUpS / kDt / 2); ++i)
    {
        engine.step(BlendMode::kActive, commanded, kDt);
    }
    const double weight_at_switch = engine.weight();
    ASSERT_GT(weight_at_switch, 0.0);
    ASSERT_LT(weight_at_switch, 1.0);

    /*
     * Switch to ramp-down mid-flight: must only ever decrease from here,
     * never jump back up or oscillate, regardless of having been ramping up
     * a moment before.
     */
    double previous = weight_at_switch;
    for (int i = 0; i < static_cast<int>(kBlendRampDownS / kDt) + 1; ++i)
    {
        const double weight = engine.step(BlendMode::kRampDown, commanded, kDt);
        EXPECT_LE(weight, previous + kEpsilon);
        previous = weight;
    }

    EXPECT_NEAR(engine.weight(), 0.0, 1e-6);
}

TEST(ArmRampEngine, EmergencyRampDurationIsHonoredAndFasterThanNormalRampDown)
{
    ArmRampEngine engine(makeConfig());
    engine.seedFromMeasured(zeroPositions());
    const auto commanded = zeroPositions();

    for (int i = 0; i < static_cast<int>(kBlendRampUpS / kDt) + 1; ++i)
    {
        engine.step(BlendMode::kActive, commanded, kDt);
    }
    ASSERT_NEAR(engine.weight(), 1.0, 1e-6);

    const int expected_ticks = static_cast<int>(kEmergencyRampDownS / kDt);
    int       ticks_taken    = 0;
    double    previous       = engine.weight();
    while (engine.weight() > 0.0 && ticks_taken < expected_ticks + 5)
    {
        const double weight = engine.step(BlendMode::kEmergencyRampDown, commanded, kDt);
        EXPECT_LE(weight, previous + kEpsilon);
        previous = weight;
        ++ticks_taken;
    }

    /*
     * Emergency ramp (0.5 s) is four times faster than the normal ramp-down
     * (2.0 s) with this config -- confirm it actually finishes in roughly
     * expected_ticks, not blend_ramp_down_s's ticks.
     */
    EXPECT_NEAR(ticks_taken, expected_ticks, 1);
    EXPECT_NEAR(engine.weight(), 0.0, 1e-6);
}

// -------------------------------------------------------------------------
// Slew clamp
// -------------------------------------------------------------------------

TEST(ArmRampEngine, SlewClampIsExactAtBoundaryWhenTargetIsFarAway)
{
    ArmRampEngine engine(makeConfig());
    engine.seedFromMeasured(zeroPositions());

    std::array<double, kNumArmJoints> far_target{};
    far_target.fill(10.0);  // far outside a single tick's reach

    engine.step(BlendMode::kActive, far_target, kDt);

    const double expected_delta = kMaxJointVelocity * kDt;
    for (const double position : engine.publishedPositions())
    {
        EXPECT_NEAR(position, expected_delta, 1e-12);
    }
}

TEST(ArmRampEngine, SlewClampConvergesExactlyWhenTargetIsWithinOneTicksReach)
{
    ArmRampEngine engine(makeConfig());
    engine.seedFromMeasured(zeroPositions());

    const double                      small_step = kMaxJointVelocity * kDt * 0.5;
    std::array<double, kNumArmJoints> near_target{};
    near_target.fill(small_step);

    engine.step(BlendMode::kActive, near_target, kDt);

    for (const double position : engine.publishedPositions())
    {
        EXPECT_NEAR(position, small_step, 1e-12);
    }
}

TEST(ArmRampEngine, SlewClampAppliesPerJointIndependently)
{
    ArmRampEngine engine(makeConfig());
    engine.seedFromMeasured(zeroPositions());

    std::array<double, kNumArmJoints> mixed_target{};
    mixed_target[0] = 10.0;                           // far -- clamped
    mixed_target[1] = kMaxJointVelocity * kDt * 0.3;  // near -- reached exactly

    engine.step(BlendMode::kActive, mixed_target, kDt);

    EXPECT_NEAR(engine.publishedPositions()[0], kMaxJointVelocity * kDt, 1e-12);
    EXPECT_NEAR(engine.publishedPositions()[1], mixed_target[1], 1e-12);
}

// -------------------------------------------------------------------------
// dt hardening -- degenerate and pathologically large dt_s
// -------------------------------------------------------------------------

TEST(ArmRampEngine, NonPositiveDtHoldsWeightAndPositionsInstead)
{
    ArmRampEngine                     engine(makeConfig());
    std::array<double, kNumArmJoints> seed{};
    seed.fill(0.3);
    engine.seedFromMeasured(seed);
    engine.step(BlendMode::kActive, seed, kDt);  // move weight off 0.0 first
    const double weight_before    = engine.weight();
    const auto   positions_before = engine.publishedPositions();

    std::array<double, kNumArmJoints> far_target{};
    far_target.fill(5.0);

    /*
     * Zero dt: no snap-to-target (the pre-hardening fallback), no change at
     * all.
     */
    EXPECT_DOUBLE_EQ(engine.step(BlendMode::kActive, far_target, 0.0), weight_before);
    EXPECT_EQ(engine.publishedPositions(), positions_before);

    /*
     * Negative dt: same hold, and not UB (a naive clamp(-max_step, max_step)
     * with a negative max_step would invert the bounds).
     */
    EXPECT_DOUBLE_EQ(engine.step(BlendMode::kActive, far_target, -0.01), weight_before);
    EXPECT_EQ(engine.publishedPositions(), positions_before);
}

TEST(ArmRampEngine, LargeDtClampsWeightRampToNominalPeriodMultiple)
{
    ArmRampEngine engine(makeConfig());
    engine.seedFromMeasured(zeroPositions());

    /*
     * Unclamped, this dt's max_step (huge_dt / blend_ramp_up_s) would exceed
     * 1.0 and snap the weight straight to its target in one tick.
     */
    constexpr double huge_dt = 10.0;
    const double     weight  = engine.step(BlendMode::kActive, zeroPositions(), huge_dt);

    const double clamped_dt = kMaxDtNominalPeriodMultiple * kNominalPeriodS;
    EXPECT_NEAR(weight, clamped_dt / kBlendRampUpS, 1e-9);
    EXPECT_LT(engine.weight(), 1.0);
}

TEST(ArmRampEngine, LargeDtClampsPositionSlewToo)
{
    ArmRampEngine engine(makeConfig());
    engine.seedFromMeasured(zeroPositions());

    std::array<double, kNumArmJoints> far_target{};
    far_target.fill(100.0);

    engine.step(BlendMode::kActive, far_target, /*dt_s=*/10.0);

    const double clamped_dt     = kMaxDtNominalPeriodMultiple * kNominalPeriodS;
    const double expected_delta = kMaxJointVelocity * clamped_dt;
    for (const double position : engine.publishedPositions())
    {
        EXPECT_NEAR(position, expected_delta, 1e-9);
    }
}

// -------------------------------------------------------------------------
// validateMotorIndexMap -- index-map correctness
// -------------------------------------------------------------------------

TEST(ValidateMotorIndexMap, AcceptsTheRealArmIndexMap)
{
    std::array<int, kNumArmJoints> indices{};
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        indices[i] = 15 + static_cast<int>(i);
    }
    EXPECT_TRUE(validateMotorIndexMap(indices).empty());
}

TEST(ValidateMotorIndexMap, RejectsOutOfRangeIndexBelowArmRange)
{
    std::array<int, kNumArmJoints> indices{};
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        indices[i] = 15 + static_cast<int>(i);
    }
    indices[0]              = 14;  // waist, not an arm index
    const std::string error = validateMotorIndexMap(indices);
    EXPECT_THAT(error, ::testing::HasSubstr("14"));
}

TEST(ValidateMotorIndexMap, RejectsOutOfRangeIndexAboveArmRange)
{
    std::array<int, kNumArmJoints> indices{};
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        indices[i] = 15 + static_cast<int>(i);
    }
    indices[0]              = 29;  // the weight slot, not an arm index
    const std::string error = validateMotorIndexMap(indices);
    EXPECT_THAT(error, ::testing::HasSubstr("29"));
}

TEST(ValidateMotorIndexMap, RejectsDuplicateIndices)
{
    std::array<int, kNumArmJoints> indices{};
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        indices[i] = 15 + static_cast<int>(i);
    }
    indices[1]              = indices[0];
    const std::string error = validateMotorIndexMap(indices);
    EXPECT_THAT(error, ::testing::HasSubstr("duplicate"));
}

// -------------------------------------------------------------------------
// resolveEffectiveMode -- staleness escalation
// -------------------------------------------------------------------------

TEST(ResolveEffectiveMode, FreshFeedbackLeavesActiveAlone)
{
    EXPECT_EQ(resolveEffectiveMode(BlendMode::kActive, false), BlendMode::kActive);
}

TEST(ResolveEffectiveMode, StaleFeedbackEscalatesActiveToEmergencyRampDown)
{
    EXPECT_EQ(resolveEffectiveMode(BlendMode::kActive, true), BlendMode::kEmergencyRampDown);
}

TEST(ResolveEffectiveMode, StalenessTriggersEmergencyExactlyOnceAndNeverOscillatesOrDeescalates)
{
    /*
     * Once the caller's shared mode has been escalated (no longer kActive),
     * resolveEffectiveMode never re-escalates or reverts it, regardless of
     * whether the feedback is still stale or has recovered -- the only way
     * back to kActive is a fresh on_activate, outside this function entirely.
     */
    EXPECT_EQ(
        resolveEffectiveMode(BlendMode::kEmergencyRampDown, true),
        BlendMode::kEmergencyRampDown);
    EXPECT_EQ(
        resolveEffectiveMode(BlendMode::kEmergencyRampDown, false),
        BlendMode::kEmergencyRampDown);
    EXPECT_EQ(resolveEffectiveMode(BlendMode::kRampDown, true), BlendMode::kRampDown);
    EXPECT_EQ(resolveEffectiveMode(BlendMode::kRampDown, false), BlendMode::kRampDown);
}

}  // namespace
}  // namespace g1_hardware_interface
