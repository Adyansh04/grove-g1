/**
 * @file test_approach_planner.cpp
 * @brief The geometry that decides whether the base can be walked into arm's reach.
 *
 * Worth testing away from a simulator because a live run exercises exactly one trajectory,
 * while the branches that matter are the ones a good run never touches: the overshoot that
 * cannot be undone, the oblique angle that makes a 0.3 m step land inside a 0.11 m window, and
 * the deliberately asymmetric quantum estimate.
 */

#include <gmock/gmock.h>

#include <cmath>

#include "g1_locomotion/approach_planner.hpp"

namespace
{

using g1_locomotion::ApproachLimits;
using g1_locomotion::ApproachMove;
using g1_locomotion::limitsAreUsable;
using g1_locomotion::maxObservedAdvance;
using g1_locomotion::planApproach;

/// The shipped window, in polar form: the manipulation scene's proven grasp at (0.28, -0.20).
ApproachLimits defaults() { return ApproachLimits{}; }

TEST(ApproachPlanner, ObjectInsideTheWindowIsDone)
{
    const auto limits  = defaults();
    const auto command = planApproach(limits.target_range_m, limits.target_bearing_rad, limits);
    EXPECT_EQ(command.move, ApproachMove::kDone);
}

TEST(ApproachPlanner, RightRangeButWrongBearingTurnsWithoutStepping)
{
    const auto limits  = defaults();
    const auto command = planApproach(limits.target_range_m, 0.0, limits);
    ASSERT_EQ(command.move, ApproachMove::kTurn);
    // Stepping here would walk out of a window it is already inside.
    EXPECT_NEAR(command.turn_rad, -limits.target_bearing_rad, 1e-9);
}

TEST(ApproachPlanner, TooCloseIsTerminalBecauseTheGaitCannotReverse)
{
    const auto limits = defaults();
    EXPECT_EQ(
        planApproach(limits.min_range_m - 0.01, limits.target_bearing_rad, limits).move,
        ApproachMove::kOvershot);
    // Past the near edge of the window but not yet under the robot: still unrecoverable, since
    // there is no command that moves the base backwards.
    const double just_short = limits.target_range_m - limits.range_tolerance_m - 0.01;
    EXPECT_EQ(
        planApproach(just_short, limits.target_bearing_rad, limits).move,
        ApproachMove::kOvershot);
}

TEST(ApproachPlanner, FarAwayAndFacingItStepsStraightIn)
{
    const auto limits = defaults();
    const auto command =
        planApproach(limits.target_range_m + 2.0, limits.target_bearing_rad, limits);
    ASSERT_EQ(command.move, ApproachMove::kStep);
    EXPECT_DOUBLE_EQ(command.oblique_rad, 0.0) << "no reason to walk crooked from two metres out";
}

TEST(ApproachPlanner, TheObliqueAngleMakesTheAdvanceMatchWhatIsLeft)
{
    auto limits            = defaults();
    limits.pulse_advance_m = 0.40;
    // Aim the robot straight at the window's centre first, so the only turn asked for is the
    // oblique itself. 0.15 of a 0.40 pulse needs 68 degrees, inside the cap the next test
    // covers -- pick anything smaller and this measures the cap instead of the geometry.
    const double remaining = 0.15;
    const auto   command =
        planApproach(limits.target_range_m + remaining, limits.target_bearing_rad, limits);

    ASSERT_EQ(command.move, ApproachMove::kTurn) << "an oblique step has to be aimed first";
    EXPECT_NEAR(command.remaining_m, remaining, 1e-9);
    // The whole point: one pulse along this heading closes exactly what is left.
    EXPECT_NEAR(limits.pulse_advance_m * std::cos(command.oblique_rad), remaining, 1e-6);
}

TEST(ApproachPlanner, TheObliqueIsCappedSoTheRobotDoesNotWalkSideways)
{
    auto limits            = defaults();
    limits.pulse_advance_m = 0.60;
    limits.max_oblique_rad = 1.2;
    // 1 cm to close with a 60 cm step wants an oblique of 89 degrees, which is almost pure
    // lateral motion.
    const auto command =
        planApproach(limits.target_range_m + 0.01, limits.target_bearing_rad, limits);
    EXPECT_LE(command.oblique_rad, limits.max_oblique_rad + 1e-9);
}

TEST(ApproachPlanner, AQuantumBiggerThanTheGapStillProducesAFiniteAngle)
{
    auto limits            = defaults();
    limits.pulse_advance_m = 0.35;
    for (double remaining = 0.001; remaining < 0.35; remaining += 0.01)
    {
        const auto command =
            planApproach(limits.target_range_m + remaining, limits.target_bearing_rad, limits);
        EXPECT_TRUE(std::isfinite(command.oblique_rad)) << "remaining " << remaining;
        EXPECT_GE(command.oblique_rad, 0.0);
    }
}

TEST(ApproachPlanner, TheQuantumEstimateOnlyEverGrows)
{
    // Asymmetric on purpose: undershooting costs another pulse, overshooting is unrecoverable.
    EXPECT_DOUBLE_EQ(maxObservedAdvance(0.35, 0.60), 0.60);
    EXPECT_DOUBLE_EQ(maxObservedAdvance(0.60, 0.30), 0.60);
    // A pulse that produced nothing means the gait failed to start, not that it can step zero.
    EXPECT_DOUBLE_EQ(maxObservedAdvance(0.35, 0.0), 0.35);
    EXPECT_DOUBLE_EQ(maxObservedAdvance(0.35, -1.0), 0.35);
}

TEST(ApproachPlanner, UnusableLimitsAreRefusedRatherThanAimedAt)
{
    EXPECT_TRUE(limitsAreUsable(defaults()));

    auto no_room        = defaults();
    no_room.min_range_m = no_room.target_range_m;
    EXPECT_FALSE(limitsAreUsable(no_room)) << "the window would sit entirely inside the robot";

    auto sideways            = defaults();
    sideways.max_oblique_rad = M_PI_2;
    EXPECT_FALSE(limitsAreUsable(sideways)) << "a right angle advances nothing, forever";

    auto still            = defaults();
    still.pulse_advance_m = 0.0;
    EXPECT_FALSE(limitsAreUsable(still));

    EXPECT_EQ(planApproach(1.0, 0.0, no_room).move, ApproachMove::kInvalid);
}

TEST(ApproachPlanner, TheLeftArmWindowIsTheRightArmWindowMirrored)
{
    auto left               = defaults();
    left.target_bearing_rad = -left.target_bearing_rad;

    // Same object, opposite sides of the robot: what one arm is done with, the other must turn
    // to face.
    const auto for_left = planApproach(left.target_range_m, left.target_bearing_rad, left);
    EXPECT_EQ(for_left.move, ApproachMove::kDone);
    const auto for_right = planApproach(left.target_range_m, left.target_bearing_rad, defaults());
    EXPECT_EQ(for_right.move, ApproachMove::kTurn);
}

}  // namespace
