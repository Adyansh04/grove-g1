/**
 * @file test_approach_planner.cpp
 * @brief The geometry that decides whether the base can be walked into arm's reach.
 *
 * Worth testing away from a simulator because a live run exercises exactly one trajectory,
 * while the branches that matter are the ones a good run never touches: the overshoot that
 * cannot be undone, the oblique angle that makes a 0.29 m step land inside a 0.09 m window, and
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

/// The shipped window: the manipulation scene's proven grasp at base-frame (0.28, -0.20).
ApproachLimits defaults() { return ApproachLimits{}; }

/// The object sitting exactly where the arm wants it.
ApproachMove moveFor(double x, double y, double heading_error, const ApproachLimits& limits)
{
    return planApproach(x, y, heading_error, limits).move;
}

TEST(ApproachPlanner, ObjectInTheWindowOnTheWorkingHeadingIsDone)
{
    const auto limits = defaults();
    EXPECT_EQ(moveFor(limits.target_x_m, limits.target_y_m, 0.0, limits), ApproachMove::kDone);
}

TEST(ApproachPlanner, FarAwayItStepsStraightIn)
{
    const auto limits  = defaults();
    const auto command = planApproach(limits.target_x_m + 2.0, limits.target_y_m, 0.0, limits);
    ASSERT_EQ(command.move, ApproachMove::kStep);
    EXPECT_DOUBLE_EQ(command.oblique_rad, 0.0) << "no reason to walk crooked from two metres out";
    EXPECT_NEAR(command.forward_error_m, 2.0, 1e-9);
}

TEST(ApproachPlanner, ForwardIsResolvedBeforeHeadingAndLateral)
{
    // A step is the only move that wants a deliberately wrong heading, so resolving heading
    // first would just undo the aim the step is about to ask for.
    const auto limits = defaults();
    EXPECT_EQ(
        moveFor(limits.target_x_m + 1.0, limits.target_y_m + 0.5, 1.0, limits),
        ApproachMove::kStep);
}

TEST(ApproachPlanner, TheObliqueAngleMakesTheAdvanceMatchWhatIsLeft)
{
    auto limits            = defaults();
    limits.pulse_advance_m = 0.40;
    // 0.15 of a 0.40 step needs 68 degrees, inside the cap the next test covers. Pick anything
    // smaller and this measures the cap instead of the geometry.
    const double remaining = 0.15;
    const auto   command =
        planApproach(limits.target_x_m + remaining, limits.target_y_m, 0.0, limits);

    ASSERT_EQ(command.move, ApproachMove::kStep);
    EXPECT_NEAR(command.forward_error_m, remaining, 1e-9);
    // The whole point: one step along this heading closes exactly what is left.
    EXPECT_NEAR(limits.pulse_advance_m * std::cos(command.oblique_rad), remaining, 1e-6);
}

TEST(ApproachPlanner, TheObliqueIsCappedSoTheStrafeCleanupStaysAffordable)
{
    auto limits            = defaults();
    limits.pulse_advance_m = 0.60;
    limits.max_oblique_rad = 1.2;
    // 1 cm to close with a 60 cm step wants 89 degrees, which is almost pure lateral motion and
    // would then cost seventeen strafes to undo.
    const auto command = planApproach(limits.target_x_m + 0.01, limits.target_y_m, 0.0, limits);
    EXPECT_LE(std::abs(command.oblique_rad), limits.max_oblique_rad + 1e-9);
}

TEST(ApproachPlanner, TheObliqueLeansTowardTheSideTheLateralErrorNeeds)
{
    const auto limits = defaults();
    // The step's sideways excursion is unavoidable, so it should at least pay down the lateral
    // error rather than add to it. Both signs advance identically, so this costs nothing.
    const auto needs_left =
        planApproach(limits.target_x_m + 0.10, limits.target_y_m + 0.30, 0.0, limits);
    ASSERT_EQ(needs_left.move, ApproachMove::kStep);
    EXPECT_GT(needs_left.oblique_rad, 0.0);

    const auto needs_right =
        planApproach(limits.target_x_m + 0.10, limits.target_y_m - 0.30, 0.0, limits);
    ASSERT_EQ(needs_right.move, ApproachMove::kStep);
    EXPECT_LT(needs_right.oblique_rad, 0.0);
}

TEST(ApproachPlanner, HeadingIsRestoredBeforeStrafing)
{
    const auto limits = defaults();
    // A strafe on the wrong heading moves in the wrong world direction, and the last step
    // deliberately left the heading off by its oblique.
    EXPECT_EQ(
        moveFor(limits.target_x_m, limits.target_y_m + 0.30, 0.5, limits),
        ApproachMove::kTurn);
    EXPECT_EQ(
        moveFor(limits.target_x_m, limits.target_y_m + 0.30, 0.0, limits),
        ApproachMove::kStrafe);
}

TEST(ApproachPlanner, StrafeGoesTowardTheObject)
{
    const auto limits = defaults();
    EXPECT_GT(
        planApproach(limits.target_x_m, limits.target_y_m + 0.30, 0.0, limits).lateral_sign,
        0.0);
    EXPECT_LT(
        planApproach(limits.target_x_m, limits.target_y_m - 0.30, 0.0, limits).lateral_sign,
        0.0);
}

TEST(ApproachPlanner, TooCloseIsTerminalBecauseTheGaitCannotReverse)
{
    const auto limits = defaults();
    EXPECT_EQ(
        moveFor(limits.min_forward_m - 0.01, limits.target_y_m, 0.0, limits),
        ApproachMove::kOvershot);
    // Past the near edge of the window but not yet under the robot: still unrecoverable, since
    // there is no command that moves the base backwards.
    EXPECT_EQ(
        moveFor(limits.target_x_m - limits.forward_tolerance_m - 0.01, limits.target_y_m, 0.0, limits),
        ApproachMove::kOvershot);
}

TEST(ApproachPlanner, AQuantumBiggerThanTheGapStillProducesAFiniteAngle)
{
    auto limits            = defaults();
    limits.pulse_advance_m = 0.293;
    for (double remaining = 0.001; remaining < 0.29; remaining += 0.01)
    {
        const auto command =
            planApproach(limits.target_x_m + remaining, limits.target_y_m, 0.0, limits);
        EXPECT_TRUE(std::isfinite(command.oblique_rad)) << "remaining " << remaining;
    }
}

TEST(ApproachPlanner, TheQuantumEstimateOnlyEverGrows)
{
    // Asymmetric on purpose: undershooting costs another pulse, overshooting is unrecoverable.
    EXPECT_DOUBLE_EQ(maxObservedAdvance(0.293, 0.60), 0.60);
    EXPECT_DOUBLE_EQ(maxObservedAdvance(0.60, 0.293), 0.60);
    // A pulse that produced nothing means the gait failed to start, not that it can step zero.
    EXPECT_DOUBLE_EQ(maxObservedAdvance(0.293, 0.0), 0.293);
    EXPECT_DOUBLE_EQ(maxObservedAdvance(0.293, -1.0), 0.293);
}

TEST(ApproachPlanner, UnusableLimitsAreRefusedRatherThanAimedAt)
{
    EXPECT_TRUE(limitsAreUsable(defaults()));

    auto no_room          = defaults();
    no_room.min_forward_m = no_room.target_x_m;
    EXPECT_FALSE(limitsAreUsable(no_room)) << "the window would sit entirely inside the robot";

    auto sideways            = defaults();
    sideways.max_oblique_rad = M_PI_2;
    EXPECT_FALSE(limitsAreUsable(sideways)) << "a right angle advances nothing, forever";

    auto still            = defaults();
    still.pulse_advance_m = 0.0;
    EXPECT_FALSE(limitsAreUsable(still));

    EXPECT_EQ(moveFor(1.0, 0.0, 0.0, no_room), ApproachMove::kInvalid);
}

TEST(ApproachPlanner, TheLeftArmWindowIsTheRightArmWindowMirrored)
{
    auto left       = defaults();
    left.target_y_m = -left.target_y_m;

    // Same object, opposite sides of the robot: what one arm is done with, the other must
    // sidestep to reach.
    EXPECT_EQ(moveFor(left.target_x_m, left.target_y_m, 0.0, left), ApproachMove::kDone);
    EXPECT_EQ(moveFor(left.target_x_m, left.target_y_m, 0.0, defaults()), ApproachMove::kStrafe);
}

}  // namespace
