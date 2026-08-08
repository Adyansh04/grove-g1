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

TEST(ApproachPlanner, ASubStepRemainderCreepsInsteadOfStepping)
{
    const auto limits = defaults();
    // Anything under one step is closed by creeping. There is no smaller forward pulse to reach
    // for: forward is irreducible at about 0.29 m however short the command.
    const auto command = planApproach(limits.target_x_m + 0.12, limits.target_y_m, 0.0, limits);
    ASSERT_EQ(command.move, ApproachMove::kCreep);
    EXPECT_NEAR(std::abs(command.fine_offset_rad), limits.fine_offset_rad, 1e-9);
}

TEST(ApproachPlanner, MoreThanAStepLeftIsStepped)
{
    const auto limits = defaults();
    // The regression this guards: with the threshold set to an UPPER bound on the step instead
    // of a typical one, 0.58 m of gap was creeped rather than stepped, at 2.5 cm a pulse, and
    // the approach never arrived.
    EXPECT_EQ(
        moveFor(limits.target_x_m + 0.58, limits.target_y_m, 0.0, limits),
        ApproachMove::kStep);
    EXPECT_EQ(
        moveFor(limits.target_x_m + limits.step_threshold_m + 0.01, limits.target_y_m, 0.0, limits),
        ApproachMove::kStep);
    EXPECT_EQ(
        moveFor(limits.target_x_m + limits.step_threshold_m - 0.01, limits.target_y_m, 0.0, limits),
        ApproachMove::kCreep);
}

TEST(ApproachPlanner, TheCreepTurnsAwayFromTheSideItNeedsToSlideToward)
{
    const auto limits = defaults();
    // Reached only once lateral is already inside tolerance, so the residual it leans into is
    // small. The offset is still signed opposite the strafe: turning right and strafing left is
    // what carries the robot forward and left at the same time.
    const auto needs_left = planApproach(
        limits.target_x_m + limits.forward_tolerance_m + 0.02,
        limits.target_y_m + 0.5 * limits.lateral_tolerance_m,
        0.0,
        limits);
    ASSERT_EQ(needs_left.move, ApproachMove::kCreep);
    EXPECT_GT(needs_left.lateral_sign, 0.0);
    EXPECT_LT(needs_left.fine_offset_rad, 0.0);

    const auto needs_right = planApproach(
        limits.target_x_m + limits.forward_tolerance_m + 0.02,
        limits.target_y_m - 0.5 * limits.lateral_tolerance_m,
        0.0,
        limits);
    ASSERT_EQ(needs_right.move, ApproachMove::kCreep);
    EXPECT_LT(needs_right.lateral_sign, 0.0);
    EXPECT_GT(needs_right.fine_offset_rad, 0.0);
}

TEST(ApproachPlanner, HeadingIsCorrectedOnlyOncePositionIsRight)
{
    const auto limits = defaults();
    // Heading is last and loose: it is not part of reachability, only of how the robot stands.
    EXPECT_EQ(moveFor(limits.target_x_m, limits.target_y_m, 0.5, limits), ApproachMove::kTurn);
    EXPECT_EQ(moveFor(limits.target_x_m, limits.target_y_m, 0.0, limits), ApproachMove::kDone);
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

TEST(ApproachPlanner, OnlyTheObjectBeingUnderTheRobotIsTerminal)
{
    const auto limits = defaults();
    EXPECT_EQ(
        moveFor(limits.min_forward_m - 0.01, limits.target_y_m, 0.0, limits),
        ApproachMove::kOvershot);
}

TEST(ApproachPlanner, PastTheWindowIsRecoveredByReversing)
{
    const auto limits = defaults();
    // Coming too far is recoverable and costs no turning. g1_gait_shaper refuses a planner's
    // backup speeds but passes a deliberate -0.60, which the policy measures at -0.247 m/s.
    // Creeping backwards would work too and would cost a 45 degree turn each way.
    const double past    = limits.target_x_m - limits.forward_tolerance_m - 0.02;
    const auto   command = planApproach(past, limits.target_y_m, 0.0, limits);
    ASSERT_EQ(command.move, ApproachMove::kReverse);
    EXPECT_LT(command.forward_error_m, 0.0);
}

TEST(ApproachPlanner, ShortOfTheWindowCreepsForwardWithNoReverseAvailableThatFine)
{
    const auto limits = defaults();
    // The asymmetry is real: reverse resolves finely, forward does not. A sub-step gap in has to
    // be taken sideways.
    const auto command = planApproach(
        limits.target_x_m + limits.forward_tolerance_m + 0.02,
        limits.target_y_m,
        0.0,
        limits);
    ASSERT_EQ(command.move, ApproachMove::kCreep);
    EXPECT_NEAR(std::abs(command.fine_offset_rad), limits.fine_offset_rad, 1e-9);
}

TEST(ApproachPlanner, UnusableLimitsAreRefusedRatherThanAimedAt)
{
    EXPECT_TRUE(limitsAreUsable(defaults()));

    auto no_room          = defaults();
    no_room.min_forward_m = no_room.target_x_m;
    EXPECT_FALSE(limitsAreUsable(no_room)) << "the window would sit entirely inside the robot";

    auto sideways            = defaults();
    sideways.fine_offset_rad = M_PI_2;
    EXPECT_FALSE(limitsAreUsable(sideways))
        << "a right-angle creep never returns to the working heading's lateral axis";

    auto still             = defaults();
    still.step_threshold_m = 0.0;
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
