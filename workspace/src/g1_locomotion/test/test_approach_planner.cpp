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
    // Anything a straight step would carry past the window is closed by creeping instead. There
    // is no smaller forward pulse to reach for: forward is irreducible at about 0.29 m.
    const auto command = planApproach(limits.target_x_m + 0.12, limits.target_y_m, 0.0, limits);
    ASSERT_EQ(command.move, ApproachMove::kCreep);
    EXPECT_NEAR(std::abs(command.fine_offset_rad), limits.fine_offset_rad, 1e-9);
}

TEST(ApproachPlanner, AStraightStepCanNeverCarryTheRobotPastTheWindow)
{
    auto limits = defaults();
    // pulse_advance_m is maintained as an UPPER bound on what a step does, so the planner only
    // steps while more than a full step is left. Sweeping the boundary is the cheap way to
    // assert the invariant that matters: the gait has no reverse, so an overshoot is terminal.
    for (double remaining = 0.0; remaining < 1.5; remaining += 0.005)
    {
        const auto command =
            planApproach(limits.target_x_m + remaining, limits.target_y_m, 0.0, limits);
        if (command.move == ApproachMove::kStep)
        {
            const double after = remaining - limits.pulse_advance_m;
            EXPECT_GT(after, -limits.forward_tolerance_m)
                << "a step from " << remaining << " would land past the window";
        }
    }
}

TEST(ApproachPlanner, TheCreepTurnsAwayFromTheSideItNeedsToSlideToward)
{
    const auto limits = defaults();
    // Turning left and strafing right advances and slides right; turning right and strafing left
    // advances and slides left. So the offset is signed opposite the strafe, and the slide pays
    // down the lateral error instead of adding to it.
    const auto needs_left =
        planApproach(limits.target_x_m + 0.12, limits.target_y_m + 0.30, 0.0, limits);
    ASSERT_EQ(needs_left.move, ApproachMove::kCreep);
    EXPECT_GT(needs_left.lateral_sign, 0.0) << "strafe left";
    EXPECT_LT(needs_left.fine_offset_rad, 0.0) << "so turn right";

    const auto needs_right =
        planApproach(limits.target_x_m + 0.12, limits.target_y_m - 0.30, 0.0, limits);
    ASSERT_EQ(needs_right.move, ApproachMove::kCreep);
    EXPECT_LT(needs_right.lateral_sign, 0.0) << "strafe right";
    EXPECT_GT(needs_right.fine_offset_rad, 0.0) << "so turn left";
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
    sideways.fine_offset_rad = M_PI_2;
    EXPECT_FALSE(limitsAreUsable(sideways))
        << "a right-angle creep never returns to the working heading's lateral axis";

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
