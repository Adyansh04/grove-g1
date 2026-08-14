/**
 * @file test_approach_planner.cpp
 * @brief The decisions that walk the base into arm's reach.
 *
 * Worth testing away from a simulator because a live run exercises one trajectory, while the
 * branches that matter are the ones a good run never touches: the overshoot that is recoverable,
 * the one that is not, and the order the axes are corrected in.
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

/// The shipped window: the arm's measured band at the workbench.
ApproachLimits defaults() { return ApproachLimits{}; }

ApproachMove moveFor(double x, double y, const ApproachLimits& limits)
{
    return planApproach(x, y, limits).move;
}

TEST(ApproachPlanner, ObjectInTheWindowIsDone)
{
    const auto limits = defaults();
    EXPECT_EQ(moveFor(limits.target_x_m, limits.target_y_m, limits), ApproachMove::kDone);
}

TEST(ApproachPlanner, FarAwayItDrivesForwardAndMayCoast)
{
    const auto limits  = defaults();
    const auto command = planApproach(limits.target_x_m + 2.0, limits.target_y_m, limits);
    ASSERT_EQ(command.move, ApproachMove::kStep);
    EXPECT_TRUE(command.coarse) << "two metres out, the caller may stop early and let it coast";
    EXPECT_NEAR(command.forward_error_m, 2.0, 1e-9);
}

TEST(ApproachPlanner, ASubStepGapDrivesToZeroInsteadOfCoasting)
{
    const auto limits = defaults();
    // Forward is irreducible at about 0.29 m, so a small gap is closed by deliberately going too
    // far and reversing back. `coarse` false is what tells the caller not to stop early.
    const auto command = planApproach(
        limits.target_x_m + limits.forward_tolerance_m + 0.02,
        limits.target_y_m,
        limits);
    ASSERT_EQ(command.move, ApproachMove::kStep);
    EXPECT_FALSE(command.coarse);
}

TEST(ApproachPlanner, PastTheWindowIsRecoveredByReversing)
{
    const auto limits = defaults();
    // Coming too far costs no turning. g1_gait_shaper refuses a planner's backup speeds but
    // passes a deliberate -0.60, which the policy measures at -0.247 m/s.
    const double past    = limits.target_x_m - limits.forward_tolerance_m - 0.02;
    const auto   command = planApproach(past, limits.target_y_m, limits);
    ASSERT_EQ(command.move, ApproachMove::kReverse);
    EXPECT_LT(command.forward_error_m, 0.0);
}

TEST(ApproachPlanner, OnlyTheObjectBeingUnderTheRobotIsTerminal)
{
    const auto limits = defaults();
    EXPECT_EQ(
        moveFor(limits.min_forward_m - 0.01, limits.target_y_m, limits),
        ApproachMove::kOvershot);
}

TEST(ApproachPlanner, LateralIsStrafedAndGoesTowardTheObject)
{
    const auto limits = defaults();
    EXPECT_GT(planApproach(limits.target_x_m, limits.target_y_m + 0.30, limits).lateral_sign, 0.0);
    EXPECT_LT(planApproach(limits.target_x_m, limits.target_y_m - 0.30, limits).lateral_sign, 0.0);
    EXPECT_EQ(moveFor(limits.target_x_m, limits.target_y_m + 0.30, limits), ApproachMove::kStrafe);
}

TEST(ApproachPlanner, ForwardIsCorrectedBeforeLateral)
{
    const auto limits = defaults();
    // Not arbitrary: the forward drive is the move that covers real distance, and the lateral
    // error it leaves behind is cheap to strafe out afterwards. The other order would strafe to
    // a place the next drive walks away from.
    EXPECT_EQ(
        moveFor(limits.target_x_m + 1.0, limits.target_y_m + 0.5, limits),
        ApproachMove::kStep);
}

TEST(ApproachPlanner, EveryRegionMapsToAMoveTheCallerHandles)
{
    const auto limits = defaults();
    // A plan the caller has no handler for is a hang, not an error -- silent and mid-mission.
    // This sweep pins every position to a move that is not kInvalid, so a new move can only be
    // added alongside its handling.
    for (double fwd : { -0.30, -0.05, 0.05, 0.30, 1.00 })
    {
        for (double lat : { -0.30, 0.0, 0.30 })
        {
            const auto move = moveFor(limits.target_x_m + fwd, limits.target_y_m + lat, limits);
            EXPECT_NE(move, ApproachMove::kInvalid) << "fwd " << fwd << " lat " << lat;
        }
    }
}

TEST(ApproachPlanner, UnusableLimitsAreRefusedRatherThanAimedAt)
{
    EXPECT_TRUE(limitsAreUsable(defaults()));

    auto no_room          = defaults();
    no_room.min_forward_m = no_room.target_x_m;
    EXPECT_FALSE(limitsAreUsable(no_room)) << "the window would sit entirely inside the robot";

    auto still             = defaults();
    still.step_threshold_m = 0.0;
    EXPECT_FALSE(limitsAreUsable(still));

    EXPECT_EQ(moveFor(1.0, 0.0, no_room), ApproachMove::kInvalid);
}

TEST(ApproachPlanner, TheLeftArmWindowIsTheRightArmWindowMirrored)
{
    auto left       = defaults();
    left.target_y_m = -left.target_y_m;

    EXPECT_EQ(moveFor(left.target_x_m, left.target_y_m, left), ApproachMove::kDone);
    EXPECT_EQ(moveFor(left.target_x_m, left.target_y_m, defaults()), ApproachMove::kStrafe);
}

}  // namespace
