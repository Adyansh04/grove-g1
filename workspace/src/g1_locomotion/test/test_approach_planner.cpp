/**
 * @file test_approach_planner.cpp
 * @brief The control law that walks the base into arm's reach.
 *
 * Worth testing away from a simulator because a live run exercises one trajectory, while the
 * properties that matter are the ones a good run never touches: the deadband floor that makes
 * the law converge at all, the overshoot that is recoverable, and the one that is not.
 */

#include <gmock/gmock.h>

#include <cmath>

#include "g1_locomotion/approach_planner.hpp"

namespace
{

using g1_locomotion::ApproachLimits;
using g1_locomotion::ApproachState;
using g1_locomotion::GaitLimits;
using g1_locomotion::gaitLimitsAreUsable;
using g1_locomotion::limitsAreUsable;
using g1_locomotion::planApproach;

/// The shipped window: the arm's measured band at the workbench.
ApproachLimits defaults() { return ApproachLimits{}; }
GaitLimits     gait() { return GaitLimits{}; }

/// Square to the surface, which is the case for everything except the heading tests.
auto plan(double x, double y, const ApproachLimits& limits)
{
    return planApproach(x, y, 0.0, limits, gait());
}

TEST(ApproachPlanner, ObjectInTheWindowIsArrivedAndCommandsNothing)
{
    const auto command = plan(0.270, -0.220, defaults());
    EXPECT_EQ(command.state, ApproachState::kArrived);
    EXPECT_DOUBLE_EQ(command.vx_mps, 0.0);
    EXPECT_DOUBLE_EQ(command.vy_mps, 0.0);
    EXPECT_DOUBLE_EQ(command.yaw_rate_rps, 0.0);
}

TEST(ApproachPlanner, AnErrorJustOutsideTheWindowStillClearsTheGaitDeadband)
{
    // The property the whole law rests on. 0.001 m past the tolerance is a 0.111 m error, and a
    // plain proportional term would ask for 0.111 m/s, which this gait ignores entirely
    // (measured 0.016 m/s delivered for a commanded 0.10). Without the floor the approach
    // stalls a centimetre outside the window and burns its whole timeout there.
    auto       limits = defaults();
    const auto command =
        plan(limits.target_x_m + limits.forward_tolerance_m + 0.001, -0.220, limits);

    EXPECT_EQ(command.state, ApproachState::kClosing);
    EXPECT_GE(command.vx_mps, gait().min_speed_x_mps);
}

TEST(ApproachPlanner, ASmallLateralErrorClearsTheLateralFloorWhichIsHigher)
{
    // Lateral's floor sits above forward's because the gait tracks it worse: 0.20 commanded
    // delivers only 0.083 m/s sideways, against 0.123 forward.
    auto       limits = defaults();
    const auto command =
        plan(0.270, limits.target_y_m + limits.lateral_tolerance_m + 0.001, limits);

    EXPECT_EQ(command.state, ApproachState::kClosing);
    EXPECT_GE(command.vy_mps, gait().min_speed_y_mps);
}

TEST(ApproachPlanner, ABigErrorIsCappedRatherThanScaledUp)
{
    const auto command = plan(2.0, 1.5, defaults());
    EXPECT_EQ(command.state, ApproachState::kClosing);
    EXPECT_DOUBLE_EQ(command.vx_mps, gait().max_speed_x_mps);
    EXPECT_DOUBLE_EQ(command.vy_mps, gait().max_speed_y_mps);
}

TEST(ApproachPlanner, BothAxesAreDrivenAtOnce)
{
    // The old gait needed forward and lateral resolved one at a time. This one takes a velocity
    // and returns a proportional fraction of it on every axis, so there is nothing to sequence.
    const auto command = plan(0.600, 0.100, defaults());
    EXPECT_GT(command.vx_mps, 0.0);
    EXPECT_GT(command.vy_mps, 0.0);
}

TEST(ApproachPlanner, VelocitiesPointAtTheError)
{
    // Sign errors here walk the robot away from the object, which reads as a stuck approach
    // rather than as a wrong direction.
    const auto too_far  = plan(0.600, -0.220, defaults());
    const auto too_near = plan(0.150, -0.220, defaults());
    EXPECT_GT(too_far.vx_mps, 0.0);
    EXPECT_LT(too_near.vx_mps, 0.0);

    const auto to_the_left  = plan(0.270, 0.100, defaults());
    const auto to_the_right = plan(0.270, -0.500, defaults());
    EXPECT_GT(to_the_left.vy_mps, 0.0);
    EXPECT_LT(to_the_right.vy_mps, 0.0);
}

TEST(ApproachPlanner, PastTheWindowIsRecoveredByReversing)
{
    // Being too close is not terminal: the gait reverses about as well as it advances.
    const auto command = plan(0.120, -0.220, defaults());
    EXPECT_EQ(command.state, ApproachState::kClosing);
    EXPECT_LE(command.vx_mps, -gait().min_speed_x_mps);
}

TEST(ApproachPlanner, OnlyTheObjectBeingUnderTheRobotIsTerminal)
{
    const auto limits = defaults();
    EXPECT_EQ(plan(limits.min_forward_m - 0.001, -0.220, limits).state, ApproachState::kOvershot);
    EXPECT_EQ(plan(limits.min_forward_m + 0.001, -0.220, limits).state, ApproachState::kClosing);
}

TEST(ApproachPlanner, HeadingIsHeldWhileClosingButIsNotPartOfArriving)
{
    const auto   limits = defaults();
    const double off    = limits.heading_tolerance_rad + 0.2;

    // Inside the window on both axes: arrived, and no yaw even though the robot is not square.
    const auto arrived = planApproach(0.270, -0.220, off, limits, gait());
    EXPECT_EQ(arrived.state, ApproachState::kArrived);
    EXPECT_DOUBLE_EQ(arrived.yaw_rate_rps, 0.0);

    // Outside it: the same heading error now gets corrected, in the direction of the error.
    const auto closing = planApproach(0.600, -0.220, off, limits, gait());
    EXPECT_EQ(closing.state, ApproachState::kClosing);
    EXPECT_GT(closing.yaw_rate_rps, 0.0);
    EXPECT_LT(planApproach(0.600, -0.220, -off, limits, gait()).yaw_rate_rps, 0.0);
}

TEST(ApproachPlanner, SmallHeadingErrorsAreLeftAloneRatherThanFloored)
{
    // Yaw has no deadband and tracks near 1:1, so it needs no floor, and flooring it would
    // swing the robot past square for a couple of degrees of error.
    const auto limits = defaults();
    const auto command =
        planApproach(0.600, -0.220, limits.heading_tolerance_rad - 0.01, limits, gait());
    EXPECT_DOUBLE_EQ(command.yaw_rate_rps, 0.0);
}

TEST(ApproachPlanner, YawIsCappedInBothDirections)
{
    const auto limits = defaults();
    EXPECT_DOUBLE_EQ(
        planApproach(0.600, -0.220, 3.0, limits, gait()).yaw_rate_rps,
        gait().max_yaw_rate_rps);
    EXPECT_DOUBLE_EQ(
        planApproach(0.600, -0.220, -3.0, limits, gait()).yaw_rate_rps,
        -gait().max_yaw_rate_rps);
}

TEST(ApproachPlanner, ErrorsAreReportedWhateverTheState)
{
    const auto command = plan(0.500, -0.100, defaults());
    EXPECT_NEAR(command.forward_error_m, 0.230, 1e-9);
    EXPECT_NEAR(command.lateral_error_m, 0.120, 1e-9);
}

TEST(ApproachPlanner, UnusableLimitsAreRefusedRatherThanAimedAt)
{
    EXPECT_TRUE(limitsAreUsable(defaults()));

    auto no_window                = defaults();
    no_window.forward_tolerance_m = 0.0;
    EXPECT_FALSE(limitsAreUsable(no_window));

    // The floor at or above the window's near end turns the recoverable overshoot into an abort.
    auto floor_too_high          = defaults();
    floor_too_high.min_forward_m = floor_too_high.target_x_m;
    EXPECT_FALSE(limitsAreUsable(floor_too_high));

    EXPECT_EQ(plan(0.270, -0.220, no_window).state, ApproachState::kInvalid);
}

TEST(ApproachPlanner, UnusableGaitLimitsAreRefusedRatherThanCommanded)
{
    EXPECT_TRUE(gaitLimitsAreUsable(gait()));

    // A floor above its ceiling clamps every command to the floor, so the robot would drive at
    // the deadband speed no matter how close it got.
    auto inverted            = gait();
    inverted.min_speed_x_mps = 0.9;
    inverted.max_speed_x_mps = 0.4;
    EXPECT_FALSE(gaitLimitsAreUsable(inverted));

    EXPECT_EQ(planApproach(0.600, -0.220, 0.0, defaults(), inverted).state, ApproachState::kInvalid);
}

TEST(ApproachPlanner, TheLeftArmWindowIsTheRightArmWindowMirrored)
{
    auto left       = defaults();
    left.target_y_m = -left.target_y_m;

    const auto right_side = plan(0.270, -0.220, defaults());
    const auto left_side  = plan(0.270, 0.220, left);
    EXPECT_EQ(right_side.state, ApproachState::kArrived);
    EXPECT_EQ(left_side.state, ApproachState::kArrived);

    // And an object on the wrong side is driven across, not accepted.
    EXPECT_EQ(plan(0.270, 0.220, defaults()).state, ApproachState::kClosing);
}

}  // namespace
