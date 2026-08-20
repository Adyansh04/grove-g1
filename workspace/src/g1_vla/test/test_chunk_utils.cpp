/**
 * @file test_chunk_utils.cpp
 * @brief The checks a chunk has to survive before any of it is executed.
 *
 * These are the gate's whole reject-before-moving story, so what matters is that each one
 * actually fires: a check that silently passes everything reads exactly like a safe robot.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "g1_vla/chunk_utils.hpp"

using g1_vla::JointMap;
using g1_vla::maxSegmentStep;
using g1_vla::maxVelocityRatio;
using g1_vla::splitByController;
using g1_vla::startJump;
using g1_vla::trackingVelocity;
using g1_vla::wellFormed;
using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;

namespace
{

const std::vector<std::string> kArm  = { "right_elbow_joint", "right_shoulder_pitch_joint" };
const std::vector<std::string> kHand = { "right_hand_index_0_joint" };

/// A chunk over @p names whose rows are the successive waypoints, each one dt further along.
JointTrajectory chunkOf(
    const std::vector<std::string>& names, const std::vector<std::vector<double>>& rows, double dt)
{
    JointTrajectory chunk;
    chunk.joint_names = names;
    double t          = dt;
    for (const std::vector<double>& row : rows)
    {
        JointTrajectoryPoint point;
        point.positions           = row;
        point.time_from_start.sec = static_cast<int32_t>(t);
        point.time_from_start.nanosec =
            static_cast<uint32_t>(std::lround((t - std::floor(t)) * 1e9));
        chunk.points.push_back(point);
        t += dt;
    }
    return chunk;
}

}  // namespace

TEST(WellFormed, AcceptsAPlainChunk)
{
    EXPECT_TRUE(wellFormed(chunkOf(kArm, { { 0.0, 0.0 }, { 0.1, 0.1 } }, 0.1)));
}

TEST(WellFormed, RejectsAnEmptyChunk)
{
    EXPECT_FALSE(wellFormed(JointTrajectory{}));
    EXPECT_FALSE(wellFormed(chunkOf(kArm, {}, 0.1)));
}

TEST(WellFormed, RejectsARowNarrowerThanTheNames)
{
    JointTrajectory chunk = chunkOf(kArm, { { 0.0, 0.0 } }, 0.1);
    chunk.points[0].positions.pop_back();
    EXPECT_FALSE(wellFormed(chunk));
}

TEST(WellFormed, RejectsTimesThatDoNotAdvance)
{
    JointTrajectory chunk           = chunkOf(kArm, { { 0.0, 0.0 }, { 0.1, 0.1 } }, 0.1);
    chunk.points[1].time_from_start = chunk.points[0].time_from_start;
    EXPECT_FALSE(wellFormed(chunk));

    // A first waypoint at t=0 is one the controller has no time to reach.
    JointTrajectory immediate           = chunkOf(kArm, { { 0.0, 0.0 } }, 0.1);
    immediate.points[0].time_from_start = builtin_interfaces::msg::Duration{};
    EXPECT_FALSE(wellFormed(immediate));
}

TEST(Split, KeepsOnlyTheControllersOwnJoints)
{
    JointTrajectory chunk = chunkOf(
        { "right_elbow_joint", "right_hand_index_0_joint", "right_shoulder_pitch_joint" },
        { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 } },
        0.1);

    const JointTrajectory arm = splitByController(chunk, kArm);

    EXPECT_THAT(
        arm.joint_names,
        testing::ElementsAre("right_elbow_joint", "right_shoulder_pitch_joint"));
    EXPECT_THAT(arm.points[0].positions, testing::ElementsAre(1.0, 3.0));
    EXPECT_THAT(arm.points[1].positions, testing::ElementsAre(4.0, 6.0));
    EXPECT_EQ(arm.points.size(), 2U);
}

TEST(Split, CarriesTheWaypointTimesThrough)
{
    JointTrajectory chunk = chunkOf(kArm, { { 0.0, 0.0 }, { 0.1, 0.1 } }, 0.25);

    const JointTrajectory arm = splitByController(chunk, kArm);

    EXPECT_EQ(arm.points[1].time_from_start, chunk.points[1].time_from_start);
}

TEST(Split, ReturnsNothingForAControllerTheChunkDoesNotName)
{
    const JointTrajectory hand = splitByController(chunkOf(kArm, { { 0.0, 0.0 } }, 0.1), kHand);

    EXPECT_TRUE(hand.joint_names.empty());
    EXPECT_TRUE(hand.points.empty());
}

TEST(StartJump, MeasuresTheGapToTheFirstWaypoint)
{
    const JointMap measured = { { "right_elbow_joint", 0.5 },
                                { "right_shoulder_pitch_joint", -0.2 } };

    const auto jump = startJump(chunkOf(kArm, { { 0.55, 0.1 } }, 0.1), measured);

    ASSERT_TRUE(jump.has_value());
    EXPECT_NEAR(*jump, 0.3, 1e-9);
}

TEST(StartJump, FailsWhenAJointWasNotMeasured)
{
    const JointMap measured = { { "right_elbow_joint", 0.5 } };

    EXPECT_FALSE(startJump(chunkOf(kArm, { { 0.5, 0.0 } }, 0.1), measured).has_value());
}

TEST(SegmentStep, FindsTheWidestGapBetweenWaypoints)
{
    const JointTrajectory chunk = chunkOf(kArm, { { 0.0, 0.0 }, { 0.1, 0.0 }, { 0.1, 0.7 } }, 0.1);

    EXPECT_NEAR(maxSegmentStep(chunk), 0.7, 1e-9);
}

TEST(SegmentStep, IsZeroForASingleWaypoint)
{
    EXPECT_EQ(maxSegmentStep(chunkOf(kArm, { { 3.0, 3.0 } }, 0.1)), 0.0);
}

TEST(VelocityRatio, CountsTheMoveOffTheMeasuredPose)
{
    const JointMap measured = { { "right_elbow_joint", 0.0 },
                                { "right_shoulder_pitch_joint", 0.0 } };
    const JointMap limits = { { "right_elbow_joint", 1.0 }, { "right_shoulder_pitch_joint", 1.0 } };

    // 0.2 rad in the first 0.1 s is 2 rad/s against a limit of 1.
    const auto ratio = maxVelocityRatio(chunkOf(kArm, { { 0.2, 0.0 } }, 0.1), measured, limits);

    ASSERT_TRUE(ratio.has_value());
    EXPECT_NEAR(*ratio, 2.0, 1e-9);
}

TEST(VelocityRatio, UsesEachSegmentsOwnDuration)
{
    const JointMap measured = { { "right_elbow_joint", 0.0 },
                                { "right_shoulder_pitch_joint", 0.0 } };
    const JointMap limits = { { "right_elbow_joint", 2.0 }, { "right_shoulder_pitch_joint", 2.0 } };

    const auto ratio =
        maxVelocityRatio(chunkOf(kArm, { { 0.1, 0.0 }, { 0.5, 0.0 } }, 0.1), measured, limits);

    ASSERT_TRUE(ratio.has_value());
    // The second segment moves 0.4 rad in 0.1 s: 4 rad/s, twice the limit.
    EXPECT_NEAR(*ratio, 2.0, 1e-9);
}

TEST(VelocityRatio, FailsWhenAJointHasNoUsableLimit)
{
    const JointMap        measured = { { "right_elbow_joint", 0.0 },
                                       { "right_shoulder_pitch_joint", 0.0 } };
    const JointTrajectory chunk    = chunkOf(kArm, { { 0.0, 0.0 } }, 0.1);

    EXPECT_FALSE(maxVelocityRatio(chunk, measured, { { "right_elbow_joint", 1.0 } }).has_value());
    EXPECT_FALSE(maxVelocityRatio(
                     chunk,
                     measured,
                     { { "right_elbow_joint", 1.0 }, { "right_shoulder_pitch_joint", 0.0 } })
                     .has_value());
}

TEST(TrackingVelocity, AimsAtTheNextWaypointFromWhereTheArmIs)
{
    const JointMap        measured = { { "right_elbow_joint", 0.0 },
                                       { "right_shoulder_pitch_joint", 0.0 } };
    const JointTrajectory chunk    = chunkOf(kArm, { { 0.02, 0.0 }, { 0.02, 0.05 } }, 0.1);

    // 0.02 rad away, 0.1 s left to get there.
    EXPECT_THAT(
        trackingVelocity(chunk, measured, 0.0, 0.02),
        testing::ElementsAre(testing::DoubleNear(0.2, 1e-9), testing::DoubleNear(0.0, 1e-9)));
    // Half way through the segment the arm has not moved, so the command doubles to still make
    // the waypoint on time. An open-loop velocity would have kept sending 0.2 and fallen short.
    EXPECT_THAT(
        trackingVelocity(chunk, measured, 0.05, 0.02),
        testing::ElementsAre(testing::DoubleNear(0.4, 1e-9), testing::DoubleNear(0.0, 1e-9)));
}

TEST(TrackingVelocity, CorrectsAnArmThatHasDriftedPastTheWaypoint)
{
    // Overshot the first waypoint: the command has to point back.
    const JointMap        measured = { { "right_elbow_joint", 0.05 },
                                       { "right_shoulder_pitch_joint", 0.0 } };
    const JointTrajectory chunk    = chunkOf(kArm, { { 0.02, 0.0 } }, 0.1);

    EXPECT_LT(trackingVelocity(chunk, measured, 0.0, 0.02).front(), 0.0);
}

TEST(TrackingVelocity, FloorsTheTimeLeftSoALateTickDoesNotDivideByZero)
{
    const JointMap        measured = { { "right_elbow_joint", 0.0 },
                                       { "right_shoulder_pitch_joint", 0.0 } };
    const JointTrajectory chunk    = chunkOf(kArm, { { 0.02, 0.0 } }, 0.1);

    const std::vector<double> velocities = trackingVelocity(chunk, measured, 0.0999, 0.02);
    ASSERT_EQ(velocities.size(), 2U);
    EXPECT_NEAR(velocities[0], 1.0, 1e-9);
}

TEST(TrackingVelocity, IsEmptyOnceTheChunkIsDone)
{
    const JointMap        measured = { { "right_elbow_joint", 0.0 },
                                       { "right_shoulder_pitch_joint", 0.0 } };
    const JointTrajectory chunk    = chunkOf(kArm, { { 0.02, 0.0 } }, 0.1);

    EXPECT_TRUE(trackingVelocity(chunk, measured, 0.1, 0.02).empty());
    EXPECT_TRUE(trackingVelocity(chunk, measured, 5.0, 0.02).empty());
}

TEST(TrackingVelocity, IsEmptyWhenAJointWasNotMeasured)
{
    EXPECT_TRUE(trackingVelocity(chunkOf(kArm, { { 0.02, 0.0 } }, 0.1), {}, 0.0, 0.02).empty());
}
