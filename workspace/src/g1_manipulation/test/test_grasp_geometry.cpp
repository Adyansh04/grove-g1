/**
 * @file test_grasp_geometry.cpp
 * @brief The arm/group mapping and the top-grasp goal, without a running MoveIt.
 */

#include <gmock/gmock.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "g1_manipulation/g1_manipulation_server_node.hpp"

using g1_manipulation::ArmContext;
using g1_manipulation::resolveArm;

namespace
{

constexpr double kDepth     = 0.026;
constexpr double kMinHeight = 0.080;

/// The orientation alone: zero height and no depth or minimum, so z passes through.
geometry_msgs::msg::Pose
graspFrameGoal(const geometry_msgs::msg::Pose& object, const std::vector<double>& rpy, bool is_left)
{
    return g1_manipulation::topGraspGoal(object, 0.0, 0.0, 0.0, rpy, is_left);
}

geometry_msgs::msg::Pose objectAt(double x, double y, double z)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x    = x;
    pose.position.y    = y;
    pose.position.z    = z;
    pose.orientation.w = 1.0;
    return pose;
}

}  // namespace

TEST(ResolveArm, RejectsAnythingElseWithoutAssigning)
{
    // No fallback to a side: a near-miss name would move the wrong arm.
    ArmContext arm;
    arm.arm_group = "sentinel";
    EXPECT_FALSE(resolveArm("Left", arm));
    EXPECT_FALSE(resolveArm("left_arm", arm));
    EXPECT_FALSE(resolveArm("", arm));
    EXPECT_EQ(arm.arm_group, "sentinel");
}

TEST(GraspFrameGoal, GoesStraightToTheObjectHorizontally)
{
    const auto target = objectAt(0.35, -0.20, 0.83);
    const auto goal   = graspFrameGoal(target, { -M_PI_2, 0.0, 0.0 }, /*is_left=*/false);

    EXPECT_DOUBLE_EQ(goal.position.x, target.position.x);
    EXPECT_DOUBLE_EQ(goal.position.y, target.position.y);
}

TEST(GraspFrameGoal, GripsUnderTheTopButNeverTooLow)
{
    const std::vector<double> rpy{ -M_PI_2, 0.0, 0.0 };
    // A 90 mm block on a 0.80 m surface: the minimum height wins over the depth under the top.
    const auto block = g1_manipulation::topGraspGoal(
        objectAt(0.3, -0.2, 0.845),
        0.09,
        kDepth,
        kMinHeight,
        rpy,
        false);
    EXPECT_NEAR(block.position.z, 0.80 + kMinHeight, 1e-9);

    // A 70 mm ball with a low minimum: the depth under the top wins.
    const auto ball =
        g1_manipulation::topGraspGoal(objectAt(0.3, -0.2, 0.835), 0.07, kDepth, 0.035, rpy, false);
    EXPECT_NEAR(ball.position.z, 0.87 - kDepth, 1e-9);
}

TEST(GraspFrameGoal, PointsTheClosingAxisAtTheFloor)
{
    // The Dex3 closes toward the palm's +y, so that axis points down for a top grasp, with +x
    // forward so the arm reaches out rather than the wrist contorting.
    const auto palm = graspFrameGoal(objectAt(0.4, 0.0, 0.8), { -M_PI_2, 0.0, 0.0 }, false);

    tf2::Quaternion rotation;
    tf2::fromMsg(palm.orientation, rotation);
    const tf2::Matrix3x3 basis(rotation);

    EXPECT_NEAR((basis * tf2::Vector3(0.0, 1.0, 0.0)).z(), -1.0, 1e-9)
        << "the palm's +y, where the fingers close, must point down";
    EXPECT_NEAR((basis * tf2::Vector3(1.0, 0.0, 0.0)).x(), 1.0, 1e-9)
        << "and the palm's +x stays forward";
}

TEST(GraspFrameGoal, TheTwoHandsMirror)
{
    // Mirror-image hands: the roll that points the right hand's closing axis down points the
    // left's up, so a shared orientation grasps upside down with the left.
    const auto right = graspFrameGoal(objectAt(0.35, 0.20, 0.83), { -M_PI_2, 0.0, 0.0 }, false);
    const auto left  = graspFrameGoal(objectAt(0.35, 0.20, 0.83), { -M_PI_2, 0.0, 0.0 }, true);

    tf2::Quaternion qr;
    tf2::Quaternion ql;
    tf2::fromMsg(right.orientation, qr);
    tf2::fromMsg(left.orientation, ql);

    EXPECT_NEAR((tf2::Matrix3x3(qr) * tf2::Vector3(0.0, 1.0, 0.0)).z(), -1.0, 1e-9);
    EXPECT_NEAR((tf2::Matrix3x3(ql) * tf2::Vector3(0.0, -1.0, 0.0)).z(), -1.0, 1e-9)
        << "the left hand closes toward its own -y, so its -y is what must point down";
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}

namespace
{

geometry_msgs::msg::Point pointAt(double x, double y, double z)
{
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

}  // namespace

TEST(PointingHand, StandsOnTheLineFromTheShoulderAtItsReach)
{
    const auto shoulder = pointAt(0.0, 0.2, 1.2);
    const auto hand     = g1_manipulation::pointingHand(shoulder, pointAt(3.0, 0.2, 1.2), 0.4);
    EXPECT_NEAR(hand.x, 0.4, 1e-9);
    EXPECT_NEAR(hand.y, 0.2, 1e-9);
    EXPECT_NEAR(hand.z, 1.2, 1e-9);

    // Pointing down at a mug on a table keeps the same direction.
    const auto low = g1_manipulation::pointingHand(shoulder, pointAt(0.8, 0.2, 0.6), 0.5);
    EXPECT_NEAR(low.x, 0.4, 1e-9);
    EXPECT_NEAR(low.z, 0.9, 1e-9);
}

TEST(PointingHand, ReachesATargetNearerThanItsReachAndStaysPutOnTheShoulder)
{
    const auto shoulder = pointAt(0.0, 0.0, 1.0);
    const auto near     = g1_manipulation::pointingHand(shoulder, pointAt(0.2, 0.0, 1.0), 0.4);
    EXPECT_NEAR(near.x, 0.2, 1e-9);
    const auto same = g1_manipulation::pointingHand(shoulder, shoulder, 0.4);
    EXPECT_NEAR(same.z, 1.0, 1e-9);
}

TEST(PointingHands, StepOutAwayFromTheOtherShoulderWithinReach)
{
    const auto left  = pointAt(0.0, 0.15, 1.2);
    const auto right = pointAt(0.0, -0.15, 1.2);
    const auto hand  = pointAt(0.3, -0.05, 0.9);
    const auto out   = g1_manipulation::pointingHands(hand, left, right, { 0.06, 0.12, 0.5 }, 0.48);
    // The 0.5 m step would leave the hand out of reach.
    ASSERT_EQ(out.size(), 3U);
    EXPECT_NEAR(out[0].y, -0.05, 1e-9);
    EXPECT_NEAR(out[1].y, 0.01, 1e-9);
    EXPECT_NEAR(out[2].y, 0.07, 1e-9);
    EXPECT_NEAR(out[2].x, 0.3, 1e-9);
    EXPECT_NEAR(out[2].z, 0.9, 1e-9);

    // The right hand steps the other way.
    const auto mirrored = g1_manipulation::pointingHands(hand, right, left, { 0.06 }, 0.48);
    ASSERT_EQ(mirrored.size(), 2U);
    EXPECT_NEAR(mirrored[1].y, -0.11, 1e-9);
}

TEST(HandOrientations, StartWithTheHandsOwnAndAreTheSameUnitRotationsEachTime)
{
    // Half a turn about z; a message's quaternion starts as the identity, w = 1.
    geometry_msgs::msg::Quaternion own;
    own.z             = 1.0;
    own.w             = 0.0;
    const auto first  = g1_manipulation::handOrientations(own, 8);
    const auto second = g1_manipulation::handOrientations(own, 8);
    ASSERT_EQ(first.size(), 9U);
    EXPECT_DOUBLE_EQ(first.front().z, 1.0);
    for (std::size_t i = 0; i < first.size(); ++i)
    {
        const auto& q = first[i];
        EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-9);
        EXPECT_DOUBLE_EQ(q.w, second[i].w);
    }
}
