/**
 * @file test_grasp_filter.cpp
 * @brief Pins the two conversions between a generated grasp and something this arm can be asked
 *        for, because getting either wrong puts the hand somewhere plausible and wrong.
 */

#include <gmock/gmock.h>

#include <array>
#include <cmath>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "g1_manipulation/grasp_filter.hpp"

namespace
{

using g1_manipulation::applyGripperOffset;
using g1_manipulation::approachAxis;
using g1_manipulation::approachTiltRad;

geometry_msgs::msg::Pose poseAt(double x, double y, double z, double roll, double pitch, double yaw)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    tf2::Quaternion rotation;
    rotation.setRPY(roll, pitch, yaw);
    pose.orientation = tf2::toMsg(rotation);
    return pose;
}

TEST(ApproachTilt, IsZeroForAGraspReachingStraightDown)
{
    // Rolled by pi, so the pose's own +z points at the floor: the generator's approach axis.
    EXPECT_NEAR(approachTiltRad(poseAt(0.3, -0.2, 0.9, M_PI, 0.0, 0.0)), 0.0, 1e-9);
    // Yaw does not change how the hand comes in, so it must not change the tilt either.
    EXPECT_NEAR(approachTiltRad(poseAt(0.3, -0.2, 0.9, M_PI, 0.0, 1.1)), 0.0, 1e-9);
}

TEST(ApproachTilt, IsARightAngleForAGraspComingInSideways)
{
    EXPECT_NEAR(approachTiltRad(poseAt(0.3, -0.2, 0.9, M_PI_2, 0.0, 0.0)), M_PI_2, 1e-9);
}

TEST(ApproachTilt, IsHalfATurnForAGraspFromUnderneath)
{
    // The identity pose's +z points at the ceiling, which on a table means through it.
    EXPECT_NEAR(approachTiltRad(poseAt(0.3, -0.2, 0.7, 0.0, 0.0, 0.0)), M_PI, 1e-9);
}

TEST(ApproachAxis, PointsWhereThePoseOwnZDoes)
{
    const std::array<double, 3> down = approachAxis(poseAt(0.0, 0.0, 0.0, M_PI, 0.0, 0.0));

    EXPECT_NEAR(down[0], 0.0, 1e-9);
    EXPECT_NEAR(down[1], 0.0, 1e-9);
    EXPECT_NEAR(down[2], -1.0, 1e-9);
}

TEST(GripperOffset, MovesTheGoalIntoTheGraspFrame)
{
    // An identity grasp at the origin, offset 1 cm along the gripper's own x.
    const geometry_msgs::msg::Pose out = applyGripperOffset(
        poseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0 },
        /*is_left=*/false);

    EXPECT_NEAR(out.position.x, 0.01, 1e-9);
    EXPECT_NEAR(out.position.y, 0.0, 1e-9);
    EXPECT_NEAR(out.position.z, 0.0, 1e-9);
}

TEST(GripperOffset, AppliesTheOffsetInTheGraspOwnFrame)
{
    // Yawed by a right angle, so the gripper's +x is the world's +y and the offset follows it.
    const geometry_msgs::msg::Pose out = applyGripperOffset(
        poseAt(0.3, -0.2, 0.9, 0.0, 0.0, M_PI_2),
        { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0 },
        /*is_left=*/false);

    EXPECT_NEAR(out.position.x, 0.3, 1e-9);
    EXPECT_NEAR(out.position.y, -0.19, 1e-9);
    EXPECT_NEAR(out.position.z, 0.9, 1e-9);
}

TEST(GripperOffset, MirrorsTheWayTheUrdfMirrorsTheTwoHands)
{
    const std::array<double, 6>    measured{ 0.010, 0.044, 0.009, 0.2, 0.0, 0.0 };
    const geometry_msgs::msg::Pose identity = poseAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    const geometry_msgs::msg::Pose right = applyGripperOffset(identity, measured, false);
    const geometry_msgs::msg::Pose left  = applyGripperOffset(identity, measured, true);

    EXPECT_NEAR(right.position.x, left.position.x, 1e-9);
    EXPECT_NEAR(right.position.y, -left.position.y, 1e-9);
    EXPECT_NEAR(right.position.z, -left.position.z, 1e-9);
    EXPECT_NEAR(right.orientation.x, -left.orientation.x, 1e-9);
}

}  // namespace
