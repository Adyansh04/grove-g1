/**
 * @file test_grasp_filter.cpp
 * @brief The generated-grasp arithmetic, where a mistake puts the hand somewhere plausible and
 *        wrong.
 */

#include <gmock/gmock.h>

#include <array>
#include <cmath>
#include <numbers>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "g1_manipulation/grasp_filter.hpp"

namespace
{

using g1_manipulation::applyGripperOffset;
using g1_manipulation::approachAxis;
using g1_manipulation::approachTiltRad;
using g1_manipulation::mirroredGripperOffset;

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

TEST(GripperOffset, AppliesTheOffsetInTheGraspOwnFrame)
{
    // Yawed by a right angle, so the gripper's +x is the world's +y and the offset follows it.
    const geometry_msgs::msg::Pose out = applyGripperOffset(
        poseAt(0.3, -0.2, 0.9, 0.0, 0.0, M_PI_2),
        { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0 });

    EXPECT_NEAR(out.position.x, 0.3, 1e-9);
    EXPECT_NEAR(out.position.y, -0.19, 1e-9);
    EXPECT_NEAR(out.position.z, 0.9, 1e-9);
}

// GraspGenX's `inspire_hand` frame to the right grasp frame, as g1_manipulation_server_inspire.yaml.
constexpr std::array<double, 6> kInspireOffset = {
    0.0007, 0.012, 0.1619, std::numbers::pi / 2, -std::numbers::pi / 4, -std::numbers::pi
};

// A pose reflected through the y-z plane, then made right-handed again by reversing its own axis
// `own_axis` (0 x, 1 y, 2 z).
geometry_msgs::msg::Pose reflected(const geometry_msgs::msg::Pose& pose, int own_axis)
{
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    const tf2::Matrix3x3 r(q);
    tf2::Matrix3x3       out;
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            out[row][col] = r[row][col] * (row == 0 ? -1.0 : 1.0) * (col == own_axis ? -1.0 : 1.0);
        }
    }
    geometry_msgs::msg::Pose result = pose;
    result.position.x               = -pose.position.x;
    tf2::Quaternion mirrored;
    out.getRotation(mirrored);
    result.orientation = tf2::toMsg(mirrored);
    return result;
}

void expectSamePose(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b)
{
    EXPECT_NEAR(a.position.x, b.position.x, 1e-9);
    EXPECT_NEAR(a.position.y, b.position.y, 1e-9);
    EXPECT_NEAR(a.position.z, b.position.z, 1e-9);
    tf2::Quaternion qa;
    tf2::Quaternion qb;
    tf2::fromMsg(a.orientation, qa);
    tf2::fromMsg(b.orientation, qb);
    // Equal rotations: q and -q are the same one.
    EXPECT_NEAR(std::abs(qa.dot(qb)), 1.0, 1e-9);
}

TEST(GripperOffset, AMirroredGraspPutsTheOtherHandWhereTheMirrorImageIs)
{
    // The adapter's answer for the other hand is a grasp found in the mirrored scene, reflected
    // back with the gripper's own x reversed. The other hand's grasp frame must land on the mirror
    // image of the first hand's, its y reversed: the G1's hands mirror through their x-z planes.
    const std::array<double, 6> offset = kInspireOffset;
    for (const geometry_msgs::msg::Pose& found : { poseAt(0.3, -0.2, 0.9, M_PI, 0.0, 0.0),
                                                   poseAt(0.1, 0.25, 0.6, 2.4, -0.3, 1.1),
                                                   poseAt(-0.2, 0.05, 0.75, -1.0, 0.7, -2.6) })
    {
        const geometry_msgs::msg::Pose answered = reflected(found, 0);
        const geometry_msgs::msg::Pose expected = reflected(applyGripperOffset(found, offset), 1);

        expectSamePose(applyGripperOffset(answered, mirroredGripperOffset(offset)), expected);
    }
}

TEST(GripperOffset, MirroringTwiceGivesTheOffsetBack)
{
    const std::array<double, 6>    offset = kInspireOffset;
    const geometry_msgs::msg::Pose at     = poseAt(0.2, -0.1, 0.8, 0.4, 0.2, -0.9);

    expectSamePose(
        applyGripperOffset(at, mirroredGripperOffset(mirroredGripperOffset(offset))),
        applyGripperOffset(at, offset));
}

}  // namespace
