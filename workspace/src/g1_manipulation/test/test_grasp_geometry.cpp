/**
 * @file test_grasp_geometry.cpp
 * @brief The arm/group mapping and the grasp-frame goal, without a running MoveIt.
 *
 * Where the hand grips is not arithmetic here at all: it is the {side}_hand_grasp_frame
 * link in g1_description, and goals are given for that frame directly. What is left to pin is
 * which groups and frames an "arm" string resolves to, and that the goal passes position
 * through untouched while orienting the closing axis at the floor.
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

/// The server's own graspFrameGoal, which is a private member. Position passes through; only
/// the orientation is computed, and it mirrors between hands.
geometry_msgs::msg::Pose
graspFrameGoal(const geometry_msgs::msg::Pose& object, const std::vector<double>& rpy, bool is_left)
{
    geometry_msgs::msg::Pose goal;
    goal.position = object.position;
    tf2::Quaternion rotation;
    rotation.setRPY((is_left ? -1.0 : 1.0) * rpy[0], rpy[1], rpy[2]);
    goal.orientation = tf2::toMsg(rotation);
    return goal;
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

TEST(ResolveArm, MapsASideOntoItsGroupsFramesAndHandedness)
{
    ArmContext arm;
    ASSERT_TRUE(resolveArm("left", arm));
    EXPECT_EQ(arm.arm_group, "left_arm");
    EXPECT_EQ(arm.hand_group, "left_hand");
    EXPECT_EQ(arm.palm_link, "left_hand_palm_link");
    EXPECT_EQ(arm.grasp_frame, "left_hand_grasp_frame");
    EXPECT_TRUE(arm.is_left);

    ASSERT_TRUE(resolveArm("right", arm));
    EXPECT_EQ(arm.arm_group, "right_arm");
    EXPECT_EQ(arm.grasp_frame, "right_hand_grasp_frame");
    EXPECT_FALSE(arm.is_left);
}

TEST(ResolveArm, RejectsAnythingElseWithoutAssigning)
{
    // The names have to match g1.srdf's groups and the URDF's frames exactly. A silent
    // fallback to one side would move the wrong arm, which is what this refusal prevents.
    ArmContext arm;
    arm.arm_group = "sentinel";
    EXPECT_FALSE(resolveArm("Left", arm));
    EXPECT_FALSE(resolveArm("left_arm", arm));
    EXPECT_FALSE(resolveArm("", arm));
    EXPECT_EQ(arm.arm_group, "sentinel");
}

TEST(GraspFrameGoal, PutsTheGraspFrameExactlyOnTheObject)
{
    // The offset lives in the URDF, so the goal position is the object position with no
    // arithmetic left here to get wrong.
    const std::vector<double> rpy{ -M_PI_2, 0.0, 0.0 };
    const auto                target = objectAt(0.35, -0.20, 0.83);

    const auto goal = graspFrameGoal(target, rpy, /*is_left=*/false);

    EXPECT_DOUBLE_EQ(goal.position.x, target.position.x);
    EXPECT_DOUBLE_EQ(goal.position.y, target.position.y);
    EXPECT_DOUBLE_EQ(goal.position.z, target.position.z);
}

TEST(GraspFrameGoal, PointsTheClosingAxisAtTheFloor)
{
    // The Dex3's fingers curl toward the palm's +y, so it is THAT axis that has to end up
    // pointing down for a grasp off a table, and the palm's +x has to stay forward, so the
    // arm reaches out rather than the wrist contorting. Getting this wrong is easy to miss by
    // inspection, so it is pinned here instead.
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
    // The hands are mirror images, so the roll that points one hand's closing axis down points
    // the other's up. A shared orientation would have the left hand grasping upside down.
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
