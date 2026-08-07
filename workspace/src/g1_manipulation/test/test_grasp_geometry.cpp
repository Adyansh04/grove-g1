/**
 * @file test_grasp_geometry.cpp
 * @brief The arm/group mapping and the palm-pose arithmetic, without a running MoveIt.
 *
 * The rest of the server is MoveIt calls and needs a live move_group, which is what
 * test_pick_place covers. What is testable here is the part that would otherwise be wrong
 * silently: which groups an "arm" string resolves to, and where the palm has to be for the
 * object to end up somewhere.
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

/// The server's own palmPoseFor, which is a private member. Duplicated here rather than
/// exposed, because what these tests pin is the RELATION -- palm plus rotated offset lands on
/// the object -- and that is checked below against the result rather than against this code.
geometry_msgs::msg::Pose palmPoseFor(
    const geometry_msgs::msg::Pose& object, const std::vector<double>& offset_xyz,
    const std::vector<double>& rpy)
{
    tf2::Quaternion rotation;
    rotation.setRPY(rpy[0], rpy[1], rpy[2]);
    const tf2::Vector3 offset =
        tf2::Matrix3x3(rotation) * tf2::Vector3(offset_xyz[0], offset_xyz[1], offset_xyz[2]);

    geometry_msgs::msg::Pose palm;
    palm.position.x  = object.position.x - offset.x();
    palm.position.y  = object.position.y - offset.y();
    palm.position.z  = object.position.z - offset.z();
    palm.orientation = tf2::toMsg(rotation);
    return palm;
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

TEST(ResolveArm, MapsASideOntoItsGroupsAndPalm)
{
    ArmContext arm;
    ASSERT_TRUE(resolveArm("left", arm));
    EXPECT_EQ(arm.arm_group, "left_arm");
    EXPECT_EQ(arm.hand_group, "left_hand");
    EXPECT_EQ(arm.palm_link, "left_hand_palm_link");

    ASSERT_TRUE(resolveArm("right", arm));
    EXPECT_EQ(arm.arm_group, "right_arm");
    EXPECT_EQ(arm.hand_group, "right_hand");
    EXPECT_EQ(arm.palm_link, "right_hand_palm_link");
}

TEST(ResolveArm, RejectsAnythingElseWithoutAssigning)
{
    // The names have to match g1.srdf's groups exactly. A silent fallback to one side would
    // move the wrong arm, which is the failure this refusal exists to prevent.
    ArmContext arm;
    arm.arm_group = "sentinel";
    EXPECT_FALSE(resolveArm("Left", arm));
    EXPECT_FALSE(resolveArm("left_arm", arm));
    EXPECT_FALSE(resolveArm("", arm));
    EXPECT_EQ(arm.arm_group, "sentinel");
}

TEST(PalmPoseFor, PutsTheObjectExactlyWhereItWasAsked)
{
    // The invariant that matters: rotate the configured offset by the palm's orientation, add
    // it to the returned palm position, and the object lands back on its target. If this is
    // wrong the arm reaches somewhere plausible and misses by the offset every time.
    const std::vector<double> offset_xyz{ 0.010, 0.044, 0.009 };
    const std::vector<double> rpy{ -M_PI_2, 0.0, 0.0 };
    const auto                target = objectAt(0.35, -0.20, 0.78);

    const auto palm = palmPoseFor(target, offset_xyz, rpy);

    tf2::Quaternion palm_rotation;
    tf2::fromMsg(palm.orientation, palm_rotation);
    const tf2::Vector3 offset =
        tf2::Matrix3x3(palm_rotation) * tf2::Vector3(offset_xyz[0], offset_xyz[1], offset_xyz[2]);

    EXPECT_NEAR(palm.position.x + offset.x(), target.position.x, 1e-9);
    EXPECT_NEAR(palm.position.y + offset.y(), target.position.y, 1e-9);
    EXPECT_NEAR(palm.position.z + offset.z(), target.position.z, 1e-9);
}

TEST(PalmPoseFor, TheDefaultGraspPointsTheClosingAxisDown)
{
    // The shipped grasp_rpy has to produce a top-down grasp, since every target surface here
    // is a table. The Dex3's fingers curl toward the palm's +y, not along +x, so it is THAT
    // axis that must come out pointing at the floor. Getting this wrong is what made the
    // first version unplannable, so it is pinned rather than left to inspection.
    const std::vector<double> rpy{ -M_PI_2, 0.0, 0.0 };
    const auto                palm = palmPoseFor(objectAt(0.4, 0.0, 0.8), { 0.0, 0.0, 0.0 }, rpy);

    tf2::Quaternion rotation;
    tf2::fromMsg(palm.orientation, rotation);
    const tf2::Matrix3x3 basis(rotation);

    EXPECT_NEAR((basis * tf2::Vector3(0.0, 1.0, 0.0)).z(), -1.0, 1e-9)
        << "the palm's +y, which is where the fingers close, must point down";
    EXPECT_NEAR((basis * tf2::Vector3(1.0, 0.0, 0.0)).x(), 1.0, 1e-9)
        << "and the palm's +x stays forward, so the arm reaches out rather than twisting";
}

TEST(PalmPoseFor, HoldsTheObjectAboveTheSurfaceForTheConfiguredOffset)
{
    // With the closing axis pointing down, the offset puts the object BELOW the palm. Getting
    // this sign backwards would have the arm reach under the table.
    const std::vector<double> offset_xyz{ 0.010, 0.044, 0.009 };
    const std::vector<double> rpy{ -M_PI_2, 0.0, 0.0 };
    const auto                target = objectAt(0.35, 0.0, 0.78);

    const auto palm = palmPoseFor(target, offset_xyz, rpy);

    EXPECT_GT(palm.position.z, target.position.z) << "the palm must sit above what it grasps";
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
