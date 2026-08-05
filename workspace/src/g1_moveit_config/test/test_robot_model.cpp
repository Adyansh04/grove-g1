/**
 * @file test_robot_model.cpp
 * @brief Loads the URDF and SRDF the way move_group does and asserts what the groups came out as.
 *
 * No simulator, no ROS graph. The SRDF is prose plus a generated block, and both are easy to
 * break in ways that only show up as a plan quietly failing much later.
 */

#include <gmock/gmock.h>

#include <fstream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

using ::testing::ElementsAreArray;

namespace
{
constexpr const char* kUrdfPath = G1_TEST_URDF_PATH;
constexpr const char* kSrdfPath = G1_TEST_SRDF_PATH;

const std::vector<std::string> kLeftArm = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint",          "left_wrist_roll_joint",    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
};

const std::vector<std::string> kRightArm = {
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint",          "right_wrist_roll_joint",    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
};

/// The links that can actually reach each other. Their cross-arm collision pairs are the whole
/// reason the both_arms group exists, so they must never end up disabled.
const std::vector<std::string> kReachingLinks = {
    "elbow_link", "wrist_roll_link", "wrist_pitch_link", "wrist_yaw_link", "hand_palm_link",
};

class RobotModelTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        auto urdf = urdf::parseURDFFile(kUrdfPath);
        ASSERT_TRUE(urdf) << "could not parse " << kUrdfPath;

        srdf_ = std::make_shared<srdf::Model>();
        ASSERT_TRUE(srdf_->initFile(*urdf, kSrdfPath)) << "could not parse " << kSrdfPath;

        model_ = std::make_shared<moveit::core::RobotModel>(urdf, srdf_);
        ASSERT_TRUE(model_);
    }

    std::shared_ptr<srdf::Model>                  srdf_;
    std::shared_ptr<moveit::core::RobotModel>     model_;
};

TEST_F(RobotModelTest, PlansInThePelvisFrame)
{
    // The vendored URDF's floating_base_joint is commented out and g1.srdf declares no virtual
    // joint, so the model root is the pelvis. Pinned because adding a virtual joint later moves
    // every pose goal without any other visible change.
    EXPECT_EQ(model_->getModelFrame(), "pelvis");
}

TEST_F(RobotModelTest, EachArmIsSevenJointsInOrder)
{
    const auto* left = model_->getJointModelGroup("left_arm");
    const auto* right = model_->getJointModelGroup("right_arm");
    ASSERT_NE(left, nullptr);
    ASSERT_NE(right, nullptr);
    EXPECT_THAT(left->getActiveJointModelNames(), ElementsAreArray(kLeftArm));
    EXPECT_THAT(right->getActiveJointModelNames(), ElementsAreArray(kRightArm));
}

TEST_F(RobotModelTest, TheDualArmGroupIsBothArmsTogether)
{
    const auto* both = model_->getJointModelGroup("both_arms");
    ASSERT_NE(both, nullptr);

    std::vector<std::string> expected = kLeftArm;
    expected.insert(expected.end(), kRightArm.begin(), kRightArm.end());
    // Order is the URDF's depth-first joint order, which puts the left arm first. Asserted
    // rather than assumed: a trajectory is matched to the controller by name, but anything
    // reading joint values positionally depends on this.
    EXPECT_THAT(both->getActiveJointModelNames(), ElementsAreArray(expected));
}

TEST_F(RobotModelTest, TheDualArmGroupIsNotAChainAndHasBothArmsAsSubgroups)
{
    const auto* both = model_->getJointModelGroup("both_arms");
    ASSERT_NE(both, nullptr);

    // This is the root reason both_arms must stay out of kinematics.yaml: every chain solver,
    // pick_ik included, refuses a group that is not a chain, and MoveIt only builds the
    // per-subgroup solver map for groups that have no solver of their own.
    EXPECT_FALSE(both->isChain()) << "both_arms is a chain, so the subgroup IK story is wrong";

    // The map is keyed on subgroups, so there have to be exactly the two arms to key it on.
    // Whether the map itself was populated needs a plugin loader and a live node, so it is
    // asserted in test_moveit_config_drift (the YAML rule) and against a running move_group.
    std::vector<const moveit::core::JointModelGroup*> sub_groups;
    both->getSubgroups(sub_groups);
    std::set<std::string> subgroups;
    for (const auto* subgroup : sub_groups)
    {
        subgroups.insert(subgroup->getName());
    }
    EXPECT_EQ(subgroups, (std::set<std::string>{ "left_arm", "right_arm" }));
}

TEST_F(RobotModelTest, NoArmGroupCommandsTheHand)
{
    // The hand is a separate controllable group with its own API (docs/CONTROL_MODES.md), and
    // nothing in this stack can command a finger. A group that reached into one would plan
    // motion that silently never executes.
    for (const auto* name : { "left_arm", "right_arm", "both_arms" })
    {
        const auto* group = model_->getJointModelGroup(name);
        ASSERT_NE(group, nullptr) << name;
        for (const auto& joint : group->getActiveJointModelNames())
        {
            EXPECT_EQ(joint.find("_hand_"), std::string::npos)
                << name << " contains the hand joint " << joint;
        }
    }
}

TEST_F(RobotModelTest, ArmsAreRootedAtTheTorsoNotThePelvis)
{
    // Spanning the waist would plan motion the onboard controller owns.
    for (const auto* name : { "left_arm", "right_arm" })
    {
        const auto* group = model_->getJointModelGroup(name);
        ASSERT_NE(group, nullptr) << name;
        for (const auto& joint : group->getActiveJointModelNames())
        {
            EXPECT_EQ(joint.find("waist"), std::string::npos)
                << name << " spans the waist joint " << joint;
        }
    }
}

TEST_F(RobotModelTest, TheCollisionMatrixExists)
{
    // Deliberately a conservative matrix: adjacent pairs plus what touches at rest, and nothing
    // found by random sampling. Enough that the robot is not in collision before it moves, which
    // is what RRTConnect needs to seed. The bound is a floor, not a target.
    EXPECT_GT(srdf_->getDisabledCollisionPairs().size(), 40u)
        << "g1.srdf carries no generated collision matrix; see the package README";
}

TEST_F(RobotModelTest, AdjacentLinksAreDisabled)
{
    std::set<std::pair<std::string, std::string>> disabled;
    for (const auto& pair : srdf_->getDisabledCollisionPairs())
    {
        disabled.insert({ pair.link1_, pair.link2_ });
        disabled.insert({ pair.link2_, pair.link1_ });
    }

    for (const auto& [name, joint] : model_->getURDF()->joints_)
    {
        (void)name;
        const auto& parent = joint->parent_link_name;
        const auto& child = joint->child_link_name;
        if (parent.empty() || child.empty())
        {
            continue;
        }
        // Links joined by a joint touch by construction. This is the category a careless hand
        // edit drops, and dropping it makes every plan start in collision.
        EXPECT_TRUE(disabled.count({ parent, child }) > 0)
            << "adjacent pair " << parent << " / " << child << " is not disabled";
    }
}

TEST_F(RobotModelTest, TheArmsCanStillCollideWithEachOther)
{
    // The safety property that makes both_arms worth having. Proximal cross-arm pairs are
    // legitimately disabled (two shoulders cannot reach each other), but anything from the
    // elbow out must stay checked, or a coordinated plan can drive the hands through one
    // another and report success.
    for (const auto& pair : srdf_->getDisabledCollisionPairs())
    {
        const bool left_then_right =
            pair.link1_.rfind("left_", 0) == 0 && pair.link2_.rfind("right_", 0) == 0;
        const bool right_then_left =
            pair.link1_.rfind("right_", 0) == 0 && pair.link2_.rfind("left_", 0) == 0;
        if (!left_then_right && !right_then_left)
        {
            continue;
        }
        for (const auto& reaching : kReachingLinks)
        {
            const bool one = pair.link1_.find(reaching) != std::string::npos;
            const bool two = pair.link2_.find(reaching) != std::string::npos;
            EXPECT_FALSE(one && two)
                << "cross-arm collision disabled between " << pair.link1_ << " and "
                << pair.link2_ << ", which dual-arm planning depends on";
        }
    }
}
}  // namespace
