/**
 * @file test_robot_model.cpp
 * @brief Loads the URDF and SRDF the way move_group does and checks the resulting model.
 */

#include <gmock/gmock.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

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

/// Links that can reach the other arm; their cross-arm pairs must stay enabled for both_arms.
const std::vector<std::string> kReachingLinks = {
    "elbow_link", "wrist_roll_link", "wrist_pitch_link", "wrist_yaw_link", "hand_palm_link",
};

/// Room every joint of a named posture needs before self-collision: an arm drooping under load
/// into a colliding start state cannot be planned out of.
constexpr double kPostureMarginRad = 0.20;
constexpr double kMarginStepRad    = 0.02;

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

    /// Smallest distance any single joint of `group` can move out of `posture` before the state
    /// self-collides, capped at `cap` so an unbounded axis does not sweep the whole range.
    double postureMargin(
        const std::string& group, const std::string& posture, std::string& tightest,
        double cap = 0.30) const
    {
        planning_scene::PlanningScene scene(model_);
        const auto*                   jmg = model_->getJointModelGroup(group);
        moveit::core::RobotState      state(model_);
        state.setToDefaultValues();
        EXPECT_TRUE(state.setToDefaultValues(jmg, posture))
            << group << " has no named posture '" << posture << "'";
        state.update();

        std::vector<double> base;
        state.copyJointGroupPositions(jmg, base);
        double worst = cap;
        for (std::size_t i = 0; i < base.size(); ++i)
        {
            for (const double direction : { -1.0, 1.0 })
            {
                for (int step = 1; step * kMarginStepRad <= cap + 1e-9; ++step)
                {
                    const double        delta = step * kMarginStepRad;
                    std::vector<double> probe = base;
                    probe[i] += direction * delta;
                    moveit::core::RobotState moved(state);
                    moved.setJointGroupPositions(jmg, probe);
                    moved.update();
                    // Joint limits are the model's business, not this test's: a posture backed
                    // against a limit is reported by satisfiesBounds, not by a collision.
                    if (!moved.satisfiesBounds(jmg))
                    {
                        break;
                    }
                    if (scene.isStateColliding(moved, group))
                    {
                        if (delta < worst)
                        {
                            worst    = delta;
                            tightest = jmg->getActiveJointModelNames()[i];
                        }
                        break;
                    }
                }
            }
        }
        return worst;
    }

    std::shared_ptr<srdf::Model>              srdf_;
    std::shared_ptr<moveit::core::RobotModel> model_;
};

TEST_F(RobotModelTest, NamedPosturesKeepRoomBeforeSelfCollision)
{
    for (const std::string& group : { "left_arm", "right_arm" })
    {
        for (const std::string& posture : { "tucked", "carry" })
        {
            std::string  tightest = "(none)";
            const double margin   = postureMargin(group, posture, tightest);
            std::cout << "  " << group << "/" << posture << ": " << margin << " rad on " << tightest
                      << "\n";
            EXPECT_GE(margin, kPostureMarginRad)
                << group << "/" << posture << " has only " << margin << " rad of room on "
                << tightest
                << ". A posture this close to a self-collision deadlocks every later plan when "
                   "the arm droops into it.";
        }
    }
}

TEST_F(RobotModelTest, PlansInThePelvisFrame)
{
    // No floating base and no virtual joint. Pinned because adding one moves every pose goal
    // without any other visible change.
    EXPECT_EQ(model_->getModelFrame(), "pelvis");
}

TEST_F(RobotModelTest, EachArmIsSevenJointsInOrder)
{
    const auto* left  = model_->getJointModelGroup("left_arm");
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
    // The URDF's depth-first order, left arm first. Trajectories match by name, but anything
    // reading joint values positionally depends on it.
    EXPECT_THAT(both->getActiveJointModelNames(), ElementsAreArray(expected));
}

TEST_F(RobotModelTest, TheDualArmGroupIsNotAChainAndHasBothArmsAsSubgroups)
{
    const auto* both = model_->getJointModelGroup("both_arms");
    ASSERT_NE(both, nullptr);

    // Why both_arms stays out of kinematics.yaml: chain solvers refuse it, and MoveIt builds the
    // per-subgroup solver map only for groups without a solver of their own.
    EXPECT_FALSE(both->isChain()) << "both_arms is a chain, so the subgroup IK story is wrong";

    // The solver map is keyed on these subgroups; test_moveit_config_drift checks the YAML rule.
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
    // The hand has its own group and controller; an arm group reaching into it would plan a
    // trajectory no single controller executes.
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
    // The waist belongs to waist_freeze_controller and the balance policy.
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

TEST_F(RobotModelTest, TheNamedPosesAgreeAcrossTheThreeGroups)
{
    // SRDF group states belong to one group, so each pose is written three times. A skill that
    // sends left_arm to "ready" and one that sends both_arms there must reach the same place.
    const auto& states = srdf_->getGroupStates();
    ASSERT_FALSE(states.empty()) << "no named poses in g1.srdf";

    const std::set<std::string> arm_groups = { "left_arm", "right_arm", "both_arms" };

    std::map<std::string, std::map<std::string, double>> by_pose;
    std::map<std::string, int>                           group_count;
    for (const auto& state : states)
    {
        if (!arm_groups.contains(state.group_))
        {
            continue;  // the hand postures are two groups, not three; see below
        }
        group_count[state.name_]++;
        for (const auto& [joint, values] : state.joint_values_)
        {
            ASSERT_EQ(values.size(), 1U) << joint << " in " << state.name_;
            auto [it, fresh] = by_pose[state.name_].emplace(joint, values.front());
            if (!fresh)
            {
                EXPECT_DOUBLE_EQ(it->second, values.front())
                    << "pose '" << state.name_ << "' sets " << joint << " differently in group '"
                    << state.group_ << "' than in another group -- the copies have drifted";
            }
        }
    }
    for (const auto& [pose, count] : group_count)
    {
        EXPECT_EQ(count, 3) << "pose '" << pose << "' should exist for left_arm, right_arm and "
                            << "both_arms";
        EXPECT_EQ(by_pose[pose].size(), 14U) << "pose '" << pose << "' should name all 14 joints";
    }
}

TEST_F(RobotModelTest, EachHandIsExactlyItsSevenFingerJoints)
{
    // A set: MoveIt sorts a joint-list group. The wire order is pinned by g1_description's xacro
    // test and test_moveit_config_drift.
    for (const auto* side : { "left", "right" })
    {
        const auto* hand = model_->getJointModelGroup(std::string(side) + "_hand");
        ASSERT_NE(hand, nullptr) << side;

        std::set<std::string> expected;
        for (const auto* suffix :
             { "thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1" })
        {
            expected.insert(std::string(side) + "_hand_" + suffix + "_joint");
        }
        const auto& actual = hand->getActiveJointModelNames();
        EXPECT_EQ(std::set<std::string>(actual.begin(), actual.end()), expected);
    }
}

TEST_F(RobotModelTest, EachHandIsItsArmsEndEffector)
{
    // Lets attachObject derive its touch links and RViz offer the hand as the arm's gripper.
    std::map<std::string, srdf::Model::EndEffector> by_group;
    for (const auto& effector : srdf_->getEndEffectors())
    {
        by_group.emplace(effector.component_group_, effector);
    }
    ASSERT_EQ(by_group.size(), 2U) << "expected one end effector per hand";

    for (const auto* side : { "left", "right" })
    {
        const auto it = by_group.find(std::string(side) + "_hand");
        ASSERT_NE(it, by_group.end()) << side;
        EXPECT_EQ(it->second.parent_link_, std::string(side) + "_hand_palm_link");
        EXPECT_EQ(it->second.parent_group_, std::string(side) + "_arm");
    }
}

TEST_F(RobotModelTest, EachHandHasAnOpenAndAClosedPosture)
{
    // Each must name all seven joints, or the finger left out stays wherever it happened to be.
    std::map<std::string, std::set<std::string>> poses_by_group;
    for (const auto& state : srdf_->getGroupStates())
    {
        if (state.group_ == "left_hand" || state.group_ == "right_hand")
        {
            poses_by_group[state.group_].insert(state.name_);
            EXPECT_EQ(state.joint_values_.size(), 7U)
                << state.group_ << " posture '" << state.name_ << "' does not cover the hand";
        }
    }
    for (const auto* side : { "left", "right" })
    {
        EXPECT_EQ(
            poses_by_group[std::string(side) + "_hand"],
            (std::set<std::string>{ "open", "closed", "pinch_ready" }));
    }
}

TEST_F(RobotModelTest, TheNamedPosesAreWithinJointLimits)
{
    // A named pose outside the limits fails every plan request that targets it.
    const auto* both = model_->getJointModelGroup("both_arms");
    ASSERT_NE(both, nullptr);
    moveit::core::RobotState state(model_);
    for (const auto& srdf_state : srdf_->getGroupStates())
    {
        if (srdf_state.group_ != "both_arms")
        {
            continue;
        }
        state.setToDefaultValues();
        for (const auto& [joint, values] : srdf_state.joint_values_)
        {
            state.setJointPositions(joint, values);
        }
        EXPECT_TRUE(state.satisfiesBounds(both))
            << "named pose '" << srdf_state.name_ << "' is outside the joint limits";
    }
}

TEST_F(RobotModelTest, TheCollisionMatrixExists)
{
    // Adjacent pairs plus what touches at rest, enough for RRTConnect to seed from a start state
    // that is not in collision. The bound is a floor, not a target.
    EXPECT_GT(srdf_->getDisabledCollisionPairs().size(), 40U)
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
        const auto& child  = joint->child_link_name;
        if (parent.empty() || child.empty())
        {
            continue;
        }
        // Links without collision shapes, like the visual-only sensor bodies, are never checked.
        const auto* parent_link = model_->getLinkModel(parent);
        const auto* child_link  = model_->getLinkModel(child);
        if (parent_link == nullptr || child_link == nullptr || parent_link->getShapes().empty() ||
            child_link->getShapes().empty())
        {
            continue;
        }
        // Links joined by a joint touch by construction; dropping one makes every plan start in
        // collision.
        EXPECT_TRUE(disabled.contains({ parent, child }))
            << "adjacent pair " << parent << " / " << child << " is not disabled";
    }
}

TEST_F(RobotModelTest, TheArmsCanStillCollideWithEachOther)
{
    // Anything from the elbow out must stay checked across arms, or a both_arms plan can drive
    // the hands through one another and report success.
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
                << "cross-arm collision disabled between " << pair.link1_ << " and " << pair.link2_
                << ", which dual-arm planning depends on";
        }
    }
}
}  // namespace
