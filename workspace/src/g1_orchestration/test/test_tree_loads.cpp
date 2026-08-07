/**
 * @file test_tree_loads.cpp
 * @brief Every shipped tree parses against the registered node set.
 *
 * The failure this catches is cheap to make and expensive to find: rename a leaf in C++ and
 * not in the XML, and nothing complains until a mission is launched against a real robot, at
 * which point the tree fails to load after the stack is already up.
 *
 * No ROS graph is needed. The leaves take a node to build their clients with, but building a
 * client neither discovers nor connects.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <gmock/gmock.h>

#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "g1_orchestration/arm_authority.hpp"
#include "g1_orchestration/skill_nodes.hpp"

namespace
{

BT::BehaviorTreeFactory makeFactory(const rclcpp::Node::SharedPtr& node)
{
    BT::BehaviorTreeFactory      factory;
    g1_orchestration::RosContext context{ node };
    g1_orchestration::registerSkillNodes(factory, context);
    g1_orchestration::registerAuthorityNodes(factory, context);
    return factory;
}

std::vector<std::filesystem::path> shippedTrees()
{
    std::vector<std::filesystem::path> trees;
    for (const auto& entry : std::filesystem::directory_iterator(G1_TREES_DIR))
    {
        if (entry.path().extension() == ".xml")
        {
            trees.push_back(entry.path());
        }
    }
    return trees;
}

}  // namespace

TEST(TreeLoads, EveryShippedTreeParses)
{
    auto       node  = std::make_shared<rclcpp::Node>("test_tree_loads");
    const auto trees = shippedTrees();
    ASSERT_FALSE(trees.empty()) << "no trees found in " << G1_TREES_DIR;

    for (const std::filesystem::path& tree_file : trees)
    {
        auto factory = makeFactory(node);
        EXPECT_NO_THROW({ BT::Tree tree = factory.createTreeFromFile(tree_file.string()); })
            << tree_file.filename();
    }
}

TEST(TreeLoads, TheMissionTreeUsesTheLeavesItIsSupposedTo)
{
    // Pinned because the mission's shape is the milestone: navigate, pick, navigate, place.
    // A tree that silently lost its Place would still parse.
    auto node    = std::make_shared<rclcpp::Node>("test_mission_shape");
    auto factory = makeFactory(node);

    BT::Tree tree = factory.createTreeFromFile(std::string(G1_TREES_DIR) + "/pick_and_place.xml");

    std::map<std::string, int> seen;
    for (const auto& subtree : tree.subtrees)
    {
        for (const auto& bt_node : subtree->nodes)
        {
            seen[bt_node->registrationName()]++;
        }
    }

    EXPECT_EQ(seen["NavigateToPose"], 2) << "one leg out, one leg back";
    EXPECT_EQ(seen["Pick"], 1);
    EXPECT_EQ(seen["Place"], 1);
    EXPECT_EQ(seen["AcquireArm"], 1);
    EXPECT_EQ(seen["ReleaseArm"], 1);
}

TEST(TreeLoads, RejectsALeafNobodyRegistered)
{
    // The compensating check for the one above: proves createTreeFromText really does fail on
    // an unknown node, so EXPECT_NO_THROW passing means something.
    auto node    = std::make_shared<rclcpp::Node>("test_unknown_leaf");
    auto factory = makeFactory(node);

    EXPECT_THROW(
        {
            factory.createTreeFromText(
                R"(<root BTCPP_format="4"><BehaviorTree ID="M">
                     <Sequence><NoSuchSkill/></Sequence>
                   </BehaviorTree></root>)");
        },
        BT::RuntimeError);
}

TEST(Ports, AStationParsesAsThreeNumbers)
{
    const auto station = BT::convertFromString<g1_orchestration::Station>("4.5;-4.5;1.57");
    EXPECT_DOUBLE_EQ(station.x, 4.5);
    EXPECT_DOUBLE_EQ(station.y, -4.5);
    EXPECT_DOUBLE_EQ(station.yaw, 1.57);

    // Rejected rather than silently zero-filled: a goal short one number would drive the base
    // somewhere nobody asked for.
    EXPECT_THROW(BT::convertFromString<g1_orchestration::Station>("4.5;-4.5"), BT::RuntimeError);
}

TEST(Ports, APointParsesAsThreeNumbers)
{
    const auto point = BT::convertFromString<g1_orchestration::Point3>("7.0;4.0;0.78");
    EXPECT_DOUBLE_EQ(point.x, 7.0);
    EXPECT_DOUBLE_EQ(point.y, 4.0);
    EXPECT_DOUBLE_EQ(point.z, 0.78);

    EXPECT_THROW(BT::convertFromString<g1_orchestration::Point3>("7.0;4.0"), BT::RuntimeError);
}

int main(int argc, char** argv)
{
    setenv("ROS_DOMAIN_ID", "79", 1);
    ::testing::InitGoogleMock(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
