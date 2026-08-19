/**
 * @file test_tree_loads.cpp
 * @brief Every shipped tree parses against the registered node set.
 *
 * Catches a leaf renamed in C++ but not in the XML, which otherwise fails to load only once a
 * mission is launched and the stack is already up.
 *
 * No ROS graph is needed: the leaves take a node to build their clients with, but building a
 * client neither discovers nor connects.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <gmock/gmock.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "g1_orchestration/skill_nodes.hpp"

namespace
{

BT::BehaviorTreeFactory makeFactory(const rclcpp::Node::SharedPtr& node)
{
    BT::BehaviorTreeFactory      factory;
    g1_orchestration::RosContext context{ node };
    g1_orchestration::registerSkillNodes(factory, context);
    return factory;
}

std::string readFile(const std::filesystem::path& path)
{
    std::ifstream in(path);
    return { std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>() };
}

// Trees only. trees/ also holds the Groot2 palette and project file, which are XML but must
// not be handed to createTreeFromFile.
std::vector<std::filesystem::path> shippedTrees()
{
    std::vector<std::filesystem::path> trees;
    for (const auto& entry : std::filesystem::directory_iterator(G1_TREES_DIR))
    {
        if (entry.path().extension() == ".xml" &&
            readFile(entry.path()).find("<BehaviorTree") != std::string::npos)
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
    // Pinned because the mission's shape is the contract: navigate, pick, navigate, place.
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
    // Both stations are staging poses roughly 0.7 m short of the surface, so every arrival must
    // be followed by an approach; Nav2 cannot honour a working pose directly.
    EXPECT_EQ(seen["ApproachObject"], 2) << "one per surface: the workbench and the drop pad";
    EXPECT_EQ(seen["Retreat"], 2) << "turning in place beside a surface drags the arm across it";

    // Both arms tuck, because a hanging hand sits 21 cm in front of the pelvis and 1 cm under
    // the workbench slab and jams on the table edge. Tucking one arm moves the collision to the
    // other.
    EXPECT_EQ(seen["SetArmPosture"], 5) << "two tucks out, one carry, two tucks back";

    // After manipulating beside a surface the costmaps hold the arm, the object and the surface
    // the base pressed against, none of it where the map says obstacles are.
    EXPECT_EQ(seen["ClearCostmaps"], 4) << "after each manipulation, and before each nav goal";
}

TEST(TreeLoads, EveryFallibleLeafInTheMissionIsRetried)
{
    // Counted rather than located, so a legitimate restructure is not a test change. Losing a
    // retry wrapper is invisible until one clipped waypoint fails an otherwise finished run.
    auto node    = std::make_shared<rclcpp::Node>("test_mission_retries");
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

    // Five postures, two navigation goals, the object approach, the pick and the
    // approach-and-place pair, each wrapped because Nav2 aborts plans transiently.
    EXPECT_EQ(seen["RetryUntilSuccessful"], 10);
}

TEST(TreeLoads, RejectsALeafNobodyRegistered)
{
    // The compensating check for the one above: proves createTreeFromText really does fail on
    // an unknown node, so EXPECT_NO_THROW passing means something.
    auto node    = std::make_shared<rclcpp::Node>("test_unknown_leaf");
    auto factory = makeFactory(node);

    EXPECT_THROW(
        {
            (void)factory.createTreeFromText(
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
    EXPECT_THROW(
        (void)BT::convertFromString<g1_orchestration::Station>("4.5;-4.5"),
        BT::RuntimeError);
}

TEST(Ports, APointParsesAsThreeNumbers)
{
    const auto point = BT::convertFromString<g1_orchestration::Point3>("7.0;4.0;0.78");
    EXPECT_DOUBLE_EQ(point.x, 7.0);
    EXPECT_DOUBLE_EQ(point.y, 4.0);
    EXPECT_DOUBLE_EQ(point.z, 0.78);

    EXPECT_THROW((void)BT::convertFromString<g1_orchestration::Point3>("7.0;4.0"), BT::RuntimeError);
}

int main(int argc, char** argv)
{
    // Before any node or thread exists, so the thread-safety this warns about does not apply.
    // NOLINTNEXTLINE(concurrency-mt-unsafe)
    setenv("ROS_DOMAIN_ID", "79", 1);
    ::testing::InitGoogleMock(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
