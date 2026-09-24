/**
 * @file test_tree_loads.cpp
 * @brief Shipped trees parse against the registered leaves, and the mission keeps its shape.
 *
 * Needs no ROS graph: building a client neither discovers nor connects.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <gmock/gmock.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
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

// Trees only: skips the Groot2 palette, which is XML but not a tree.
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
    // The mission's shape is the contract; a tree that lost its Place would still parse.
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
    // Both stations are staging poses short of the surface, so every arrival needs an approach.
    EXPECT_EQ(seen["ApproachObject"], 2) << "one per surface: the workbench and the drop pad";
    EXPECT_EQ(seen["Retreat"], 2) << "turning in place beside a surface drags the arm across it";

    // Both arms tuck: either hand left hanging jams on the workbench edge.
    EXPECT_EQ(seen["SetArmPosture"], 4)
        << "two tucks out and two back: the object rides at the lift pose, with no carry posture";

    // The detector runs only at the benches: it shares the GPU with the sim camera, and would
    // track the carried object.
    EXPECT_EQ(seen["StopLooking"], 3) << "off for the walk out, for the carry, and at the end";

    // Manipulating beside a surface leaves phantom obstacles in the costmaps.
    EXPECT_EQ(seen["ClearCostmaps"], 4) << "after each manipulation, and before each nav goal";
}

TEST(TreeLoads, EveryFallibleLeafInTheMissionIsRetried)
{
    // Counted, not located, so a restructure needs no test change.
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

    // Four tucks, two navigation goals, the object approach, the pick, the approach-and-place
    // pair and the two LookFors.
    EXPECT_EQ(seen["RetryUntilSuccessful"], 11);
}

TEST(TreeLoads, RejectsALeafNobodyRegistered)
{
    // Proves an unknown leaf throws, so EveryShippedTreeParses passing means something.
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

    // A goal short one number must throw, not zero-fill.
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
    // No other thread exists yet.
    // NOLINTNEXTLINE(concurrency-mt-unsafe)
    setenv("ROS_DOMAIN_ID", "79", 1);
    ::testing::InitGoogleMock(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
