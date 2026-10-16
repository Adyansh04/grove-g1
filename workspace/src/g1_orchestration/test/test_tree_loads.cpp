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
