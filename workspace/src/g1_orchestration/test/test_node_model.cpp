/**
 * @file test_node_model.cpp
 * @brief The checked-in Groot2 palette still matches what the code registers.
 *
 * Groot2 offers a tree author whatever the model file says, so a stale one invites trees that
 * cannot load: a port renamed in C++ would keep appearing in the editor under its old name.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <gmock/gmock.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "g1_orchestration/skill_nodes.hpp"

TEST(NodeModel, TheCheckedInPaletteMatchesTheRegisteredNodes)
{
    auto                         node = std::make_shared<rclcpp::Node>("test_node_model");
    BT::BehaviorTreeFactory      factory;
    g1_orchestration::RosContext context{ node };
    g1_orchestration::registerSkillNodes(factory, context);

    const std::filesystem::path model_file =
        std::filesystem::path(G1_TREES_DIR) / "g1_orchestration_nodes.xml";
    ASSERT_TRUE(std::filesystem::exists(model_file)) << model_file;

    std::ifstream     in(model_file);
    const std::string on_disk{ std::istreambuf_iterator<char>(in),
                               std::istreambuf_iterator<char>() };

    EXPECT_EQ(on_disk, g1_orchestration::nodeModelXml(factory))
        << "the Groot2 palette has drifted. Regenerate it:\n"
        << "  ros2 run g1_orchestration g1_bt_node_model "
        << "src/g1_orchestration/trees/g1_orchestration_nodes.xml";
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
