/**
 * @file test_action_leaf.cpp
 * @brief The action-leaf base against a real server, and LookFor against a stand-in detector.
 *
 * Driven the way g1_bt_executor runs them: the tree on this thread, the executor on another.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <g1_msgs/action/retreat.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_orchestration/skill_nodes.hpp"

namespace
{

using Retreat    = g1_msgs::action::Retreat;
using GoalHandle = rclcpp_action::ServerGoalHandle<Retreat>;

constexpr const char* kRetreatAction = "/g1_base_approach/retreat";

/// How long an accepted goal stays in flight. Non-zero, so the leaf ticks at least once with the
/// goal accepted but unfinished.
constexpr auto kGoalDuration = std::chrono::milliseconds(300);

/// Answers one goal the way the test asked, and finishes it on a timer.
class FakeRetreatServer
{
public:
    FakeRetreatServer(rclcpp::Node::SharedPtr node, bool accept)
      : node_(std::move(node))
    {
        server_ = rclcpp_action::create_server<Retreat>(
            node_,
            kRetreatAction,
            [accept](const rclcpp_action::GoalUUID&, const std::shared_ptr<const Retreat::Goal>&) {
                return accept ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
                                rclcpp_action::GoalResponse::REJECT;
            },
            [](const std::shared_ptr<GoalHandle>&) { return rclcpp_action::CancelResponse::ACCEPT; },
            [this](const std::shared_ptr<GoalHandle>& handle) {
                pending_ = handle;
                finish_  = node_->create_wall_timer(kGoalDuration, [this] {
                    finish_->cancel();
                    auto result     = std::make_shared<Retreat::Result>();
                    result->success = true;
                    result->message = "reversed";
                    pending_->succeed(result);
                });
            });
    }

private:
    rclcpp::Node::SharedPtr                   node_;
    rclcpp_action::Server<Retreat>::SharedPtr server_;
    std::shared_ptr<GoalHandle>               pending_;
    rclcpp::TimerBase::SharedPtr              finish_;
};

/// Ticks a one-leaf tree to completion, or gives up and returns RUNNING.
BT::NodeStatus runRetreatLeaf(bool server_accepts)
{
    auto              server_node = std::make_shared<rclcpp::Node>("test_action_leaf_server");
    auto              tree_node   = std::make_shared<rclcpp::Node>("test_action_leaf_client");
    FakeRetreatServer server(server_node, server_accepts);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server_node);
    executor.add_node(tree_node);
    std::atomic<bool> stop{ false };
    std::thread       spinner([&executor, &stop] {
        while (rclcpp::ok() && !stop)
        {
            executor.spin_once(std::chrono::milliseconds(10));
        }
    });

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    {
        BT::BehaviorTreeFactory      factory;
        g1_orchestration::RosContext context{ tree_node };
        g1_orchestration::registerSkillNodes(factory, context);
        BT::Tree tree = factory.createTreeFromText(
            R"(<root BTCPP_format="4"><BehaviorTree ID="M">
                 <Retreat distance="0.5" server_timeout_s="20.0"/>
               </BehaviorTree></root>)");

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
        {
            status = tree.tickOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    stop = true;
    spinner.join();
    executor.remove_node(tree_node);
    executor.remove_node(server_node);
    return status;
}

/// What LookFor writes to a detector's `phrases`, read back off a node standing in for one.
std::vector<std::string> phrasesLookForWrites(const std::string& objects, const std::string& also)
{
    auto detector  = std::make_shared<rclcpp::Node>("test_fake_detector");
    auto tree_node = std::make_shared<rclcpp::Node>("test_look_for_client");
    detector->declare_parameter<std::vector<std::string>>("phrases", std::vector<std::string>{});

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(detector);
    std::atomic<bool> stop{ false };
    std::thread       spinner([&executor, &stop] {
        while (rclcpp::ok() && !stop)
        {
            executor.spin_once(std::chrono::milliseconds(10));
        }
    });

    {
        BT::BehaviorTreeFactory      factory;
        g1_orchestration::RosContext context{ tree_node };
        g1_orchestration::registerSkillNodes(factory, context);
        // Nothing publishes /objects, so the leaf fails after the write, which is what is tested.
        BT::Tree tree = factory.createTreeFromText(
            R"(<root BTCPP_format="4"><BehaviorTree ID="M">
                 <LookFor objects=")" +
            objects + R"(" also=")" + also +
            R"(" detector="/test_fake_detector" timeout_s="1.0"/>
               </BehaviorTree></root>)");
        tree.tickWhileRunning();
    }

    stop = true;
    spinner.join();
    executor.remove_node(detector);
    return detector->get_parameter("phrases").as_string_array();
}

/// Runs LookFor for @p objects while a stand-in detector publishes one object under @p id.
BT::NodeStatus lookForWhilePublishing(const std::string& objects, const std::string& id)
{
    auto detector  = std::make_shared<rclcpp::Node>("test_fake_detector");
    auto tree_node = std::make_shared<rclcpp::Node>("test_look_for_client");
    detector->declare_parameter<std::vector<std::string>>("phrases", std::vector<std::string>{});
    auto objects_pub = detector->create_publisher<vision_msgs::msg::Detection3DArray>(
        "/objects",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    auto timer = detector->create_wall_timer(std::chrono::milliseconds(100), [&objects_pub, &id] {
        vision_msgs::msg::Detection3DArray msg;
        msg.detections.emplace_back().id = id;
        objects_pub->publish(msg);
    });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(detector);
    std::atomic<bool> stop{ false };
    std::thread       spinner([&executor, &stop] {
        while (rclcpp::ok() && !stop)
        {
            executor.spin_once(std::chrono::milliseconds(10));
        }
    });

    BT::NodeStatus status = BT::NodeStatus::IDLE;
    {
        BT::BehaviorTreeFactory      factory;
        g1_orchestration::RosContext context{ tree_node };
        g1_orchestration::registerSkillNodes(factory, context);
        BT::Tree tree = factory.createTreeFromText(
            R"(<root BTCPP_format="4"><BehaviorTree ID="M">
                 <LookFor objects=")" +
            objects + R"(" detector="/test_fake_detector" timeout_s="3.0"/>
               </BehaviorTree></root>)");
        status = tree.tickWhileRunning();
    }

    stop = true;
    spinner.join();
    executor.remove_node(detector);
    return status;
}

}  // namespace

TEST(LookFor, WaitsForTheIdTheDetectorPublishes)
{
    // /objects ids are slugs of the entry's id half.
    EXPECT_EQ(lookForWhilePublishing("Red Block", "red_block"), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(
        lookForWhilePublishing(" brown_box = brown box container", "brown_box"),
        BT::NodeStatus::SUCCESS);
}

TEST(LookFor, WritesTheObjectsAndTheAlsoEntriesVerbatim)
{
    EXPECT_THAT(
        phrasesLookForWrites(
            "brown_box=brown box container",
            "red_block=bright red plastic ball,blue_cup=blue plastic cup"),
        ::testing::ElementsAre(
            "brown_box=brown box container",
            "red_block=bright red plastic ball",
            "blue_cup=blue plastic cup"));
}

TEST(LookFor, TicksWithoutBlocking)
{
    auto                         tree_node = std::make_shared<rclcpp::Node>("test_look_for_client");
    BT::BehaviorTreeFactory      factory;
    g1_orchestration::RosContext context{ tree_node };
    g1_orchestration::registerSkillNodes(factory, context);
    BT::Tree tree = factory.createTreeFromText(
        R"(<root BTCPP_format="4"><BehaviorTree ID="M">
             <LookFor objects="red_block" detector="" timeout_s="30.0"/>
           </BehaviorTree></root>)");

    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(1));
    tree.haltTree();
}

TEST(ActionLeaf, ARejectedGoalFailsTheLeafRatherThanRunningForever)
{
    // A rejection arrives as a null goal handle and no result at all.
    EXPECT_EQ(runRetreatLeaf(false), BT::NodeStatus::FAILURE);
}

TEST(ActionLeaf, AnAcceptedGoalThatSucceedsSucceedsTheLeaf)
{
    // The counterpart: a leaf wired to fail on everything would pass the test above.
    EXPECT_EQ(runRetreatLeaf(true), BT::NodeStatus::SUCCESS);
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
