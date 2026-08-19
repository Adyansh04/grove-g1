/**
 * @file test_action_leaf.cpp
 * @brief The action-leaf base against a real server, on the two threads the executor runs it on.
 *
 * The base answers RUNNING across ticks and reads its outcome from callbacks arriving on a
 * different thread, so neither the rejection nor the success path exists until something
 * answers a goal. Driven in the shape g1_bt_executor uses, tree on this thread and executor on
 * another, because that split is the thing under test.
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

#include "g1_orchestration/skill_nodes.hpp"

namespace
{

using Retreat    = g1_msgs::action::Retreat;
using GoalHandle = rclcpp_action::ServerGoalHandle<Retreat>;

constexpr const char* kRetreatAction = "/g1_base_approach/retreat";

/// How long an accepted goal stays in flight. Must not be zero: an instant server lands the
/// result before the next tick, so the leaf never runs a tick in the accepted-but-unfinished
/// state, which is the only state where reading an accepted goal as refused is visible.
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

/// Ticks a one-leaf tree to completion, or gives up. Returns RUNNING if it never settled, which
/// is a distinct failure from FAILURE and the one a leaf that ignores a rejection produces.
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

}  // namespace

TEST(ActionLeaf, ARejectedGoalFailsTheLeafRatherThanRunningForever)
{
    // A rejection arrives as a null goal handle and produces no result at all, so a leaf that
    // only watched for a result would sit RUNNING until the mission was killed.
    EXPECT_EQ(runRetreatLeaf(false), BT::NodeStatus::FAILURE);
}

TEST(ActionLeaf, AnAcceptedGoalThatSucceedsSucceedsTheLeaf)
{
    // The compensating half: a leaf wired to fail on everything would pass the test above, and
    // one that read an accepted goal as refused would fail here while the goal is unfinished.
    EXPECT_EQ(runRetreatLeaf(true), BT::NodeStatus::SUCCESS);
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
