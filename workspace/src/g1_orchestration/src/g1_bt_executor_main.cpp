/**
 * @file g1_bt_executor_main.cpp
 * @brief Loads a behavior tree and ticks it, with the arm bracket guaranteed around the run.
 *
 * The tree decides what happens. This file's own job is narrower and is the part that must not
 * be got wrong: whatever the tree does -- succeed, fail, throw, or be interrupted -- the arm
 * and hands are released before this process exits. docs/CONTROL_MODES.md rule 4 asks for
 * exactly that, and a tree cannot promise it for itself, because the paths where it matters
 * most are the ones where the tree stopped running.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "g1_orchestration/arm_authority.hpp"
#include "g1_orchestration/skill_nodes.hpp"

namespace
{

// Set from the signal handler, so it must be exactly this type: everything else is undefined
// behaviour in a handler, including rclcpp::shutdown.
std::atomic<bool> g_interrupted{ false };

void onSignal(int) { g_interrupted = true; }

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("g1_bt_executor");

    const std::string tree_file    = node->declare_parameter<std::string>("tree_file", "");
    const double      tick_rate_hz = node->declare_parameter<double>("tick_rate_hz", 10.0);
    // 0 disables. 1667 is Groot2's own default, and the container runs with host networking,
    // so the editor on the host reaches it at localhost with nothing to configure.
    const int groot2_port = node->declare_parameter<int>("groot2_port", 1667);

    if (tree_file.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "tree_file is required");
        return 1;
    }

    // Installed before anything is acquired, so a Ctrl-C during the run reaches the release
    // below rather than killing the process with the arm still active.
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor] {
        while (rclcpp::ok() && !g_interrupted)
        {
            executor.spin_some(std::chrono::milliseconds(10));
        }
    });

    int exit_code = 0;
    try
    {
        BT::BehaviorTreeFactory      factory;
        g1_orchestration::RosContext context{ node };
        g1_orchestration::registerSkillNodes(factory, context);
        g1_orchestration::registerAuthorityNodes(factory, context);

        BT::Tree tree = factory.createTreeFromFile(tree_file);
        RCLCPP_INFO(node->get_logger(), "loaded %s", tree_file.c_str());

        BT::StdCoutLogger                    cout_logger(tree);
        std::unique_ptr<BT::Groot2Publisher> groot2;
        if (groot2_port > 0)
        {
            groot2 = std::make_unique<BT::Groot2Publisher>(tree, groot2_port);
            RCLCPP_INFO(
                node->get_logger(),
                "Groot2 can connect on port %d. Note the free tier monitors at most 20 nodes.",
                groot2_port);
        }

        // Ticked by hand rather than with tickWhileRunning, so the interrupt is checked
        // between ticks and a halt still runs every leaf's own cancellation.
        const auto     period = std::chrono::duration<double>(1.0 / tick_rate_hz);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        while (rclcpp::ok() && !g_interrupted && status == BT::NodeStatus::RUNNING)
        {
            status = tree.tickOnce();
            std::this_thread::sleep_for(
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
        }

        if (g_interrupted)
        {
            RCLCPP_WARN(node->get_logger(), "interrupted; halting the tree");
            tree.haltTree();
            exit_code = 130;
        }
        else
        {
            RCLCPP_INFO(
                node->get_logger(),
                "mission finished: %s",
                status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
            exit_code = status == BT::NodeStatus::SUCCESS ? 0 : 1;
        }
    }
    catch (const std::exception& e)
    {
        // Caught rather than allowed to propagate, so the release below still runs. A tree
        // that failed to load is a normal enough mistake; leaving the arm acquired after one
        // is not.
        RCLCPP_ERROR(node->get_logger(), "mission aborted: %s", e.what());
        exit_code = 1;
    }

    // Spinning stops first: releaseArm blocks on service calls, and it needs the executor out
    // of the way to make them.
    g_interrupted = true;
    spinner.join();
    executor.remove_node(node);

    // The bracket closes here, on every path out of the block above. It runs even when the
    // tree never acquired anything, which is safe: deactivating an already-inactive component
    // is a no-op that logs.
    g1_orchestration::releaseArm(node->get_logger(), 15.0);

    rclcpp::shutdown();
    return exit_code;
}
