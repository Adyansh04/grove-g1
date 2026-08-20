/**
 * @file g1_bt_executor_main.cpp
 * @brief Loads a behavior tree and ticks it, with the arm bracket guaranteed around the run.
 *
 * The tree decides what happens. This file guarantees the narrower thing: whatever the tree
 * does, succeed, fail, throw or be interrupted, the arm and hands are released before the
 * process exits. A tree cannot promise that for itself, because the paths where it matters most
 * are the ones where the tree stopped running.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <atomic>
#include <chrono>
#include <cmath>
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
// behaviour in a handler, rclcpp::shutdown included.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::atomic<bool> g_interrupted{ false };

void onSignal(int) { g_interrupted = true; }

constexpr double kReleaseTimeoutS = 15.0;

/// How long the executor keeps running after a halt, so the cancels it published are delivered
/// and answered before the clients that sent them are destroyed.
constexpr auto kCancelSettle = std::chrono::milliseconds(500);

/// Releases the arm and hands when it goes out of scope, however that happens. A destructor
/// rather than a call at the end of main, so no path out can skip it.
class ArmBracket
{
public:
    ArmBracket(const rclcpp::Logger& logger, double timeout_s)
      : logger_(logger)
      , timeout_s_(timeout_s)
    {}

    ArmBracket(const ArmBracket&)            = delete;
    ArmBracket& operator=(const ArmBracket&) = delete;

    ~ArmBracket()
    {
        // A destructor is noexcept, and releaseArm makes service calls on a node it builds, so
        // anything escaping it would end the process instead of releasing.
        try
        {
            g1_orchestration::releaseArm(logger_, timeout_s_);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "the arm may still be acquired: release threw: %s", e.what());
        }
    }

private:
    rclcpp::Logger logger_;
    double         timeout_s_;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Unwinding past main() uncaught doesn't guarantee ArmBracket's destructor runs.
    try
    {
        auto node = std::make_shared<rclcpp::Node>("g1_bt_executor");

        const std::string tree_file    = node->declare_parameter<std::string>("tree_file", "");
        const double      tick_rate_hz = node->declare_parameter<double>("tick_rate_hz", 10.0);
        const int groot2_port = static_cast<int>(node->declare_parameter<int>("groot2_port", 1667));

        if (tree_file.empty())
        {
            RCLCPP_ERROR(node->get_logger(), "tree_file is required");
            rclcpp::shutdown();
            return 1;
        }

        // Refused rather than clamped: 0 divides to infinity and casting that to a duration is
        // undefined, and a negative ticks the tree as fast as the CPU allows.
        if (!std::isfinite(tick_rate_hz) || tick_rate_hz <= 0.0)
        {
            RCLCPP_ERROR(node->get_logger(), "tick_rate_hz must be positive, got %f", tick_rate_hz);
            rclcpp::shutdown();
            return 1;
        }

        // Installed before anything is acquired, so a Ctrl-C reaches the release below rather
        // than killing the process with the arm still active.
        const auto previous_int  = std::signal(SIGINT, onSignal);
        const auto previous_term = std::signal(SIGTERM, onSignal);
        if (previous_int == SIG_ERR || previous_term == SIG_ERR)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "could not install the SIGINT/SIGTERM handlers -- an interrupt will now exit "
                "without releasing the arm");
        }

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        int exit_code = 0;
        {
            // Closes when this scope ends, on every path out of it.
            const ArmBracket bracket(node->get_logger(), kReleaseTimeoutS);

            try
            {
                BT::BehaviorTreeFactory      factory;
                g1_orchestration::RosContext context{ node };
                g1_orchestration::registerSkillNodes(factory, context);

                BT::Tree tree = factory.createTreeFromFile(tree_file);
                RCLCPP_INFO(node->get_logger(), "loaded %s", tree_file.c_str());

                BT::StdCoutLogger                    cout_logger(tree);
                std::unique_ptr<BT::Groot2Publisher> groot2;
                if (groot2_port > 0)
                {
                    groot2 = std::make_unique<BT::Groot2Publisher>(tree, groot2_port);
                    RCLCPP_INFO(
                        node->get_logger(),
                        "Groot2 can connect on port %d. Note the free tier monitors at most 20 "
                        "nodes.",
                        groot2_port);
                }

                // Declared after the tree so it is joined before the tree is destroyed; the
                // other order frees a leaf while the executor is inside its result callback.
                //
                // spin_once, not spin_some: spin_some never blocks waiting for work, so on an
                // idle executor it returns at once and this loop spins hot.
                std::jthread spinner([&executor](const std::stop_token& stop) {
                    while (rclcpp::ok() && !stop.stop_requested())
                    {
                        executor.spin_once(std::chrono::milliseconds(10));
                    }
                });

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
                    // haltTree publishes each leaf's cancel and returns. The spinner is still up
                    // here, so this is the window in which those reach the wire and are answered.
                    std::this_thread::sleep_for(kCancelSettle);
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
                // The bracket above still releases the arm on the way out of this scope.
                RCLCPP_ERROR(node->get_logger(), "mission aborted: %s", e.what());
                exit_code = 1;
            }

            // The spinner and the tree are already gone by here, both inside the block above.
            executor.remove_node(node);
        }

        rclcpp::shutdown();
        return exit_code;
    }
    catch (const std::exception& e)
    {
        // Nothing here has acquired the arm yet, so there is nothing to release.
        RCLCPP_ERROR(
            rclcpp::get_logger("g1_bt_executor"),
            "startup or shutdown failed: %s",
            e.what());
        rclcpp::shutdown();
        return 1;
    }
}
