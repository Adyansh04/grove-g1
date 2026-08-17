/**
 * @file g1_bt_executor_main.cpp
 * @brief Loads a behavior tree and ticks it, with the arm bracket guaranteed around the run.
 *
 * The tree decides what happens. This file's own job is narrower and is the part that must not
 * be got wrong: whatever the tree does -- succeed, fail, throw, or be interrupted -- the arm
 * and hands are released before this process exits. control-mode rule 4 asks for
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
// behaviour in a handler, including rclcpp::shutdown. Mutable at namespace scope because a
// handler cannot capture.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::atomic<bool> g_interrupted{ false };

void onSignal(int) { g_interrupted = true; }

constexpr double kReleaseTimeoutS = 15.0;

/// Releases the arm and hands when it goes out of scope, however that happens.
///
/// A destructor rather than a call at the end of main: releasing only stays correct if every
/// path out actually reaches it, and a destructor turns that from a property of the control
/// flow into a property of the type -- control-mode rule 4 is then enforced by the language
/// itself.
class ArmBracket
{
public:
    ArmBracket(const rclcpp::Logger& logger, double timeout_s)
      : logger_(logger)
      , timeout_s_(timeout_s)
    {}

    ArmBracket(const ArmBracket&)            = delete;
    ArmBracket& operator=(const ArmBracket&) = delete;

    ~ArmBracket() { g1_orchestration::releaseArm(logger_, timeout_s_); }

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
        // 0 disables. 1667 is Groot2's own default, and the container runs with host
        // networking, so the editor on the host reaches it at localhost with nothing to
        // configure.
        const int groot2_port = static_cast<int>(node->declare_parameter<int>("groot2_port", 1667));

        if (tree_file.empty())
        {
            RCLCPP_ERROR(node->get_logger(), "tree_file is required");
            rclcpp::shutdown();
            return 1;
        }

        // Installed before anything is acquired, so a Ctrl-C during the run reaches the release
        // below rather than killing the process with the arm still active. Checked, because a
        // failed install silently removes that guarantee.
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
        std::thread spinner([&executor] {
            while (rclcpp::ok() && !g_interrupted)
            {
                executor.spin_some(std::chrono::milliseconds(10));
            }
        });

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
                // A tree that failed to load is a normal enough mistake; leaving the arm
                // acquired after one is not.
                RCLCPP_ERROR(node->get_logger(), "mission aborted: %s", e.what());
                exit_code = 1;
            }

            // Spinning stops before the bracket closes: releaseArm blocks on service calls and
            // needs the executor out of the way to make them.
            g_interrupted = true;
            spinner.join();
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
