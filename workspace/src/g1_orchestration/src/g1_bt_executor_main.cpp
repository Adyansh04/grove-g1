/**
 * @file g1_bt_executor_main.cpp
 * @brief Loads a behavior tree and ticks it, with the arm bracket guaranteed around the run.
 *
 * Whether the tree succeeds, fails, throws or is interrupted, arms this run took are released
 * before exit, unless a hand stands closed on something: a released hand drops what it holds.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "g1_orchestration/arm_authority.hpp"
#include "g1_orchestration/authority_port.hpp"
#include "g1_orchestration/skill_nodes.hpp"

namespace
{

// Written by the signal handler, where only a lock-free atomic is safe (not rclcpp::shutdown).
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::atomic<bool> g_interrupted{ false };

void onSignal(int) { g_interrupted = true; }

constexpr double kReleaseTimeoutS = 15.0;

/// Spin time after a halt, so published cancels are delivered before their clients are destroyed.
constexpr auto kCancelSettle = std::chrono::milliseconds(500);

/// Releases the arms this run took on scope exit, so no path out of main can skip it.
class ArmBracket
{
public:
    ArmBracket(
        const rclcpp::Logger& logger, double timeout_s,
        std::shared_ptr<g1_orchestration::ArmClaim> claim)
      : logger_(logger)
      , timeout_s_(timeout_s)
      , claim_(std::move(claim))
    {}

    ArmBracket(const ArmBracket&)            = delete;
    ArmBracket& operator=(const ArmBracket&) = delete;

    ~ArmBracket()
    {
        // Destructors are noexcept: an escaping exception would terminate the process.
        try
        {
            // Arms taken before this run (a bringup, another executor) are not its to release.
            if (!claim_->taken)
            {
                RCLCPP_INFO(logger_, "leaving the arms as they are: this run did not take them");
                return;
            }
            (void)g1_orchestration::releaseUnlessHolding(logger_, timeout_s_, claim_->fingers);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "the arm may still be acquired: release threw: %s", e.what());
        }
    }

private:
    rclcpp::Logger                              logger_;
    double                                      timeout_s_;
    std::shared_ptr<g1_orchestration::ArmClaim> claim_;
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
        auto      claim       = std::make_shared<g1_orchestration::ArmClaim>();
        claim->fingers.joints = node->declare_parameter<std::vector<std::string>>(
            "finger_joints",
            std::vector<std::string>{});
        claim->fingers.closed_rad = node->declare_parameter<double>("finger_closed_rad", 0.3);
        claim->fingers.timeout_s  = node->declare_parameter<double>("finger_read_timeout_s", 2.0);

        if (tree_file.empty())
        {
            RCLCPP_ERROR(node->get_logger(), "tree_file is required");
            rclcpp::shutdown();
            return 1;
        }

        // 1/0 is infinite, which is UB to cast to a duration; a negative rate would tick flat out.
        if (!std::isfinite(tick_rate_hz) || tick_rate_hz <= 0.0)
        {
            RCLCPP_ERROR(node->get_logger(), "tick_rate_hz must be positive, got %f", tick_rate_hz);
            rclcpp::shutdown();
            return 1;
        }

        // Installed before anything is acquired, so Ctrl-C reaches the release.
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
            const ArmBracket bracket(node->get_logger(), kReleaseTimeoutS, claim);

            try
            {
                BT::BehaviorTreeFactory      factory;
                g1_orchestration::RosContext context{ .node      = node,
                                                      .stop      = nullptr,
                                                      .pause     = nullptr,
                                                      .arm_claim = claim };
                g1_orchestration::registerSkillNodes(factory, context);

                BT::Tree tree = factory.createTreeFromFile(tree_file);
                RCLCPP_INFO(node->get_logger(), "loaded %s", tree_file.c_str());

                BT::StdCoutLogger                  cout_logger(tree);
                std::optional<BT::Groot2Publisher> groot2;
                if (groot2_port > 0)
                {
                    groot2.emplace(tree, groot2_port);
                    RCLCPP_INFO(
                        node->get_logger(),
                        "Groot2 can connect on port %d. Note the free tier monitors at most 20 "
                        "nodes.",
                        groot2_port);
                }

                // Declared after the tree so it joins first; otherwise a leaf can be freed inside
                // its result callback. spin_once, since spin_some never blocks and spins hot.
                std::jthread spinner([&executor](const std::stop_token& stop) {
                    while (rclcpp::ok() && !stop.stop_requested())
                    {
                        executor.spin_once(std::chrono::milliseconds(10));
                    }
                });

                // Ticked by hand, not tickWhileRunning, so an interrupt is seen between ticks.
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
                    // The spinner is still up, so the cancels haltTree published go out now.
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

            // The spinner and the tree are already destroyed.
            executor.remove_node(node);
        }

        rclcpp::shutdown();
        return exit_code;
    }
    catch (const std::exception& e)
    {
        // Either the bracket never existed or it has already released.
        RCLCPP_ERROR(
            rclcpp::get_logger("g1_bt_executor"),
            "startup or shutdown failed: %s",
            e.what());
        rclcpp::shutdown();
        return 1;
    }
}
