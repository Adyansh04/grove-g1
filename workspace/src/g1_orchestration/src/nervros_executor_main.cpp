/**
 * @file nervros_executor_main.cpp
 * @brief Entry point for the nervros_executor node.
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "g1_orchestration/nervros_executor.hpp"

namespace
{

// Written by the signal handler, where only a lock-free atomic is safe (not rclcpp::shutdown).
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::atomic<bool> g_interrupted{ false };

void onSignal(int) { g_interrupted = true; }

/// One thread for each callback group that may be busy at once: the mission's action, StopAll,
/// the queries, the servers' status, and the state timer.
constexpr std::size_t kExecutorThreads = 5;

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // Installed after init, over rclcpp's own, so an interrupt reaches the orderly shutdown below
    // while the context is still valid: the mission's cancels have to go out first.
    const auto previous_int  = std::signal(SIGINT, onSignal);
    const auto previous_term = std::signal(SIGTERM, onSignal);
    if (previous_int == SIG_ERR || previous_term == SIG_ERR)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("nervros_executor"),
            "could not install the SIGINT/SIGTERM handlers; an interrupt will end the process "
            "without halting the mission");
    }

    int exit_code = 0;
    try
    {
        auto node = std::make_shared<g1_orchestration::MissionExecutor>();
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(),
            kExecutorThreads);
        executor.add_node(node);
        {
            std::jthread spinner([&executor] { executor.spin(); });
            while (rclcpp::ok() && !g_interrupted)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            // Still spinning, so the halt's cancels and the servers' answers are handled. Whatever
            // shutdown() does, the executor is cancelled: the spinner's join would wait for it.
            try
            {
                node->shutdown();
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("nervros_executor"),
                    "the orderly shutdown failed: %s",
                    e.what());
            }
            executor.cancel();
        }
        executor.remove_node(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("nervros_executor"), "fatal: %s", e.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
