#ifndef G1_ORCHESTRATION__MISSION_OBSERVER_HPP_
#define G1_ORCHESTRATION__MISSION_OBSERVER_HPP_

/**
 * @file mission_observer.hpp
 * @brief Watches a running mission tree: what changed, what runs, which step, and why it failed.
 *
 * Transitions arrive on the thread that ticks, and also on the timer thread of a Timeout node
 * that halts its child. They are batched for the action's feedback, the running step is tracked
 * for the robot state, and the failure that ends a mission is kept together with the words the
 * failing leaf gave for it.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>

#include <functional>
#include <mutex>
#include <nervros_interfaces/msg/node_event.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "g1_orchestration/mission_validator.hpp"

namespace g1_orchestration
{

/// The first step id in a node's path, or empty when the node sits outside every step.
std::string stepOfPath(std::string_view path);

/**
 * @brief The failure that ended a mission.
 */
struct Failure
{
    /// Path of the leaf, or of the Timeout that gave up on it.
    std::string path;
    /// The step id the node belongs to, such as "s2".
    std::string step;
    std::string reason;
    /// The node was a Timeout: the mission or a step ran out of time.
    bool timed_out = false;
};

class MissionObserver : public BT::StatusChangeLogger
{
public:
    /// Which transitions become events.
    enum class Level
    {
        kSteps,  ///< The step SubTrees and the skill leaves.
        kAll,    ///< Every node.
    };

    struct Hooks
    {
        /// A step began running; gets its name, such as "s2_GoToPose".
        std::function<void(const std::string&)> on_step;
        /// A leaf reported SUCCESS.
        std::function<void(const BT::TreeNode&)> on_success;
        /// A node was halted while it ran, whoever halted it: the executor, or a Timeout.
        std::function<void(const BT::TreeNode&)> on_halt;
    };

    /**
     * @param root The tree's root node.
     * @param clock Stamps the events.
     * @param level Which transitions are kept as events.
     * @param skill_leaves Registration IDs of the leaves that count as action leaves.
     * @param hooks What to tell the executor as it happens.
     */
    MissionObserver(
        BT::TreeNode* root, rclcpp::Clock::SharedPtr clock, Level level,
        std::set<std::string> skill_leaves, Hooks hooks);

    void callback(
        BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev,
        BT::NodeStatus status) override;

    void flush() override {}

    /// Events since the last call.
    [[nodiscard]] std::vector<nervros_interfaces::msg::NodeEvent> takeEvents();

    /// Paths of the nodes that are RUNNING now.
    [[nodiscard]] std::vector<std::string> running() const;

    /// The last leaf, or Timeout that ran out, to fail before the tree did, if any has.
    [[nodiscard]] std::optional<Failure> failure() const;

    /// The step that started running most recently, such as "s2_GoToPose".
    [[nodiscard]] std::string currentStep() const;

private:
    [[nodiscard]] bool
    wanted(const BT::TreeNode& node, BT::NodeStatus prev, BT::NodeStatus status) const;
    [[nodiscard]] Failure describe(const BT::TreeNode& node) const;
    /// Whether a Timeout is failing because it halted its child, not because the child failed.
    [[nodiscard]] bool ranOut(const BT::TreeNode& timeout) const;

    rclcpp::Clock::SharedPtr clock_;
    Level                    level_;
    std::set<std::string>    skill_leaves_;
    Hooks                    hooks_;

    mutable std::mutex                              mutex_;
    std::vector<nervros_interfaces::msg::NodeEvent> events_;
    std::set<std::string>                           running_;
    /// Nodes whose last transition was a halt, so a Timeout can tell it fired.
    std::set<std::string>  halted_;
    std::optional<Failure> failure_;
    std::string            current_step_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__MISSION_OBSERVER_HPP_
