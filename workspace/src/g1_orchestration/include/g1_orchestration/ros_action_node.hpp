#ifndef G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_
#define G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_

/**
 * @file ros_action_node.hpp
 * @brief Base for BT leaves that drive one ROS action without blocking a tick.
 *
 * Hand-rolled because BehaviorTree.ROS2 is not in this image. Halting cancels the goal, so a
 * halted skill does not leave an arm mid-trajectory. The tree ticks on one thread and the
 * executor runs the client callbacks on another; only `result_` is shared, under `mutex_`.
 */

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <format>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <utility>

#include "g1_orchestration/leaf_report.hpp"

namespace g1_orchestration
{

/**
 * @brief What every leaf here needs from the tree.
 */
struct RosContext
{
    rclcpp::Node::SharedPtr node;
    /// True while the mission has to wind down (a stop, a cancel, the watchdog). A leaf that would
    /// start something then fails at once, so a tick that loops through retries and waits comes to
    /// an end. Null in a tree that runs without an executor around it.
    const std::atomic<bool>* stop = nullptr;
};

/// Whether a leaf should refuse to start what it is about to: the mission is winding down.
inline bool windingDown(const std::atomic<bool>* stop) { return stop != nullptr && stop->load(); }

/**
 * @brief How a goal ended, in words, for a leaf's failure text.
 *
 * @param code The result code of a finished goal.
 */
inline std::string describeResultCode(rclcpp_action::ResultCode code)
{
    switch (code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            return "the goal succeeded";
        case rclcpp_action::ResultCode::ABORTED:
            return "the server aborted the goal";
        case rclcpp_action::ResultCode::CANCELED:
            return "the goal was cancelled";
        case rclcpp_action::ResultCode::UNKNOWN:
            break;
    }
    return "the goal ended in an unknown state";
}

/**
 * @brief A BT leaf wrapping one ROS action client.
 *
 * Derived classes supply the action name, fillGoal() and judgeResult().
 *
 * @tparam ActionT The ROS action this leaf drives.
 */
template <typename ActionT>
class RosActionNode : public BT::StatefulActionNode, public LeafReport
{
public:
    using Goal          = typename ActionT::Goal;
    using GoalHandle    = rclcpp_action::ClientGoalHandle<ActionT>;
    using WrappedResult = typename GoalHandle::WrappedResult;

    RosActionNode(
        const std::string& instance_name, const BT::NodeConfig& config, RosContext context,
        std::string action_name)
      : BT::StatefulActionNode(instance_name, config)
      , node_(std::move(context.node))
      , stop_(context.stop)
      , action_name_(std::move(action_name))
    {
        client_ = rclcpp_action::create_client<ActionT>(node_, action_name_);
    }

    /**
     * @brief Ports every action leaf shares.
     *
     * @param extra The derived leaf's own ports.
     * @return @p extra with the shared ports added.
     */
    static BT::PortsList providedBasicPorts(BT::PortsList extra)
    {
        extra.insert(BT::InputPort<double>(
            "server_timeout_s",
            10.0,
            "How long to wait for the action server to appear."));
        return extra;
    }

    [[nodiscard]] std::string actionServer() const override { return action_name_; }

protected:
    /**
     * @brief Fills the goal from the leaf's ports.
     *
     * @param goal Goal to populate.
     * @return False to fail the leaf before any goal is sent, e.g. on a malformed port.
     */
    virtual bool fillGoal(Goal& goal) = 0;

    /**
     * @brief Turns a finished goal into a node status.
     *
     * A SUCCEEDED goal can still carry a failed skill in its result fields.
     *
     * @param result The completed goal's wrapped result.
     * @return SUCCESS or FAILURE for the leaf.
     */
    virtual BT::NodeStatus judgeResult(const WrappedResult& result) = 0;

    rclcpp::Node::SharedPtr  node_;
    const std::atomic<bool>* stop_;

private:
    /**
     * @brief Waits for the server, in short slices so that a stop is seen within one of them.
     *
     * @param timeout_s The longest to wait.
     * @return Whether the server is there.
     */
    bool waitForServer(double timeout_s)
    {
        using Clock         = std::chrono::steady_clock;
        constexpr auto kNap = std::chrono::milliseconds(100);
        const auto     end  = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                            std::chrono::duration<double>(timeout_s));
        for (;;)
        {
            if (client_->wait_for_action_server(std::min<Clock::duration>(
                    std::max(end - Clock::now(), Clock::duration::zero()),
                    kNap)))
            {
                return true;
            }
            if (Clock::now() >= end || windingDown(stop_))
            {
                return false;
            }
        }
    }

    BT::NodeStatus onStart() override
    {
        setFailureText({});
        if (windingDown(stop_))
        {
            setFailureText("stopped");
            return BT::NodeStatus::FAILURE;
        }
        const double timeout = getInput<double>("server_timeout_s").value_or(10.0);
        if (!waitForServer(timeout))
        {
            if (windingDown(stop_))
            {
                setFailureText("stopped");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] no action server on '%s' after %.1f s",
                name().c_str(),
                action_name_.c_str(),
                timeout);
            setFailureText(
                std::format("no action server on {} after {:.1f} s", action_name_, timeout));
            return BT::NodeStatus::FAILURE;
        }

        Goal goal;
        if (!fillGoal(goal))
        {
            setFailureText(std::format("the ports for {} are unusable; see the log", action_name_));
            return BT::NodeStatus::FAILURE;
        }

        {
            const std::lock_guard<std::mutex> lock(mutex_);
            result_.reset();
        }

        typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
        // Lands on the executor's thread, not this one.
        options.result_callback = [this](const WrappedResult& result) {
            const std::lock_guard<std::mutex> lock(mutex_);
            result_ = result;
        };

        goal_future_ = client_->async_send_goal(goal, options);
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] sent goal to %s",
            name().c_str(),
            action_name_.c_str());
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief The server's answer to the goal that was sent.
     *
     * From the future, not goal_response_callback: rclcpp_action sets the future first, so a
     * tick between the two would see an accepted goal as refused.
     *
     * @return The handle if the server accepted, null if it refused, nullopt while in flight.
     */
    std::optional<typename GoalHandle::SharedPtr> serverAnswer()
    {
        if (!goal_future_.valid() ||
            goal_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        {
            return std::nullopt;
        }
        return goal_future_.get();
    }

    BT::NodeStatus onRunning() override
    {
        // Never spin here: an executor owns this node.
        std::optional<WrappedResult> result;
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            result = result_;
        }

        if (!result.has_value())
        {
            // A refused goal never produces a result, so it is caught here.
            const auto answer = serverAnswer();
            if (answer.has_value() && *answer == nullptr)
            {
                RCLCPP_ERROR(node_->get_logger(), "[%s] goal was rejected", name().c_str());
                setFailureText(std::format(
                    "{} rejected the goal (busy, or the goal is invalid)",
                    action_name_));
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
        // Outside the lock: judgeResult is derived code, and it logs.
        return judgeResult(*result);
    }

    void onHalted() override
    {
        bool have_result = false;
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            have_result = result_.has_value();
            result_.reset();
        }

        // Must not throw: BT.CPP halts from ~Tree(), and terminating there skips the arm release.
        try
        {
            // Not once the result is in: rclcpp_action forgets a finished goal, and cancelling
            // a forgotten goal throws.
            const auto answer = serverAnswer();
            if (!have_result && answer.has_value() && *answer != nullptr)
            {
                RCLCPP_WARN(node_->get_logger(), "[%s] halted; cancelling the goal", name().c_str());
                client_->async_cancel_goal(*answer);
            }
        }
        catch (const std::exception& e)
        {
            // The result raced the cancel, or the request could not be published.
            RCLCPP_WARN(node_->get_logger(), "[%s] cancel not sent: %s", name().c_str(), e.what());
        }
        goal_future_ = {};
    }

    typename rclcpp_action::Client<ActionT>::SharedPtr client_;
    std::string                                        action_name_;
    std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
    std::mutex                                         mutex_;
    std::optional<WrappedResult>                       result_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_
