#ifndef G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_
#define G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_

/**
 * @file ros_action_node.hpp
 * @brief Base for BT leaves that drive one ROS action without blocking a tick.
 *
 * Hand-rolled because BehaviorTree.ROS2 is not in this image. Halting cancels the goal, so a
 * halted skill does not leave an arm mid-trajectory. The tree ticks on one thread and the
 * executor runs the client callbacks on another; only `result_` is shared, under `mutex_`. While
 * the mission is paused a leaf that is about to send its goal waits instead.
 */

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <format>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <string_view>
#include <utility>

#include "g1_orchestration/leaf_report.hpp"

namespace g1_orchestration
{

/**
 * @brief Whether the mission is paused, as its leaves see it.
 *
 * Closed, it holds every leaf about to send a goal: the leaf waits, RUNNING, and sends it once
 * the gate opens. The one step a pause lets finish, a pick or a place, runs on.
 */
class PauseGate
{
public:
    /// Holds the leaves of every step but @p finishing, the name of a step allowed to end first.
    void close(std::string finishing = {})
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        closed_    = true;
        finishing_ = std::move(finishing);
    }

    void open()
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        closed_ = false;
        finishing_.clear();
    }

    /// Whether the leaf at @p path, which begins with its step's name, has to wait.
    [[nodiscard]] bool holds(std::string_view path) const
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        return closed_ && (finishing_.empty() || path.substr(0, path.find('/')) != finishing_);
    }

private:
    mutable std::mutex mutex_;
    bool               closed_ = false;
    std::string        finishing_;
};

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
    /// Closed while the mission is paused. Null in a tree that runs without an executor.
    const PauseGate* pause = nullptr;
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
    using Feedback      = typename ActionT::Feedback;
    using GoalHandle    = rclcpp_action::ClientGoalHandle<ActionT>;
    using WrappedResult = typename GoalHandle::WrappedResult;

    RosActionNode(
        const std::string& instance_name, const BT::NodeConfig& config, RosContext context,
        std::string action_name)
      : BT::StatefulActionNode(instance_name, config)
      , node_(std::move(context.node))
      , stop_(context.stop)
      , pause_(context.pause)
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

    /// The running goal's feedback, on the thread that runs the client's callbacks.
    virtual void onFeedback(const Feedback& /*feedback*/) {}

    /**
     * @brief The leaf was halted while its goal was out.
     *
     * @param pausing The mission is pausing, so the leaf runs again once it is resumed.
     * @param finished The goal had already succeeded when the halt came.
     */
    virtual void onHaltedWithGoal(bool /*pausing*/, bool /*finished*/) {}

    /// Whether there is nothing left to do, as for a walk resumed after it covered its distance:
    /// the leaf then succeeds without sending a goal.
    virtual bool nothingLeft() { return false; }

    rclcpp::Node::SharedPtr  node_;
    const std::atomic<bool>* stop_;
    const PauseGate*         pause_;

private:
    [[nodiscard]] bool held() const { return pause_ != nullptr && pause_->holds(fullPath()); }

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
        setGoalSent(false);
        setServerMissing(false);
        if (windingDown(stop_))
        {
            setFailureText("stopped");
            return BT::NodeStatus::FAILURE;
        }
        if (held())
        {
            waiting_ = true;
            return BT::NodeStatus::RUNNING;
        }
        return startGoal();
    }

    BT::NodeStatus startGoal()
    {
        if (nothingLeft())
        {
            return BT::NodeStatus::SUCCESS;
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
            setServerMissing(true);
            return BT::NodeStatus::FAILURE;
        }

        Goal goal;
        if (!fillGoal(goal))
        {
            // A leaf that said why, such as an object the world model does not know, keeps it.
            if (failureText().empty())
            {
                setFailureText(
                    std::format("the ports for {} are unusable; see the log", action_name_));
            }
            return BT::NodeStatus::FAILURE;
        }

        std::uint64_t sent = 0;
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            result_.reset();
            sent = ++goals_;
        }

        typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
        // Lands on the executor's thread, not this one. Only this goal's: a goal halted before,
        // as by a pause, can still answer after its step has set out again.
        options.result_callback = [this, sent](const WrappedResult& result) {
            const std::lock_guard<std::mutex> lock(mutex_);
            if (sent == goals_)
            {
                result_ = result;
            }
        };
        options.feedback_callback = [this](
                                        const typename GoalHandle::SharedPtr& /*handle*/,
                                        const std::shared_ptr<const Feedback>& feedback) {
            onFeedback(*feedback);
        };

        goal_future_ = client_->async_send_goal(goal, options);
        setGoalSent(true);
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
        if (waiting_)
        {
            if (windingDown(stop_))
            {
                waiting_ = false;
                setFailureText("stopped");
                return BT::NodeStatus::FAILURE;
            }
            if (held())
            {
                return BT::NodeStatus::RUNNING;
            }
            waiting_ = false;
            return startGoal();
        }
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
        if (waiting_)
        {
            // Held by a pause, it never sent its goal.
            waiting_ = false;
            return;
        }
        bool have_result = false;
        bool succeeded   = false;
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            have_result = result_.has_value();
            succeeded   = have_result && result_->code == rclcpp_action::ResultCode::SUCCEEDED;
            result_.reset();
            ++goals_;
        }

        // Must not throw: BT.CPP halts from ~Tree(), and terminating there skips the arm release.
        try
        {
            // Not once the result is in: rclcpp_action forgets a finished goal, and cancelling
            // a forgotten goal throws.
            const auto answer = serverAnswer();
            if (answer.has_value() && *answer != nullptr)
            {
                onHaltedWithGoal(held(), succeeded);
            }
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
    /// Counts the goals sent and the halts, so a result is kept only for the goal now out.
    std::uint64_t goals_ = 0;
    /// Held by a pause before its goal went out.
    bool waiting_ = false;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_
