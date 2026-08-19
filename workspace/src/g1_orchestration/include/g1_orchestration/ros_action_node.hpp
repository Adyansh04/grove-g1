#ifndef G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_
#define G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_

/**
 * @file ros_action_node.hpp
 * @brief The one pattern every action leaf in this package uses.
 *
 * Hand-rolled rather than taken from BehaviorTree.ROS2: that package is not in this image, and
 * vendoring the repo to get four leaves is more to carry than the ~100 lines it would replace.
 *
 * The shape is forced by how a behavior tree ticks. A tick must return promptly, so the leaf
 * cannot block on an action: it sends the goal on the first tick, answers RUNNING while the
 * goal is in flight, and reports the outcome on whichever tick sees the result. A halted leaf
 * cancels rather than abandoning the goal -- an arm left executing a trajectory the tree has
 * moved on from is exactly the "released cleanly on success or failure" rule in
 * the control-mode rules being broken.
 *
 * Two threads meet here: the tree is ticked on one and the executor serves this client's
 * callbacks on another, so the result is written by one and read by the other and is under
 * `mutex_`. Nothing else here is shared -- the goal handle is reached through the future, which
 * is itself the synchronisation.
 */

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <utility>

namespace g1_orchestration
{

/// What every leaf here needs from the tree: the node it borrows for its client.
struct RosContext
{
    rclcpp::Node::SharedPtr node;
};

/**
 * @brief A BT leaf wrapping one ROS action client.
 *
 * Derived classes supply the action name, fill the goal, and judge the result. Everything
 * about goal handling, cancellation and timeouts lives here.
 */
template <typename ActionT>
class RosActionNode : public BT::StatefulActionNode
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
      , action_name_(std::move(action_name))
    {
        client_ = rclcpp_action::create_client<ActionT>(node_, action_name_);
    }

    /// Ports every action leaf shares. Derived classes append their own to this.
    static BT::PortsList providedBasicPorts(BT::PortsList extra)
    {
        extra.insert(BT::InputPort<double>(
            "server_timeout_s",
            10.0,
            "How long to wait for the action server to appear."));
        return extra;
    }

protected:
    /// False to fail the leaf before any goal is sent, e.g. on a malformed port.
    virtual bool fillGoal(Goal& goal) = 0;

    /// Judged rather than assumed: an action can succeed at the protocol level while the
    /// skill it ran reports failure in its own result fields.
    virtual BT::NodeStatus judgeResult(const WrappedResult& result) = 0;

    rclcpp::Node::SharedPtr node_;

private:
    BT::NodeStatus onStart() override
    {
        const double timeout = getInput<double>("server_timeout_s").value_or(10.0);
        if (!client_->wait_for_action_server(std::chrono::duration<double>(timeout)))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] no action server on '%s' after %.1f s",
                name().c_str(),
                action_name_.c_str(),
                timeout);
            return BT::NodeStatus::FAILURE;
        }

        Goal goal;
        if (!fillGoal(goal))
        {
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

    /// The server's answer: the handle if it accepted, null if it refused, nullopt while the
    /// answer is still in flight.
    ///
    /// Taken from the future rather than from a goal_response_callback, and that is the whole
    /// reason there isn't one: rclcpp_action satisfies the future one statement BEFORE it would
    /// call that callback, so a tick landing between the two reads an accepted goal as refused.
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
        // Nothing is spun here: the executor owns this node, and a leaf spinning it from
        // inside a tick would re-enter the executor from its own callback.
        std::optional<WrappedResult> result;
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            result = result_;
        }

        if (!result.has_value())
        {
            // A refused goal never produces a result, so it has to be caught here or the leaf
            // runs forever.
            const auto answer = serverAnswer();
            if (answer.has_value() && *answer == nullptr)
            {
                RCLCPP_ERROR(node_->get_logger(), "[%s] goal was rejected", name().c_str());
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
        // Judged outside the lock: it logs, and a derived judgeResult is arbitrary code.
        return judgeResult(*result);
    }

    void onHalted() override
    {
        // The tree has moved on; the robot has not. Cancelling is what keeps a halted skill
        // from leaving an arm mid-trajectory.
        bool have_result = false;
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            have_result = result_.has_value();
            result_.reset();
        }

        // Nothing below may throw. BT.CPP calls this from Tree::haltTree(), which ~Tree() calls
        // from a destructor, so an escape here is std::terminate -- and it would land before the
        // executor's arm bracket had run, leaving the arm acquired by a dead process.
        try
        {
            // Skipped once the result is in: rclcpp_action forgets a goal the moment its result
            // lands, and cancelling one it has forgotten throws rather than returning.
            const auto answer = serverAnswer();
            if (!have_result && answer.has_value() && *answer != nullptr)
            {
                RCLCPP_WARN(node_->get_logger(), "[%s] halted; cancelling the goal", name().c_str());
                client_->async_cancel_goal(*answer);
            }
        }
        catch (const std::exception& e)
        {
            // The result raced us by a tick, or the request could not be published. Either way
            // the goal is no longer ours to stop.
            RCLCPP_WARN(node_->get_logger(), "[%s] cancel not sent: %s", name().c_str(), e.what());
        }
        goal_future_ = {};
    }

    typename rclcpp_action::Client<ActionT>::SharedPtr client_;
    std::string                                        action_name_;
    /// Tick thread only. The executor's thread satisfies the promise behind it and never reads
    /// this object, so it needs no lock.
    std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
    /// Written by the executor's thread, read by the tree's: guards result_ and nothing else.
    std::mutex                   mutex_;
    std::optional<WrappedResult> result_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_
