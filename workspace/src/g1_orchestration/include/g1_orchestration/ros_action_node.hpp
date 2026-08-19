#ifndef G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_
#define G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_

/**
 * @file ros_action_node.hpp
 * @brief The one pattern every action leaf in this package uses.
 *
 * Hand-rolled because BehaviorTree.ROS2 is not in this image.
 *
 * A tick must return promptly, so a leaf cannot block on an action: it sends the goal on the
 * first tick, answers RUNNING while the goal is in flight, and reports the outcome on whichever
 * tick sees the result. Halting cancels rather than abandons, so a halted skill cannot leave an
 * arm mid-trajectory.
 *
 * Two threads meet here: the tree ticks on one, the executor serves this client's callbacks on
 * the other. Only `result_` is shared and it is under `mutex_`; the goal handle is reached
 * through the future, which is its own synchronisation.
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

/**
 * @brief What every leaf here needs from the tree.
 */
struct RosContext
{
    rclcpp::Node::SharedPtr node;
};

/**
 * @brief A BT leaf wrapping one ROS action client.
 *
 * Derived classes supply the action name, fill the goal, and judge the result. Everything
 * about goal handling, cancellation and timeouts lives here.
 *
 * @tparam ActionT The ROS action this leaf drives.
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
     * An action can succeed at the protocol level while the skill it ran reports failure in its
     * own result fields, so the outcome is judged rather than assumed.
     *
     * @param result The completed goal's wrapped result.
     * @return SUCCESS or FAILURE for the leaf.
     */
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

    /**
     * @brief The server's answer to the goal that was sent.
     *
     * Read from the future rather than a goal_response_callback: rclcpp_action satisfies the
     * future one statement before it would invoke that callback, so a tick landing between the
     * two reads an accepted goal as refused.
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

        // Must not throw: BT.CPP halts from ~Tree(), so an escape here is std::terminate before
        // the executor's arm bracket runs, leaving the arm acquired by a dead process.
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
    std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
    std::mutex                                         mutex_;
    std::optional<WrappedResult>                       result_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__ROS_ACTION_NODE_HPP_
