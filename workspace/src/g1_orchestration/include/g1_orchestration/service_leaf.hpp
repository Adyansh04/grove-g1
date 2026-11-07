#ifndef G1_ORCHESTRATION__SERVICE_LEAF_HPP_
#define G1_ORCHESTRATION__SERVICE_LEAF_HPP_

/**
 * @file service_leaf.hpp
 * @brief Blocking service calls from a leaf, on a short-lived node of their own.
 *
 * spin_until_future_complete throws on a node an executor already holds.
 */

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "g1_orchestration/leaf_report.hpp"
#include "g1_orchestration/ros_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief A node nothing else is spinning, for the duration of one call sequence.
 *
 * @param name Node name; must be unique on the graph.
 * @return The new node, owned by the caller.
 */
rclcpp::Node::SharedPtr makeClientNode(const std::string& name);

/**
 * @brief Calls a service and waits for the response, blocking the tick.
 *
 * @tparam ServiceT The ROS service type.
 * @param node A node no executor owns, from makeClientNode().
 * @param service Fully qualified service name.
 * @param request The request to send.
 * @param timeout_s Total budget for discovery and the call together.
 * @param stop A stop or pause of the mission, read between 100 ms slices of the waits; null to
 *             wait the whole budget.
 * @return The response, or null if the service never appeared, did not answer in time, or the
 *         mission was stopped while it waited.
 */
template <typename ServiceT>
typename ServiceT::Response::SharedPtr callService(
    const rclcpp::Node::SharedPtr& node, const std::string& service,
    const typename ServiceT::Request::SharedPtr& request, double timeout_s,
    const std::atomic<bool>* stop = nullptr)
{
    // One deadline across discovery and the call, so timeout_s is the real bound.
    using Clock         = std::chrono::steady_clock;
    const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                             std::chrono::duration<double>(timeout_s));
    const auto remaining = [&deadline] {
        return std::max(deadline - Clock::now(), Clock::duration::zero());
    };

    // In slices, so a stop ends the wait within one rather than at the budget's end.
    const auto slice = [&remaining] {
        return std::min<Clock::duration>(std::chrono::milliseconds(100), remaining());
    };

    auto client = node->create_client<ServiceT>(service);
    while (!client->wait_for_service(slice()))
    {
        if (windingDown(stop) || remaining() == Clock::duration::zero())
        {
            RCLCPP_ERROR(node->get_logger(), "service '%s' never appeared", service.c_str());
            return nullptr;
        }
    }
    auto future = client->async_send_request(request);
    for (;;)
    {
        const auto code = rclcpp::spin_until_future_complete(node, future, slice());
        if (code == rclcpp::FutureReturnCode::SUCCESS)
        {
            return future.get();
        }
        if (code != rclcpp::FutureReturnCode::TIMEOUT || windingDown(stop) ||
            remaining() == Clock::duration::zero())
        {
            RCLCPP_ERROR(node->get_logger(), "call to '%s' did not return", service.c_str());
            client->remove_pending_request(future);
            return nullptr;
        }
    }
}

/**
 * @brief Base for leaves that complete within one tick.
 *
 * Holds the tree's node for logging; the calls themselves use makeClientNode(). A call can block
 * for its whole budget, so none is started once the mission is winding down, and a pause holds
 * the leaf, RUNNING, until it is resumed.
 */
class ServiceLeaf : public BT::StatefulActionNode, public LeafReport
{
public:
    ServiceLeaf(const std::string& name, const BT::NodeConfig& config, RosContext context)
      : BT::StatefulActionNode(name, config)
      , node_(std::move(context.node))
      , stop_(context.stop)
      , pause_(context.pause)
    {}

    BT::NodeStatus onStart() final
    {
        setFailureText({});
        if (windingDown(stop_))
        {
            setFailureText("stopped");
            return BT::NodeStatus::FAILURE;
        }
        return heldByPause(pause_, *this) ? BT::NodeStatus::RUNNING : call();
    }

    BT::NodeStatus onRunning() final { return onStart(); }

    void onHalted() final {}

protected:
    /// The leaf's work, which finishes within the tick.
    virtual BT::NodeStatus call() = 0;

    /// The mission's stop flag, for the leaf's own waits and calls to end on.
    [[nodiscard]] const std::atomic<bool>* stopFlag() const { return stop_; }

    rclcpp::Node::SharedPtr node_;

private:
    const std::atomic<bool>* stop_;
    const PauseGate*         pause_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SERVICE_LEAF_HPP_
