#ifndef G1_ORCHESTRATION__SERVICE_LEAF_HPP_
#define G1_ORCHESTRATION__SERVICE_LEAF_HPP_

/**
 * @file service_leaf.hpp
 * @brief Blocking service calls from a leaf, on a node the executor does not own.
 *
 * spin_until_future_complete on a node an executor already holds throws rather than waiting, so
 * these calls need a node of their own. It is created for the call and destroyed with it.
 */

#include <behaviortree_cpp/action_node.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

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
 * @return The response, or null if the service never appeared or did not answer in time.
 */
template <typename ServiceT>
typename ServiceT::Response::SharedPtr callService(
    const rclcpp::Node::SharedPtr& node, const std::string& service,
    const typename ServiceT::Request::SharedPtr& request, double timeout_s)
{
    // One budget across both waits, not one each: a full timeout apiece makes a leaf that says
    // it will take 15 s take 30, and an acquire runs five of them inside one tick.
    using Clock         = std::chrono::steady_clock;
    const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                             std::chrono::duration<double>(timeout_s));
    const auto remaining = [&deadline] {
        return std::max(deadline - Clock::now(), Clock::duration::zero());
    };

    auto client = node->create_client<ServiceT>(service);
    if (!client->wait_for_service(remaining()))
    {
        RCLCPP_ERROR(node->get_logger(), "service '%s' never appeared", service.c_str());
        return nullptr;
    }
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, remaining()) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "call to '%s' did not return", service.c_str());
        return nullptr;
    }
    return future.get();
}

/**
 * @brief Base for leaves that complete within one tick.
 *
 * Holds the tree's node for logging; the calls themselves use makeClientNode().
 */
class ServiceLeaf : public BT::SyncActionNode
{
public:
    ServiceLeaf(const std::string& name, const BT::NodeConfig& config, RosContext context)
      : BT::SyncActionNode(name, config)
      , node_(std::move(context.node))
    {}

protected:
    rclcpp::Node::SharedPtr node_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SERVICE_LEAF_HPP_
