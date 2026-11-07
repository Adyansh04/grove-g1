/**
 * @file clear_costmaps.cpp
 * @brief Ports and tick for the ClearCostmaps leaf.
 */

#include "g1_orchestration/skills/clear_costmaps.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

ClearCostmaps::ClearCostmaps(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : ServiceLeaf(name, config, std::move(context))
{}

BT::PortsList ClearCostmaps::providedPorts()
{
    return {
        ports::serviceTimeout(5.0, "Per-costmap service budget."),
        BT::InputPort<std::string>(
            "global_service",
            "/global_costmap/clear_entirely_global_costmap",
            "Nav2 global costmap clear service. Empty skips it."),
        BT::InputPort<std::string>(
            "local_service",
            "/local_costmap/clear_entirely_local_costmap",
            "Nav2 local costmap clear service. Empty skips it."),
    };
}

BT::NodeStatus ClearCostmaps::call()
{
    using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;
    using Clock              = std::chrono::steady_clock;

    const double timeout_s   = getInput<double>("timeout_s").value_or(5.0);
    auto         client_node = makeClientNode("g1_clear_costmaps_client");
    const auto   deadline    = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                             std::chrono::duration<double>(timeout_s));
    // In slices, so a stop ends a wait within one.
    const auto slice = [&deadline] {
        return std::min<Clock::duration>(
            std::chrono::milliseconds(100),
            std::max(deadline - Clock::now(), Clock::duration::zero()));
    };
    const auto expired = [&] { return windingDown(stopFlag()) || Clock::now() >= deadline; };

    // Both requests go out before either is waited on, so the one budget covers the pair.
    struct Call
    {
        std::string                                                           service;
        rclcpp::Client<ClearEntireCostmap>::SharedPtr                         client;
        std::optional<rclcpp::Client<ClearEntireCostmap>::FutureAndRequestId> sent;
    };
    std::vector<Call> calls;
    for (const char* port : { "global_service", "local_service" })
    {
        std::string service = getInput<std::string>(port).value_or("");
        if (!service.empty())
        {
            auto client = client_node->create_client<ClearEntireCostmap>(service);
            calls.push_back({ std::move(service), std::move(client), std::nullopt });
        }
    }

    bool               all_cleared = true;
    std::vector<Call*> waiting;
    for (Call& call : calls)
    {
        while (!call.client->wait_for_service(slice()) && !expired()) {}
        if (!call.client->service_is_ready())
        {
            RCLCPP_ERROR(node_->get_logger(), "service '%s' never appeared", call.service.c_str());
            all_cleared = false;
            continue;
        }
        call.sent.emplace(
            call.client->async_send_request(std::make_shared<ClearEntireCostmap::Request>()));
        waiting.push_back(&call);
    }
    for (Call* call : waiting)
    {
        rclcpp::FutureReturnCode code{};
        do
        {
            code = rclcpp::spin_until_future_complete(client_node, call->sent->future, slice());
        } while (code == rclcpp::FutureReturnCode::TIMEOUT && !expired());
        if (code != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "call to '%s' did not return", call->service.c_str());
            call->client->remove_pending_request(call->sent->request_id);
            all_cleared = false;
        }
    }

    if (!all_cleared)
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] continuing with a costmap uncleared", name().c_str());
    }
    return BT::NodeStatus::SUCCESS;
}

}  // namespace g1_orchestration
