/**
 * @file server_fleet.cpp
 * @brief Cancelling and counting goals on a list of action servers.
 */

#include "g1_orchestration/server_fleet.hpp"

#include <algorithm>
#include <exception>
#include <format>
#include <stdexcept>
#include <thread>

namespace g1_orchestration
{

namespace
{

/// What a server's status topic uses: reliable and latched, so a late subscriber still learns
/// what the server is holding.
rclcpp::QoS statusQos() { return rclcpp::QoS(1).reliable().transient_local(); }

bool inFlight(int8_t status)
{
    using Status = action_msgs::msg::GoalStatus;
    return status == Status::STATUS_ACCEPTED || status == Status::STATUS_EXECUTING ||
           status == Status::STATUS_CANCELING;
}

}  // namespace

ServerFleet::ServerFleet(
    rclcpp::Node& node, const std::vector<std::string>& specs,
    const rclcpp::CallbackGroup::SharedPtr& group)
  : logger_(node.get_logger())
{
    for (const std::string& spec : specs)
    {
        const std::size_t colon = spec.find(':');
        if (colon == std::string::npos || colon == 0 || colon + 1 == spec.size())
        {
            throw std::invalid_argument("'" + spec + "' is not /action/name:package/action/Type");
        }
        auto server  = std::make_unique<Server>();
        server->name = spec.substr(0, colon);
        try
        {
            server->client = rclcpp_action::create_generic_client(
                &node,
                server->name,
                spec.substr(colon + 1),
                group);
        }
        catch (const std::exception& e)
        {
            throw std::invalid_argument(std::format("cannot follow {}: {}", spec, e.what()));
        }
        rclcpp::SubscriptionOptions options;
        options.callback_group = group;
        Server* raw            = server.get();
        server->status         = node.create_subscription<action_msgs::msg::GoalStatusArray>(
            server->name + "/_action/status",
            statusQos(),
            [raw](const action_msgs::msg::GoalStatusArray::ConstSharedPtr& msg) {
                raw->in_flight = static_cast<int>(std::ranges::count_if(
                    msg->status_list,
                    [](const action_msgs::msg::GoalStatus& s) { return inFlight(s.status); }));
            },
            options);
        servers_.push_back(std::move(server));
    }
}

std::size_t ServerFleet::cancelAll()
{
    std::size_t asked = 0;
    for (const auto& server : servers_)
    {
        try
        {
            // A request nobody answers stays pending in the client, so only ask a server that is
            // there.
            if (server->client->action_server_is_ready())
            {
                (void)server->client->async_cancel_all_goals();
                ++asked;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(
                logger_,
                "could not ask %s to cancel its goals: %s",
                server->name.c_str(),
                e.what());
        }
    }
    return asked;
}

std::vector<std::string> ServerFleet::busy() const
{
    std::vector<std::string> out;
    for (const auto& server : servers_)
    {
        // A server that died mid-goal leaves its last status behind; it is no longer holding anything.
        if (server->in_flight > 0 && server->status->get_publisher_count() > 0)
        {
            out.push_back(std::format("{} ({} goals)", server->name, server->in_flight.load()));
        }
    }
    return out;
}

std::vector<std::string> ServerFleet::settle(std::chrono::duration<double> timeout) const
{
    constexpr auto kPoll    = std::chrono::milliseconds(20);
    const auto     deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout);
    std::vector<std::string> still = busy();
    while (!still.empty() && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(kPoll);
        still = busy();
    }
    return still;
}

std::vector<std::string> ServerFleet::names() const
{
    std::vector<std::string> out;
    out.reserve(servers_.size());
    for (const auto& server : servers_)
    {
        out.push_back(server->name);
    }
    return out;
}

}  // namespace g1_orchestration
