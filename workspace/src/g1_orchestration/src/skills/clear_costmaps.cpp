/**
 * @file clear_costmaps.cpp
 * @brief Ports and tick for the ClearCostmaps leaf.
 */

#include "g1_orchestration/skills/clear_costmaps.hpp"

#include <memory>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <string>

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

BT::NodeStatus ClearCostmaps::tick()
{
    using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;

    const double timeout_s   = getInput<double>("timeout_s").value_or(5.0);
    auto         client_node = makeClientNode("g1_clear_costmaps_client");

    bool all_cleared = true;
    for (const char* port : { "global_service", "local_service" })
    {
        const std::string service = getInput<std::string>(port).value_or("");
        if (service.empty())
        {
            continue;
        }
        const auto response = callService<ClearEntireCostmap>(
            client_node,
            service,
            std::make_shared<ClearEntireCostmap::Request>(),
            timeout_s);
        if (response == nullptr)
        {
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
