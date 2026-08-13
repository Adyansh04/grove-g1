#include "g1_orchestration/skills/clear_octomap.hpp"

#include <memory>
#include <std_srvs/srv/empty.hpp>
#include <string>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

ClearOctomap::ClearOctomap(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : ServiceLeaf(name, config, std::move(context))
{}

BT::PortsList ClearOctomap::providedPorts()
{
    return {
        ports::serviceTimeout(5.0, "Service budget."),
        BT::InputPort<std::string>(
            "service",
            "/clear_octomap",
            "move_group's octomap clear service. Empty skips it."),
    };
}

BT::NodeStatus ClearOctomap::tick()
{
    const double      timeout_s = getInput<double>("timeout_s").value_or(5.0);
    const std::string service   = getInput<std::string>("service").value_or("");
    if (service.empty())
    {
        return BT::NodeStatus::SUCCESS;
    }

    auto       client_node = makeClientNode("g1_clear_octomap_client");
    const auto response    = callService<std_srvs::srv::Empty>(
        client_node,
        service,
        std::make_shared<std_srvs::srv::Empty::Request>(),
        timeout_s);
    if (response == nullptr)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] continuing with the octomap uncleared",
            name().c_str());
    }
    return BT::NodeStatus::SUCCESS;
}

}  // namespace g1_orchestration
