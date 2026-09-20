/**
 * @file look_for.cpp
 * @brief Ports and tick for the LookFor leaf.
 */

#include "g1_orchestration/skills/look_for.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/parameter.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{
namespace
{

/// Reliable, keep-last-1: what g1_object_geometry publishes and every skill above it expects.
rclcpp::QoS objectsQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

/// The id a tree says, out of an `id=phrase` entry or, without one, out of the phrase's own slug.
std::string idOf(const std::string& entry)
{
    const std::size_t split = entry.find('=');
    return split == std::string::npos ? entry : entry.substr(0, split);
}

}  // namespace

LookFor::LookFor(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : ServiceLeaf(name, config, std::move(context))
{}

BT::PortsList LookFor::providedPorts()
{
    return {
        BT::InputPort<std::string>(
            "objects",
            "Comma-separated objects to look for. Each is an id the detector is asked for and "
            "waited on by name, or 'id=phrase' when the wording that finds it reliably is not "
            "the name the tree uses, e.g. 'red_block=bright red plastic block'."),
        BT::InputPort<std::string>(
            "detector",
            "/g1_detector",
            "Node whose `phrases` to write. Empty waits without narrowing anything."),
        ports::serviceTimeout(
            20.0,
            "Budget for the write and for the objects to appear. A detector pass is ~0.5 s per "
            "phrase and the tracker needs a few before it trusts a track."),
    };
}

BT::NodeStatus LookFor::tick()
{
    const auto objects = getInput<std::string>("objects");
    if (!objects.has_value() || objects->empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] needs at least one object", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // The entries go to the detector as written; only the ids are waited on. A detector needs a
    // wordier phrase than a tree wants to say, and "id=phrase" lets each have its own.
    std::vector<std::string> entries;
    std::vector<std::string> wanted;
    for (const auto& part : BT::splitString(objects.value(), ','))
    {
        std::string entry = BT::convertFromString<std::string>(part);
        if (entry.empty())
        {
            continue;
        }
        wanted.push_back(idOf(entry));
        entries.push_back(std::move(entry));
    }

    const double      timeout_s   = getInput<double>("timeout_s").value_or(20.0);
    const std::string detector    = getInput<std::string>("detector").value_or("");
    auto              client_node = makeClientNode("g1_look_for_client");

    using Clock         = std::chrono::steady_clock;
    const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                             std::chrono::duration<double>(timeout_s));

    if (!detector.empty())
    {
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        request->parameters.push_back(rclcpp::Parameter("phrases", entries).to_parameter_msg());

        const auto response = callService<rcl_interfaces::srv::SetParameters>(
            client_node,
            detector + "/set_parameters",
            request,
            std::chrono::duration<double>(std::max(deadline - Clock::now(), Clock::duration::zero()))
                .count());
        if (response == nullptr || response->results.empty() ||
            !response->results.front().successful)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] %s would not take the phrases",
                name().c_str(),
                detector.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    // Wait for the objects rather than trusting the write: the detector only narrows on its next
    // pass, and until the tracker settles a phrase can hold two tracks, which costs it the bare
    // alias that Pick resolves by. Seeing the alias is the proof the write took effect.
    std::set<std::string> seen;
    auto subscription = client_node->create_subscription<vision_msgs::msg::Detection3DArray>(
        "/objects",
        objectsQos(),
        [&seen](const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg) {
            seen.clear();
            for (const auto& detection : msg->detections)
            {
                seen.insert(detection.id);
            }
        });

    while (Clock::now() < deadline)
    {
        rclcpp::spin_some(client_node);
        if (std::all_of(wanted.begin(), wanted.end(), [&seen](const std::string& id) {
                return seen.contains(id);
            }))
        {
            return BT::NodeStatus::SUCCESS;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    for (const std::string& id : wanted)
    {
        if (!seen.contains(id))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] nothing called '%s' on /objects after %.1f s",
                name().c_str(),
                id.c_str(),
                timeout_s);
        }
    }
    return BT::NodeStatus::FAILURE;
}

}  // namespace g1_orchestration
