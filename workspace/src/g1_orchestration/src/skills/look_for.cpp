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

/// `red_block` is the slug of the phrase `red block`, so the inverse recovers what to ask for.
std::string phraseOf(std::string id)
{
    std::replace(id.begin(), id.end(), '_', ' ');
    return id;
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
            "Comma-separated object ids to look for, e.g. 'red_block,brown_box'. Each is also "
            "the phrase the detector is asked for, with underscores as spaces."),
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

    std::vector<std::string> wanted;
    for (const auto& part : BT::splitString(objects.value(), ','))
    {
        std::string id = BT::convertFromString<std::string>(part);
        if (!id.empty())
        {
            wanted.push_back(std::move(id));
        }
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
        std::vector<std::string> phrases;
        phrases.reserve(wanted.size());
        std::transform(wanted.begin(), wanted.end(), std::back_inserter(phrases), phraseOf);
        request->parameters.push_back(rclcpp::Parameter("phrases", phrases).to_parameter_msg());

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
