/**
 * @file look_for.cpp
 * @brief Ports and ticks for the LookFor and StopLooking leaves.
 */

#include "g1_orchestration/skills/look_for.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <memory>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/parameter.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{
namespace
{

using SetParameters = rcl_interfaces::srv::SetParameters;

/// How long a phrase write gets before the wait for the objects starts without it.
constexpr double kServiceBudgetS = 5.0;

/// What g1_object_geometry publishes and every skill above it expects.
rclcpp::QoS objectsQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

std::chrono::steady_clock::duration seconds(double value)
{
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(value));
}

/// g1_perception's slugify, which turns a detector label into an /objects id. Mirrored rather than
/// linked, since the perception package would pull OpenCV into the tree.
std::string slugOf(std::string_view text)
{
    std::string out;
    for (const char character : text)
    {
        const auto raw = static_cast<unsigned char>(character);
        if (std::isalnum(raw) != 0)
        {
            out.push_back(static_cast<char>(std::tolower(raw)));
        }
        else if (!out.empty() && out.back() != '_')
        {
            out.push_back('_');
        }
    }
    while (!out.empty() && out.back() == '_')
    {
        out.pop_back();
    }
    return out;
}

/// The id /objects will carry for an entry: the slug of its `id` half, or of the whole phrase.
std::string idOf(std::string_view entry) { return slugOf(entry.substr(0, entry.find('='))); }

}  // namespace

LookFor::LookFor(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : BT::StatefulActionNode(name, config)
  , node_(std::move(context.node))
{}

BT::PortsList LookFor::providedPorts()
{
    return {
        BT::InputPort<std::string>(
            "objects",
            "Comma-separated objects to ask the detector for and wait on: an id, or 'id=phrase' "
            "when the wording that finds it differs from the name the tree uses."),
        BT::InputPort<std::string>(
            "also",
            "",
            "More objects to ask for, in the same form, without waiting on them: ones a later step "
            "needs to see, like the object Place confirms after letting go."),
        BT::InputPort<std::string>(
            "detector",
            "/g1_detector",
            "Node whose `phrases` to write. Empty waits without narrowing anything."),
        ports::serviceTimeout(
            20.0,
            "Seconds to wait for the objects once the phrases are written. The write gets up to "
            "5 s of its own."),
    };
}

BT::NodeStatus LookFor::onStart()
{
    const auto objects = getInput<std::string>("objects");
    phrases_.clear();
    wanted_.clear();
    if (objects.has_value())
    {
        for (const auto& part : BT::splitString(objects.value(), ','))
        {
            std::string entry = BT::convertFromString<std::string>(part);
            if (!entry.empty())
            {
                wanted_.push_back(idOf(entry));
                phrases_.push_back(std::move(entry));
            }
        }
    }
    if (wanted_.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] needs at least one object", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    // Named first: splitString returns views into its argument.
    const std::string also = getInput<std::string>("also").value_or("");
    for (const auto& part : BT::splitString(also, ','))
    {
        std::string entry = BT::convertFromString<std::string>(part);
        if (!entry.empty())
        {
            phrases_.push_back(std::move(entry));
        }
    }
    timeout_s_ = getInput<double>("timeout_s").value_or(20.0);
    detector_  = getInput<std::string>("detector").value_or("");

    client_node_ = makeClientNode("g1_look_for_client");
    executor_    = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(client_node_);
    seen_.clear();
    objects_sub_ = client_node_->create_subscription<vision_msgs::msg::Detection3DArray>(
        "/objects",
        objectsQos(),
        [this](const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg) {
            seen_.clear();
            for (const auto& detection : msg->detections)
            {
                seen_.insert(detection.id);
            }
        });

    if (detector_.empty())
    {
        finishWrite(/*taken=*/true);
    }
    else
    {
        set_phrases_    = client_node_->create_client<SetParameters>(detector_ + "/set_parameters");
        write_          = Write::kWaitingForService;
        write_deadline_ = Clock::now() + seconds(std::min(timeout_s_, kServiceBudgetS));
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LookFor::onRunning()
{
    executor_->spin_some();
    pumpWrite();
    if (std::ranges::all_of(wanted_, [this](const std::string& id) { return seen_.contains(id); }))
    {
        reset();
        return BT::NodeStatus::SUCCESS;
    }
    if (write_ != Write::kDone || Clock::now() < deadline_)
    {
        return BT::NodeStatus::RUNNING;
    }
    for (const std::string& id : wanted_)
    {
        if (!seen_.contains(id))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] nothing called '%s' on /objects after %.1f s",
                name().c_str(),
                id.c_str(),
                timeout_s_);
        }
    }
    reset();
    return BT::NodeStatus::FAILURE;
}

void LookFor::onHalted() { reset(); }

void LookFor::pumpWrite()
{
    if (write_ == Write::kWaitingForService && set_phrases_->service_is_ready())
    {
        auto request = std::make_shared<SetParameters::Request>();
        request->parameters.push_back(rclcpp::Parameter("phrases", phrases_).to_parameter_msg());
        write_ = Write::kAwaitingReply;
        set_phrases_->async_send_request(
            request,
            // rclcpp matches the callback signature exactly, so the future is taken by value.
            // NOLINTNEXTLINE(performance-unnecessary-value-param)
            [this](rclcpp::Client<SetParameters>::SharedFuture reply) {
                const auto& response = reply.get();
                finishWrite(
                    response != nullptr && !response->results.empty() &&
                    response->results.front().successful);
            });
        return;
    }
    if (write_ != Write::kDone && Clock::now() >= write_deadline_)
    {
        finishWrite(/*taken=*/false);
    }
}

void LookFor::finishWrite(bool taken)
{
    // Not a failure: the promise is the objects on /objects, and a stand-in detector or one
    // already asking for them needs no write. The wait says so by name if they never show.
    if (!taken)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] %s would not take the phrases; waiting on /objects anyway",
            name().c_str(),
            detector_.c_str());
    }
    write_    = Write::kDone;
    deadline_ = Clock::now() + seconds(timeout_s_);
}

void LookFor::reset()
{
    objects_sub_.reset();
    set_phrases_.reset();
    if (executor_ && client_node_)
    {
        executor_->remove_node(client_node_);
    }
    executor_.reset();
    client_node_.reset();
}

StopLooking::StopLooking(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : ServiceLeaf(name, config, std::move(context))
{}

BT::PortsList StopLooking::providedPorts()
{
    return {
        BT::InputPort<std::string>("detector", "/g1_detector", "Node whose `phrases` to empty."),
        ports::serviceTimeout(kServiceBudgetS, "Budget for the write."),
    };
}

BT::NodeStatus StopLooking::tick()
{
    const std::string detector = getInput<std::string>("detector").value_or("/g1_detector");
    auto              request  = std::make_shared<SetParameters::Request>();
    request->parameters.push_back(
        rclcpp::Parameter("phrases", std::vector<std::string>{}).to_parameter_msg());

    const auto response = callService<SetParameters>(
        makeClientNode("g1_stop_looking_client"),
        detector + "/set_parameters",
        request,
        getInput<double>("timeout_s").value_or(kServiceBudgetS));
    // Succeeds either way: a detector that will not take the write is either not running or
    // ignoring it, and neither should stop the mission.
    if (response == nullptr || response->results.empty() || !response->results.front().successful)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] %s would not take an empty phrase list; perception may still be running",
            name().c_str(),
            detector.c_str());
    }
    return BT::NodeStatus::SUCCESS;
}

}  // namespace g1_orchestration
