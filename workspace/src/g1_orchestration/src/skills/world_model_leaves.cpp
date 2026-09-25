/**
 * @file world_model_leaves.cpp
 * @brief Ports and ticks for the world model leaves.
 */

#include "g1_orchestration/skills/world_model_leaves.hpp"

#include <behaviortree_cpp/decorators/loop_node.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <g1_msgs/srv/get_approach_pose.hpp>
#include <g1_msgs/srv/next_viewpoint.hpp>
#include <g1_msgs/srv/report_viewpoint.hpp>
#include <memory>
#include <mutex>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

namespace
{

constexpr const char* kWorldModel = "/g1_world_model";

double yawOf(const geometry_msgs::msg::Quaternion& q)
{
    return std::atan2(2.0 * ((q.w * q.z) + (q.x * q.y)), 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))));
}

/// One TF listener for every TurnTo in the process, on the tree's node.
std::shared_ptr<tf2_ros::Buffer> sharedTf(const rclcpp::Node::SharedPtr& node)
{
    static std::mutex                                  mutex;
    static std::shared_ptr<tf2_ros::Buffer>            buffer;
    static std::shared_ptr<tf2_ros::TransformListener> listener;
    const std::lock_guard<std::mutex>                  lock(mutex);
    if (buffer == nullptr)
    {
        buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    }
    return buffer;
}

}  // namespace

// --- NextViewpoint --------------------------------------------------------------------------

NextViewpoint::NextViewpoint(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : ServiceLeaf(name, config, std::move(context))
{}

BT::PortsList NextViewpoint::providedPorts()
{
    return {
        ports::serviceTimeout(30.0, "How long to wait for a map and a pose before giving up."),
        BT::InputPort<std::string>(
            "mode",
            "coverage",
            "'frontier' to close the map, 'coverage' to make the camera see it."),
        BT::InputPort<std::string>(
            "service",
            std::string(kWorldModel) + "/next_viewpoint",
            "The world model's service."),
        BT::OutputPort<Station>("goal", "Where to stand, facing the first heading."),
        BT::OutputPort<BT::SharedQueue<double>>("headings", "Map-frame yaws to face, in order."),
        BT::OutputPort<int>("viewpoint_id", "For ReportViewpoint."),
        BT::OutputPort<std::string>("room_id", "The room the viewpoint stands in."),
        BT::OutputPort<std::string>("outcome", "'viewpoint', 'done' or 'error'."),
    };
}

BT::NodeStatus NextViewpoint::tick()
{
    using Service = g1_msgs::srv::NextViewpoint;

    const double timeout_s = getInput<double>("timeout_s").value_or(30.0);
    auto         request   = std::make_shared<Service::Request>();
    request->mode          = getInput<std::string>("mode").value_or("coverage");
    const std::string service =
        getInput<std::string>("service").value_or(std::string(kWorldModel) + "/next_viewpoint");

    auto       client = makeClientNode("g1_next_viewpoint_client");
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    for (;;)
    {
        const auto response = callService<Service>(client, service, request, timeout_s);
        if (response == nullptr)
        {
            setOutput("outcome", std::string("error"));
            return BT::NodeStatus::FAILURE;
        }
        if (response->status == Service::Response::STATUS_DONE)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "[%s] done: %s",
                name().c_str(),
                response->message.c_str());
            setOutput("outcome", std::string("done"));
            return BT::NodeStatus::FAILURE;
        }
        if (response->status == Service::Response::STATUS_VIEWPOINT)
        {
            const auto& pose = response->pose.pose;
            setOutput("goal", Station{ pose.position.x, pose.position.y, yawOf(pose.orientation) });
            auto headings = std::make_shared<std::deque<double>>(
                response->headings.begin(),
                response->headings.end());
            setOutput("headings", headings);
            setOutput("viewpoint_id", static_cast<int>(response->viewpoint_id));
            setOutput("room_id", response->room_id);
            setOutput("outcome", std::string("viewpoint"));
            return BT::NodeStatus::SUCCESS;
        }
        // Unavailable: no map or no pose yet. Worth waiting for at the start of a run.
        if (std::chrono::steady_clock::now() >= deadline)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[%s] world model unavailable: %s",
                name().c_str(),
                response->message.c_str());
            setOutput("outcome", std::string("error"));
            return BT::NodeStatus::FAILURE;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// --- ReportViewpoint ------------------------------------------------------------------------

ReportViewpoint::ReportViewpoint(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : ServiceLeaf(name, config, std::move(context))
{}

BT::PortsList ReportViewpoint::providedPorts()
{
    return {
        ports::serviceTimeout(5.0, "Service budget."),
        BT::InputPort<int>("viewpoint_id", "From NextViewpoint."),
        BT::InputPort<bool>("reached", true, "Whether navigation got there."),
        BT::InputPort<std::string>(
            "service",
            std::string(kWorldModel) + "/report_viewpoint",
            "The world model's service."),
    };
}

BT::NodeStatus ReportViewpoint::tick()
{
    using Service         = g1_msgs::srv::ReportViewpoint;
    auto request          = std::make_shared<Service::Request>();
    request->viewpoint_id = static_cast<std::uint32_t>(getInput<int>("viewpoint_id").value_or(0));
    request->reached      = getInput<bool>("reached").value_or(true);
    const auto response   = callService<Service>(
        makeClientNode("g1_report_viewpoint_client"),
        getInput<std::string>("service").value_or(std::string(kWorldModel) + "/report_viewpoint"),
        request,
        getInput<double>("timeout_s").value_or(5.0));
    if (response == nullptr)
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] report not delivered", name().c_str());
    }
    return BT::NodeStatus::SUCCESS;
}

// --- ResolveTarget --------------------------------------------------------------------------

ResolveTarget::ResolveTarget(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : ServiceLeaf(name, config, std::move(context))
{}

BT::PortsList ResolveTarget::providedPorts()
{
    return {
        ports::serviceTimeout(10.0, "Service budget."),
        BT::InputPort<std::string>("target", "An object id, a room, or a label to search for."),
        BT::InputPort<std::string>("room", "", "Narrows a label search to one room."),
        BT::InputPort<double>(
            "standoff",
            0.0,
            "Distance from the target's footprint, m; 0 for the default."),
        BT::InputPort<std::string>(
            "service",
            std::string(kWorldModel) + "/get_approach_pose",
            "The world model's service."),
        BT::OutputPort<Station>("goal", "A reachable pose facing the target."),
        BT::OutputPort<std::string>("target_id", "What it resolved to."),
    };
}

BT::NodeStatus ResolveTarget::tick()
{
    using Service      = g1_msgs::srv::GetApproachPose;
    auto       request = std::make_shared<Service::Request>();
    const auto target  = getInput<std::string>("target");
    if (!target || target->empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] no target given", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    request->target     = *target;
    request->room       = getInput<std::string>("room").value_or("");
    request->standoff   = static_cast<float>(getInput<double>("standoff").value_or(0.0));
    const auto response = callService<Service>(
        makeClientNode("g1_resolve_target_client"),
        getInput<std::string>("service").value_or(std::string(kWorldModel) + "/get_approach_pose"),
        request,
        getInput<double>("timeout_s").value_or(10.0));
    if (response == nullptr || !response->success)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] cannot resolve '%s': %s",
            name().c_str(),
            target->c_str(),
            response == nullptr ? "no answer" : response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }
    const auto& pose = response->pose.pose;
    setOutput("goal", Station{ pose.position.x, pose.position.y, yawOf(pose.orientation) });
    setOutput("target_id", response->target_id);
    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] '%s' -> %s at (%.2f, %.2f)",
        name().c_str(),
        target->c_str(),
        response->message.c_str(),
        pose.position.x,
        pose.position.y);
    return BT::NodeStatus::SUCCESS;
}

// --- SaveWorld ------------------------------------------------------------------------------

SaveWorld::SaveWorld(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : ServiceLeaf(name, config, std::move(context))
{}

BT::PortsList SaveWorld::providedPorts()
{
    return {
        ports::serviceTimeout(20.0, "Service budget."),
        BT::InputPort<std::string>(
            "service",
            std::string(kWorldModel) + "/save",
            "The world model's service."),
    };
}

BT::NodeStatus SaveWorld::tick()
{
    using Service       = std_srvs::srv::Trigger;
    const auto response = callService<Service>(
        makeClientNode("g1_save_world_client"),
        getInput<std::string>("service").value_or(std::string(kWorldModel) + "/save"),
        std::make_shared<Service::Request>(),
        getInput<double>("timeout_s").value_or(20.0));
    if (response == nullptr || !response->success)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] %s",
            name().c_str(),
            response == nullptr ? "no answer" : response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", name().c_str(), response->message.c_str());
    return BT::NodeStatus::SUCCESS;
}

// --- TurnTo ---------------------------------------------------------------------------------

TurnTo::TurnTo(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : RosActionNode(name, config, std::move(context), "/spin")
{
    sharedTf(node_);
}

BT::PortsList TurnTo::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("yaw", "Heading to face, rad, in `frame`."),
        BT::InputPort<std::string>("frame", "map", "Frame of `yaw`."),
        BT::InputPort<std::string>("base_frame", "base_footprint", "The robot's base frame."),
    });
}

bool TurnTo::fillGoal(Goal& goal)
{
    const auto yaw = getInput<double>("yaw");
    if (!yaw)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name().c_str(), yaw.error().c_str());
        return false;
    }
    const std::string frame   = getInput<std::string>("frame").value_or("map");
    const std::string base    = getInput<std::string>("base_frame").value_or("base_footprint");
    double            now_yaw = 0.0;
    try
    {
        now_yaw = yawOf(
            sharedTf(node_)->lookupTransform(frame, base, tf2::TimePointZero).transform.rotation);
    }
    catch (const tf2::TransformException& error)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] no %s -> %s: %s",
            name().c_str(),
            frame.c_str(),
            base.c_str(),
            error.what());
        return false;
    }
    const double turn = std::remainder(*yaw - now_yaw, 2.0 * M_PI);
    goal.target_yaw   = static_cast<float>(turn);
    // Generous: the controller caps yaw rate below what the spin behavior asks for.
    goal.time_allowance = rclcpp::Duration::from_seconds(8.0 + (std::abs(turn) / 0.4));
    return true;
}

BT::NodeStatus TurnTo::judgeResult(const WrappedResult& result)
{
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] spin did not finish", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

}  // namespace g1_orchestration
