#include "g1_orchestration/skill_nodes.hpp"

#include <behaviortree_cpp/action_node.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace BT
{
template <>
g1_orchestration::Station convertFromString(StringView text)
{
    const std::vector<StringView> parts = splitString(text, ';');
    if (parts.size() != 3)
    {
        throw RuntimeError("a station is 'x;y;yaw', got: ", std::string(text));
    }
    g1_orchestration::Station station;
    station.x   = convertFromString<double>(parts[0]);
    station.y   = convertFromString<double>(parts[1]);
    station.yaw = convertFromString<double>(parts[2]);
    return station;
}

template <>
g1_orchestration::Point3 convertFromString(StringView text)
{
    const std::vector<StringView> parts = splitString(text, ';');
    if (parts.size() != 3)
    {
        throw RuntimeError("a point is 'x;y;z', got: ", std::string(text));
    }
    g1_orchestration::Point3 point;
    point.x = convertFromString<double>(parts[0]);
    point.y = convertFromString<double>(parts[1]);
    point.z = convertFromString<double>(parts[2]);
    return point;
}
}  // namespace BT

namespace g1_orchestration
{

namespace
{

geometry_msgs::msg::PoseStamped toPose(const Station& station, const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = station.x;
    pose.pose.position.y = station.y;

    tf2::Quaternion heading;
    heading.setRPY(0.0, 0.0, station.yaw);
    pose.pose.orientation = tf2::toMsg(heading);
    return pose;
}

/// Shared by every leaf whose result carries `success` and `message`.
template <typename ResultT>
BT::NodeStatus
judgeSkillResult(const rclcpp::Logger& logger, const std::string& name, const ResultT& wrapped)
{
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        // An ABORTED goal still carries the server's result, and that message is the only
        // place the reason exists -- every abort in these servers writes a phase-prefixed
        // explanation into it. Logging just "did not complete" threw all of it away at exactly
        // the moment it was wanted, and cost a full mission re-run per diagnosis.
        const std::string why =
            wrapped.result && !wrapped.result->message.empty() ? wrapped.result->message : "";
        RCLCPP_ERROR(
            logger,
            "[%s] did not complete%s%s",
            name.c_str(),
            why.empty() ? "" : ": ",
            why.c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (!wrapped.result->success)
    {
        // The skill's own message names the phase that failed, which is the part worth
        // surfacing: "the pick failed" is not actionable, "grasp: the hand did not close" is.
        RCLCPP_ERROR(logger, "[%s] %s", name.c_str(), wrapped.result->message.c_str());
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger, "[%s] %s", name.c_str(), wrapped.result->message.c_str());
    return BT::NodeStatus::SUCCESS;
}

}  // namespace

NavigateToPose::NavigateToPose(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : RosActionNode(name, config, context, "/navigate_to_pose")
{}

BT::PortsList NavigateToPose::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<Station>("goal", "Where to drive to, as 'x;y;yaw'."),
        BT::InputPort<std::string>("frame_id", "map", "Frame the goal is expressed in."),
    });
}

bool NavigateToPose::fillGoal(Goal& goal)
{
    const auto station = getInput<Station>("goal");
    if (!station)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name().c_str(), station.error().c_str());
        return false;
    }
    goal.pose = toPose(*station, getInput<std::string>("frame_id").value_or("map"));
    return true;
}

BT::NodeStatus NavigateToPose::judgeResult(const WrappedResult& result)
{
    // NavigateToPose's result is empty: reaching the goal is reported by the result CODE and
    // nothing else, unlike this stack's own skills.
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] did not reach the goal", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] arrived", name().c_str());
    return BT::NodeStatus::SUCCESS;
}

ApproachObject::ApproachObject(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : RosActionNode(name, config, context, "/g1_base_approach/approach_object")
{}

BT::PortsList ApproachObject::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("object_id", "Must match a class_id published on /objects."),
        BT::InputPort<std::string>("arm", "right", "'left' or 'right'."),
        BT::InputPort<double>(
            "working_yaw",
            "Heading to hold while approaching. Pass the same yaw the staging goal used."),
        BT::InputPort<double>("timeout_s", 0.0, "0 uses the server's own default."),
    });
}

bool ApproachObject::fillGoal(Goal& goal)
{
    const auto object_id = getInput<std::string>("object_id");
    if (!object_id || object_id->empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] needs an object_id", name().c_str());
        return false;
    }
    // Required, with no default. A missing heading would silently mean "face +x", which is a
    // valid yaw and almost never the right one -- the skill would approach square to nothing
    // and the failure would look like bad geometry rather than a missing port.
    const auto working_yaw = getInput<double>("working_yaw");
    if (!working_yaw)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[%s] needs working_yaw: %s",
            name().c_str(),
            working_yaw.error().c_str());
        return false;
    }
    goal.object_id           = *object_id;
    goal.arm                 = getInput<std::string>("arm").value_or("right");
    goal.working_yaw         = *working_yaw;
    goal.use_current_heading = false;
    goal.timeout_s           = getInput<double>("timeout_s").value_or(0.0);
    return true;
}

BT::NodeStatus ApproachObject::judgeResult(const WrappedResult& result)
{
    return judgeSkillResult(node_->get_logger(), name(), result);
}

Retreat::Retreat(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : RosActionNode(name, config, context, "/g1_base_approach/retreat")
{}

BT::PortsList Retreat::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("distance", 0.6, "How far to reverse, in metres."),
        BT::InputPort<double>("timeout_s", 0.0, "0 uses the server's own default."),
    });
}

bool Retreat::fillGoal(Goal& goal)
{
    goal.distance_m = getInput<double>("distance").value_or(0.6);
    goal.timeout_s  = getInput<double>("timeout_s").value_or(0.0);
    if (goal.distance_m <= 0.0)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] distance must be positive", name().c_str());
        return false;
    }
    return true;
}

BT::NodeStatus Retreat::judgeResult(const WrappedResult& result)
{
    return judgeSkillResult(node_->get_logger(), name(), result);
}

Pick::Pick(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : RosActionNode(name, config, context, "/g1_manipulation_server/pick")
{}

BT::PortsList Pick::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("object_id", "Must match a class_id published on /objects."),
        BT::InputPort<std::string>("arm", "right", "'left' or 'right'."),
    });
}

bool Pick::fillGoal(Goal& goal)
{
    const auto object_id = getInput<std::string>("object_id");
    if (!object_id || object_id->empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] needs an object_id", name().c_str());
        return false;
    }
    goal.object_id = *object_id;
    goal.arm       = getInput<std::string>("arm").value_or("right");
    return true;
}

BT::NodeStatus Pick::judgeResult(const WrappedResult& result)
{
    return judgeSkillResult(node_->get_logger(), name(), result);
}

Place::Place(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : RosActionNode(name, config, context, "/g1_manipulation_server/place")
{}

BT::PortsList Place::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>(
            "surface",
            "",
            "Detected surface to place ON TOP OF. Preferred over 'target'."),
        BT::InputPort<Point3>("target", "Where the OBJECT should end up, as 'x;y;z'."),
        BT::InputPort<std::string>("arm", "right", "'left' or 'right'."),
        BT::InputPort<std::string>(
            "frame_id",
            "",
            "Frame of the target. Empty means the server's planning frame."),
    });
}

bool Place::fillGoal(Goal& goal)
{
    goal.arm = getInput<std::string>("arm").value_or("right");

    // A named surface beats a coordinate, and the reason is worth knowing before writing a tree
    // the other way. A coordinate here is in the MAP frame, while ApproachObject parks the base
    // against /objects, which is published in ODOM. Those agree only as well as AMCL does, and
    // it was measured 0.23 m out mid-mission against an arm window of 0.04 m.
    goal.surface_object_id = getInput<std::string>("surface").value_or("");
    if (!goal.surface_object_id.empty())
    {
        return true;
    }

    const auto target = getInput<Point3>("target");
    if (!target)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name().c_str(), target.error().c_str());
        return false;
    }
    // Position only. How the object is oriented when it lands is the server's business: it
    // knows how the object is held and this tree does not.
    goal.pose.header.frame_id    = getInput<std::string>("frame_id").value_or("");
    goal.pose.pose.position.x    = target->x;
    goal.pose.pose.position.y    = target->y;
    goal.pose.pose.position.z    = target->z;
    goal.pose.pose.orientation.w = 1.0;
    return true;
}

BT::NodeStatus Place::judgeResult(const WrappedResult& result)
{
    return judgeSkillResult(node_->get_logger(), name(), result);
}

SetArmPosture::SetArmPosture(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : RosActionNode(name, config, context, "/g1_manipulation_server/set_arm_posture")
{}

BT::PortsList SetArmPosture::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("group", "A group in g1.srdf, e.g. right_arm."),
        BT::InputPort<std::string>("named_target", "A group_state of that group, e.g. tucked."),
    });
}

bool SetArmPosture::fillGoal(Goal& goal)
{
    const auto group        = getInput<std::string>("group");
    const auto named_target = getInput<std::string>("named_target");
    if (!group || !named_target)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] needs both group and named_target", name().c_str());
        return false;
    }
    goal.group        = *group;
    goal.named_target = *named_target;
    return true;
}

BT::NodeStatus SetArmPosture::judgeResult(const WrappedResult& result)
{
    return judgeSkillResult(node_->get_logger(), name(), result);
}

ClearCostmaps::ClearCostmaps(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : BT::SyncActionNode(name, config)
  , node_(std::move(context.node))
{}

BT::PortsList ClearCostmaps::providedPorts()
{
    return { BT::InputPort<double>("timeout_s", 5.0, "Per-costmap service budget.") };
}

BT::NodeStatus ClearCostmaps::tick()
{
    using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;

    const double timeout_s = getInput<double>("timeout_s").value_or(5.0);
    // A node of its own, spun here. The executor already owns the tree's node, and
    // spin_until_future_complete on a node an executor holds throws rather than waiting --
    // the same reason arm_authority builds one.
    auto client_node = std::make_shared<rclcpp::Node>("g1_clear_costmaps_client");

    bool all_cleared = true;
    for (const char* service : { "/global_costmap/clear_entirely_global_costmap",
                                 "/local_costmap/clear_entirely_local_costmap" })
    {
        auto client = client_node->create_client<ClearEntireCostmap>(service);
        if (!client->wait_for_service(std::chrono::duration<double>(timeout_s)))
        {
            RCLCPP_WARN(node_->get_logger(), "[%s] no '%s'", name().c_str(), service);
            all_cleared = false;
            continue;
        }
        auto future = client->async_send_request(std::make_shared<ClearEntireCostmap::Request>());
        if (rclcpp::spin_until_future_complete(
                client_node,
                future,
                std::chrono::duration<double>(timeout_s)) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_WARN(node_->get_logger(), "[%s] '%s' did not return", name().c_str(), service);
            all_cleared = false;
        }
    }

    // SUCCESS even when a costmap did not clear. This is hygiene before a navigation goal, not
    // a precondition for one: Nav2 plans perfectly well from a stale costmap, just less
    // directly. Failing here would abort a mission over a housekeeping step.
    if (!all_cleared)
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] continuing with a costmap uncleared", name().c_str());
    }
    return BT::NodeStatus::SUCCESS;
}

namespace
{
// The builder form, not registerNodeType<T>(): every leaf needs the ROS node, and the plain
// form can only construct from (name, config).
template <typename LeafT>
void registerLeaf(BT::BehaviorTreeFactory& factory, const std::string& id, const RosContext& context)
{
    factory.registerBuilder<LeafT>(
        id,
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<LeafT>(name, config, context);
        });
}
}  // namespace

void registerSkillNodes(BT::BehaviorTreeFactory& factory, const RosContext& context)
{
    registerLeaf<NavigateToPose>(factory, "NavigateToPose", context);
    registerLeaf<ApproachObject>(factory, "ApproachObject", context);
    registerLeaf<Retreat>(factory, "Retreat", context);
    registerLeaf<Pick>(factory, "Pick", context);
    registerLeaf<Place>(factory, "Place", context);
    registerLeaf<SetArmPosture>(factory, "SetArmPosture", context);
    registerLeaf<ClearCostmaps>(factory, "ClearCostmaps", context);
}

}  // namespace g1_orchestration
