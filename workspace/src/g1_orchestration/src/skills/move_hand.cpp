/**
 * @file move_hand.cpp
 * @brief Ports and goal for the MoveHand leaf; an object is found in the world model first.
 */

#include "g1_orchestration/skills/move_hand.hpp"

#include <algorithm>
#include <canopy_msgs/msg/world_object.hpp>
#include <canopy_msgs/srv/find_objects.hpp>
#include <cmath>
#include <cstdint>
#include <string>

#include "g1_orchestration/ports.hpp"
#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

namespace
{

/// The farthest a hand is sent from its target along any axis, in metres: about an arm's reach.
constexpr double kMaxOffsetM = 1.5;
/// How many matches are asked for: the first that is still there is the one.
constexpr std::uint32_t kCandidates = 5;

}  // namespace

MoveHand::MoveHand(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : SkillActionNode(name, config, std::move(context), "/g1_manipulation_server/move_hand")
{}

BT::PortsList MoveHand::providedPorts()
{
    return providedBasicPorts({
        ports::arm(),
        BT::InputPort<std::string>(
            "target",
            "",
            "An object of the world model, by id or words; or leave it for frame."),
        BT::InputPort<std::string>(
            "frame",
            "",
            "A frame of the robot, such as torso_link, when there is no target."),
        BT::InputPort<double>("offset_x", 0.0, "Added forward, m."),
        BT::InputPort<double>("offset_y", 0.0, "Added to the left, m."),
        BT::InputPort<double>("offset_z", 0.0, "Added upward, m."),
        BT::InputPort<std::string>("mode", "point", "'point' at it, or 'reach' to it."),
        BT::InputPort<double>("speed", 0.0, "A share of the server's speed, 0 for all of it."),
        BT::InputPort<std::string>(
            "find_service",
            "/canopy/find_objects",
            "The world model's object search."),
        BT::InputPort<std::string>(
            "world_frame",
            "map",
            "The frame the world model's objects are in."),
    });
}

bool MoveHand::fillGoal(Goal& goal)
{
    const std::string target = getInput<std::string>("target").value_or("");
    const std::string frame  = getInput<std::string>("frame").value_or("");
    if (target.empty() == frame.empty())
    {
        setFailureText("give either a target in the world model or a frame, not both or neither");
        return false;
    }
    goal.arm      = getInput<std::string>("arm").value_or("right");
    goal.mode     = getInput<std::string>("mode").value_or("point");
    goal.speed    = getInput<double>("speed").value_or(0.0);
    goal.offset.x = getInput<double>("offset_x").value_or(0.0);
    goal.offset.y = getInput<double>("offset_y").value_or(0.0);
    goal.offset.z = getInput<double>("offset_z").value_or(0.0);
    // A taught or hand-written literal is checked only here: past an arm's reach is a slip.
    const auto within = [](double v, double most) {
        return std::isfinite(v) && std::abs(v) <= most;
    };
    if (!within(goal.offset.x, kMaxOffsetM) || !within(goal.offset.y, kMaxOffsetM) ||
        !within(goal.offset.z, kMaxOffsetM) || !std::isfinite(goal.speed) || goal.speed < 0.0)
    {
        setFailureText(
            "offsets take finite numbers within an arm's reach, speed one of at least 0");
        return false;
    }
    if (!frame.empty())
    {
        goal.target.header.frame_id = frame;
        return true;
    }
    using Service        = canopy_msgs::srv::FindObjects;
    using Object         = canopy_msgs::msg::WorldObject;
    auto request         = std::make_shared<Service::Request>();
    request->query       = target;
    request->max_results = kCandidates;
    const auto found     = callService<Service>(
        makeClientNode("g1_move_hand_client"),
        getInput<std::string>("find_service").value_or("/canopy/find_objects"),
        request,
        5.0);
    // A stale or removed object's pose is where it was: the hand goes only where one is now.
    const Object* live = nullptr;
    if (found != nullptr)
    {
        const auto it = std::ranges::find_if(found->objects, [](const Object& o) {
            return o.state == Object::STATE_ACTIVE;
        });
        live          = it == found->objects.end() ? nullptr : &*it;
    }
    if (live == nullptr)
    {
        setFailureText(
            "the world model knows nothing called '" + target + "'" +
            (found == nullptr        ? std::string(": it did not answer") :
             !found->objects.empty() ? std::string(" that is still there") :
                                       std::string("; a point by the robot is a frame, such as "
                                                   "torso_link, with offset_x, offset_y and "
                                                   "offset_z")));
        return false;
    }
    goal.target.header.frame_id = getInput<std::string>("world_frame").value_or("map");
    goal.target.point           = live->pose.position;
    // Its centre is inside it: a hand reaching for it goes to its top, as HandTo says.
    if (goal.mode == "reach")
    {
        goal.target.point.z += std::max(live->size.z, 0.0) / 2.0;
    }
    return true;
}

}  // namespace g1_orchestration
