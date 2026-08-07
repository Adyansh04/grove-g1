#ifndef G1_ORCHESTRATION__SKILL_NODES_HPP_
#define G1_ORCHESTRATION__SKILL_NODES_HPP_

/**
 * @file skill_nodes.hpp
 * @brief The leaves a mission tree is built from.
 *
 * Each is a thin client. The tree decides what happens and in what order; the skills decide
 * how. Nothing here plans, moves a joint, or takes control authority of its own -- Nav2 and
 * g1_manipulation own all of that, and this package only sequences them.
 */

#include <behaviortree_cpp/bt_factory.h>

#include <g1_msgs/action/approach_object.hpp>
#include <g1_msgs/action/pick.hpp>
#include <g1_msgs/action/place.hpp>
#include <g1_msgs/action/retreat.hpp>
#include <g1_msgs/action/set_arm_posture.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <string>

#include "g1_orchestration/ros_action_node.hpp"

namespace g1_orchestration
{

/// A base goal written into the tree XML as "x;y;yaw", in metres and radians.
struct Station
{
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

/// A point written into the tree XML as "x;y;z", in metres. Distinct from Station rather than
/// reusing its third field: a place target has a height, not a heading.
struct Point3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/// Drives the base to a pose. Nav2 is a black box here: the tree never sees a costmap, a
/// planner or a recovery, only whether the goal was reached.
class NavigateToPose : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
    NavigateToPose(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool           fillGoal(Goal& goal) override;
    BT::NodeStatus judgeResult(const WrappedResult& result) override;
};

/// Walks the base the last half metre, until the object is somewhere the arm can reach it.
///
/// The step NavigateToPose cannot do: Nav2 arrives within 0.5 m of a pose it chose from a map,
/// and the arm's whole usable window is about a quarter of that.
class ApproachObject : public RosActionNode<g1_msgs::action::ApproachObject>
{
public:
    ApproachObject(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool           fillGoal(Goal& goal) override;
    BT::NodeStatus judgeResult(const WrappedResult& result) override;
};

/// Backs the base away from a surface. Turns around and walks, because this gait has no
/// reverse -- see the action definition.
class Retreat : public RosActionNode<g1_msgs::action::Retreat>
{
public:
    Retreat(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool           fillGoal(Goal& goal) override;
    BT::NodeStatus judgeResult(const WrappedResult& result) override;
};

/// Picks a named object up. The pose is not a port: the skill reads it fresh from /objects,
/// so a retry re-reads rather than replaying a stale one.
class Pick : public RosActionNode<g1_msgs::action::Pick>
{
public:
    Pick(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool           fillGoal(Goal& goal) override;
    BT::NodeStatus judgeResult(const WrappedResult& result) override;
};

/// Puts down whatever the given arm holds, at a pose in the planning frame.
class Place : public RosActionNode<g1_msgs::action::Place>
{
public:
    Place(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool           fillGoal(Goal& goal) override;
    BT::NodeStatus judgeResult(const WrappedResult& result) override;
};

/// Moves a planning group to one of its named SRDF poses.
class SetArmPosture : public RosActionNode<g1_msgs::action::SetArmPosture>
{
public:
    SetArmPosture(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool           fillGoal(Goal& goal) override;
    BT::NodeStatus judgeResult(const WrappedResult& result) override;
};

/// Registers every leaf this package provides, binding each to `context`.
void registerSkillNodes(BT::BehaviorTreeFactory& factory, const RosContext& context);

}  // namespace g1_orchestration

// Lets a tree write `goal="4.1;-4.1;1.57"`. Declared in BT's namespace because that is where
// the library looks the conversion up.
namespace BT
{
template <>
g1_orchestration::Station convertFromString(StringView text);

template <>
g1_orchestration::Point3 convertFromString(StringView text);
}  // namespace BT

#endif  // G1_ORCHESTRATION__SKILL_NODES_HPP_
