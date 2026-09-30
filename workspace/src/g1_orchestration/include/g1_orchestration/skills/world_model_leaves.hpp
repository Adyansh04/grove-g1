#ifndef G1_ORCHESTRATION__SKILLS__WORLD_MODEL_LEAVES_HPP_
#define G1_ORCHESTRATION__SKILLS__WORLD_MODEL_LEAVES_HPP_

/**
 * @file world_model_leaves.hpp
 * @brief BT leaves that ask the world model where to go and what is where.
 *
 * The world model only answers; these leaves and the existing NavigateToPose move the robot, so
 * the tree stays the one thing commanding the base, one goal at a time.
 */

#include <nav2_msgs/action/spin.hpp>
#include <string>

#include "g1_orchestration/port_types.hpp"
#include "g1_orchestration/ros_action_node.hpp"
#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

/**
 * @brief Fetches the next viewpoint: where to stand and the headings to face there.
 *
 * FAILURE ends an exploration loop: `outcome` then says whether it was "done" or an error.
 * SUCCESS with `outcome` "stuck" asks the tree to move the robot off first. While the model has
 * no map or pose yet, or SLAM is catching up, it waits, up to timeout_s.
 */
class NextViewpoint : public ServiceLeaf
{
public:
    NextViewpoint(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       call() override;
};

/**
 * @brief Tells the world model whether a viewpoint was reached, so it can drop unreachable ones
 * and write off targets that stay unseen. Always succeeds.
 */
class ReportViewpoint : public ServiceLeaf
{
public:
    ReportViewpoint(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       call() override;
};

/**
 * @brief Turns a target named in words ("dustbin", "O12", "kitchen") into a reachable pose
 * facing it, for NavigateToPose. Replaces the literal x;y;yaw goals missions used to carry.
 */
class ResolveTarget : public ServiceLeaf
{
public:
    ResolveTarget(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       call() override;
};

/// Saves the world model to disk. Succeeds only if the save did.
class SaveWorld : public ServiceLeaf
{
public:
    SaveWorld(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       call() override;
};

/// Has the world model take out what it can tell is not an object, once exploring is over and
/// before the world is saved for a person to check.
class CleanUpWorld : public SaveWorld
{
public:
    using SaveWorld::SaveWorld;
    static BT::PortsList providedPorts();
};

/**
 * @brief Turns in place to a map-frame heading with Nav2's spin behavior.
 *
 * The spin is relative, so the robot's heading is read from TF when the goal is sent: after a
 * NavigateToPose the robot can stand up to 0.5 rad off the goal's yaw.
 */
class TurnTo : public RosActionNode<nav2_msgs::action::Spin>
{
public:
    TurnTo(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool           fillGoal(Goal& goal) override;
    BT::NodeStatus judgeResult(const WrappedResult& result) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__WORLD_MODEL_LEAVES_HPP_
