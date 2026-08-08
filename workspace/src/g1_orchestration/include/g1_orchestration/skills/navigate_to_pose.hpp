#ifndef G1_ORCHESTRATION__SKILLS__NAVIGATE_TO_POSE_HPP_
#define G1_ORCHESTRATION__SKILLS__NAVIGATE_TO_POSE_HPP_

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <string>

#include "g1_orchestration/port_types.hpp"
#include "g1_orchestration/ros_action_node.hpp"

namespace g1_orchestration
{

/// Drives the base to a pose. Nav2 is a black box here: the tree never sees a costmap, a
/// planner or a recovery, only whether the goal was reached.
///
/// The one action leaf that is not a SkillActionNode. Nav2's result is empty, so the outcome is
/// the result code alone.
class NavigateToPose : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
    NavigateToPose(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool           fillGoal(Goal& goal) override;
    BT::NodeStatus judgeResult(const WrappedResult& result) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__NAVIGATE_TO_POSE_HPP_
