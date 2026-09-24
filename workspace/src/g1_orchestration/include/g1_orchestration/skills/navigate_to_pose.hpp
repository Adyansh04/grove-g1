#ifndef G1_ORCHESTRATION__SKILLS__NAVIGATE_TO_POSE_HPP_
#define G1_ORCHESTRATION__SKILLS__NAVIGATE_TO_POSE_HPP_

/**
 * @file navigate_to_pose.hpp
 * @brief BT leaf that drives the base to a pose through Nav2.
 */

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <string>

#include "g1_orchestration/port_types.hpp"
#include "g1_orchestration/ros_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Drives the base to a pose through Nav2.
 *
 * Not a SkillActionNode: Nav2's result is empty, so the result code is the outcome.
 */
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
