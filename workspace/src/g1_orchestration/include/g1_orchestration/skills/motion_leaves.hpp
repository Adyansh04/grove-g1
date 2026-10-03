#ifndef G1_ORCHESTRATION__SKILLS__MOTION_LEAVES_HPP_
#define G1_ORCHESTRATION__SKILLS__MOTION_LEAVES_HPP_

/**
 * @file motion_leaves.hpp
 * @brief BT leaves for small exact moves through Nav2's behavior server.
 *
 * Nav2's behaviors project each command ahead on the local costmap and stop on a predicted
 * collision, which a raw velocity command would not.
 */

#include <nav2_msgs/action/drive_on_heading.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <string>

#include "g1_orchestration/ros_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Walks straight forward or backward by a distance with Nav2's drive_on_heading.
 *
 * On a predicted collision Nav2 stops the robot where it is and the leaf fails: it never detours.
 */
class DriveStraight : public RosActionNode<nav2_msgs::action::DriveOnHeading>
{
public:
    DriveStraight(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool           fillGoal(Goal& goal) override;
    BT::NodeStatus judgeResult(const WrappedResult& result) override;
};

/**
 * @brief Turns on the spot by an angle relative to where the robot faces, with Nav2's spin.
 *
 * Unlike TurnTo it needs no TF: the angle is relative.
 */
class TurnBy : public RosActionNode<nav2_msgs::action::Spin>
{
public:
    TurnBy(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool           fillGoal(Goal& goal) override;
    BT::NodeStatus judgeResult(const WrappedResult& result) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__MOTION_LEAVES_HPP_
