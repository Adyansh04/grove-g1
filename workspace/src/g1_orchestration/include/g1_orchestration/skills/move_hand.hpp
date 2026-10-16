#ifndef G1_ORCHESTRATION__SKILLS__MOVE_HAND_HPP_
#define G1_ORCHESTRATION__SKILLS__MOVE_HAND_HPP_

/**
 * @file move_hand.hpp
 * @brief BT leaf for MoveHand: one hand to a point, or pointing at one.
 */

#include <g1_msgs/action/move_hand.hpp>
#include <string>

#include "g1_orchestration/skill_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Moves one hand to, or points it at, an object of the world model or a frame of the
 *        robot plus an offset. A learnable leaf: the server plans every move against the scene.
 */
class MoveHand : public SkillActionNode<g1_msgs::action::MoveHand>
{
public:
    MoveHand(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal& goal) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__MOVE_HAND_HPP_
