#ifndef G1_ORCHESTRATION__SKILLS__SET_ARM_POSTURE_HPP_
#define G1_ORCHESTRATION__SKILLS__SET_ARM_POSTURE_HPP_

/**
 * @file set_arm_posture.hpp
 * @brief BT leaf for the SetArmPosture skill.
 */

#include <g1_msgs/action/set_arm_posture.hpp>
#include <string>

#include "g1_orchestration/skill_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Moves a planning group to one of its named SRDF poses.
 */
class SetArmPosture : public SkillActionNode<g1_msgs::action::SetArmPosture>
{
public:
    SetArmPosture(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal& goal) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__SET_ARM_POSTURE_HPP_
