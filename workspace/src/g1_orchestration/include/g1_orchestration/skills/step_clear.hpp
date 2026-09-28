#ifndef G1_ORCHESTRATION__SKILLS__STEP_CLEAR_HPP_
#define G1_ORCHESTRATION__SKILLS__STEP_CLEAR_HPP_

/**
 * @file step_clear.hpp
 * @brief BT leaf for the StepClear skill.
 */

#include <g1_msgs/action/step_clear.hpp>
#include <string>

#include "g1_orchestration/skill_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Steps the base out of the band where Nav2 counts it as touching an obstacle.
 *
 * Succeeds at once when the robot is already clear, so it can run before every Nav2 motion.
 */
class StepClear : public SkillActionNode<g1_msgs::action::StepClear>
{
public:
    StepClear(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal& goal) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__STEP_CLEAR_HPP_
