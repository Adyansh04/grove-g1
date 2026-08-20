#ifndef G1_ORCHESTRATION__SKILLS__GRASP_HPP_
#define G1_ORCHESTRATION__SKILLS__GRASP_HPP_

/**
 * @file grasp.hpp
 * @brief BT leaf for the learned-grasp skill.
 */

#include <g1_msgs/action/grasp.hpp>
#include <string>

#include "g1_orchestration/skill_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Grasps an object by running a learned policy under a planning-scene check.
 *
 * Sits beside Pick rather than replacing it: same authority, same controllers, different way of
 * deciding where the arm goes. A failure leaves the arm where it stopped, so a tree using this
 * needs its own recovery.
 */
class Grasp : public SkillActionNode<g1_msgs::action::Grasp>
{
public:
    Grasp(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal& goal) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__GRASP_HPP_
