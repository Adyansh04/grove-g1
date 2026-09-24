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
 * @brief Grasps an object with a learned policy, checked against the planning scene.
 *
 * A failure leaves the arm where it stopped, so the tree must recover it.
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
