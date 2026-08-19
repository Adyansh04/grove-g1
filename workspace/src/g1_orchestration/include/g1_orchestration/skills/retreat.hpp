#ifndef G1_ORCHESTRATION__SKILLS__RETREAT_HPP_
#define G1_ORCHESTRATION__SKILLS__RETREAT_HPP_

/**
 * @file retreat.hpp
 * @brief BT leaf for the Retreat skill.
 */

#include <g1_msgs/action/retreat.hpp>
#include <string>

#include "g1_orchestration/skill_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Reverses the base away from a surface and stops.
 *
 * No turn, no walk: a navigation goal normally follows and does that far better.
 */
class Retreat : public SkillActionNode<g1_msgs::action::Retreat>
{
public:
    Retreat(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal& goal) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__RETREAT_HPP_
