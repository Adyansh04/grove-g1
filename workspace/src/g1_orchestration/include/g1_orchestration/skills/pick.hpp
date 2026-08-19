#ifndef G1_ORCHESTRATION__SKILLS__PICK_HPP_
#define G1_ORCHESTRATION__SKILLS__PICK_HPP_

/**
 * @file pick.hpp
 * @brief BT leaf for the Pick skill.
 */

#include <g1_msgs/action/pick.hpp>
#include <string>

#include "g1_orchestration/skill_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Picks a named object up.
 *
 * The pose is not a port: the skill reads it fresh from /objects, so a retry re-reads rather
 * than replaying a stale one.
 */
class Pick : public SkillActionNode<g1_msgs::action::Pick>
{
public:
    Pick(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal& goal) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__PICK_HPP_
