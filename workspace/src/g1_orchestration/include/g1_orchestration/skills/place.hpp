#ifndef G1_ORCHESTRATION__SKILLS__PLACE_HPP_
#define G1_ORCHESTRATION__SKILLS__PLACE_HPP_

/**
 * @file place.hpp
 * @brief BT leaf for the Place skill.
 */

#include <g1_msgs/action/place.hpp>
#include <string>

#include "g1_orchestration/port_types.hpp"
#include "g1_orchestration/skill_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Puts down whatever the given arm holds, on a detected surface or at a fixed point.
 */
class Place : public SkillActionNode<g1_msgs::action::Place>
{
public:
    Place(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal& goal) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__PLACE_HPP_
