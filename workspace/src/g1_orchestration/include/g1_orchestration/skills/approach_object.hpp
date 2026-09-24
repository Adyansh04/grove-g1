#ifndef G1_ORCHESTRATION__SKILLS__APPROACH_OBJECT_HPP_
#define G1_ORCHESTRATION__SKILLS__APPROACH_OBJECT_HPP_

/**
 * @file approach_object.hpp
 * @brief BT leaf for the ApproachObject skill.
 */

#include <g1_msgs/action/approach_object.hpp>
#include <string>

#include "g1_orchestration/skill_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Walks the base the last stretch, until the object is within the arm's reach.
 *
 * Nav2's goal tolerance is several times wider than the arm's reach window.
 */
class ApproachObject : public SkillActionNode<g1_msgs::action::ApproachObject>
{
public:
    ApproachObject(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal& goal) override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__APPROACH_OBJECT_HPP_
