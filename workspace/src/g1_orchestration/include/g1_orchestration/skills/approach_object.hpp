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
 * @brief Walks the base the last half metre, until the object is where the arm can reach it.
 *
 * The step NavigateToPose cannot do: Nav2 arrives within 0.5 m of a pose it chose from a map,
 * and the arm's whole usable window is about a quarter of that.
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
