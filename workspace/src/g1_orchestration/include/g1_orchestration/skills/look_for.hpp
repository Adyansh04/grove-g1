#ifndef G1_ORCHESTRATION__SKILLS__LOOK_FOR_HPP_
#define G1_ORCHESTRATION__SKILLS__LOOK_FOR_HPP_

/**
 * @file look_for.hpp
 * @brief BT leaf that narrows the detector to the objects a task needs, then waits for them.
 */

#include <string>

#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

/**
 * @brief Writes the detector's `phrases` and blocks until every named object is on /objects.
 *
 * Asking for the whole scene is not free: a phrase the detector half-matches elsewhere gives one
 * name two tracks, and a phrase with two tracks loses its bare-phrase alias, which is the name
 * Pick and Place resolve by. Naming only what the task needs is what keeps the alias unique.
 */
class LookFor : public ServiceLeaf
{
public:
    LookFor(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       tick() override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__LOOK_FOR_HPP_
