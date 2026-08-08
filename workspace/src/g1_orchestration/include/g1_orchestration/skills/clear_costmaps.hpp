#ifndef G1_ORCHESTRATION__SKILLS__CLEAR_COSTMAPS_HPP_
#define G1_ORCHESTRATION__SKILLS__CLEAR_COSTMAPS_HPP_

#include <string>

#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

/// Wipes both Nav2 costmaps. Not a skill: housekeeping between one manipulation and the next
/// navigation goal.
///
/// Manipulating beside a surface leaves the costmaps holding the robot's own arm, the object it
/// lifted, and whatever the base swept past. None of it is where the map says obstacles are, so
/// Nav2 plans around the robot's own recent history.
///
/// Succeeds even when a costmap does not clear: hygiene before a goal, not a precondition.
class ClearCostmaps : public ServiceLeaf
{
public:
    ClearCostmaps(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       tick() override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__CLEAR_COSTMAPS_HPP_
