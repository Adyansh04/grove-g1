#ifndef G1_ORCHESTRATION__SKILLS__CLEAR_COSTMAPS_HPP_
#define G1_ORCHESTRATION__SKILLS__CLEAR_COSTMAPS_HPP_

/**
 * @file clear_costmaps.hpp
 * @brief BT leaf that wipes both Nav2 costmaps.
 */

#include <string>

#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

/**
 * @brief Wipes both Nav2 costmaps.
 *
 * Housekeeping between a manipulation and the next navigation goal: working beside a surface
 * leaves the arm and the lifted object in the costmaps as obstacles that never were. Succeeds
 * even when a clear fails; this is hygiene, not a precondition.
 */
class ClearCostmaps : public ServiceLeaf
{
public:
    ClearCostmaps(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       tick() override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__CLEAR_COSTMAPS_HPP_
