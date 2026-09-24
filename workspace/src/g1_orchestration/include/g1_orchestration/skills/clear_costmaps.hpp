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
 * Run after manipulating beside a surface, which leaves the arm and a lifted object in the
 * costmaps as phantom obstacles. Succeeds even if a clear fails: Nav2 plans on a stale costmap.
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
