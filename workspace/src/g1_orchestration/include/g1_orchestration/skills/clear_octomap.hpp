#ifndef G1_ORCHESTRATION__SKILLS__CLEAR_OCTOMAP_HPP_
#define G1_ORCHESTRATION__SKILLS__CLEAR_OCTOMAP_HPP_

/**
 * @file clear_octomap.hpp
 * @brief BT leaf that wipes MoveIt's octomap.
 */

#include <string>

#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

/**
 * @brief Wipes MoveIt's octomap, the planning scene the arm plans against.
 *
 * ClearCostmaps is the 2D counterpart and does not touch it. Stale voxels never decay, so a
 * later plan routes around a ghost of an object that has since moved. Succeeds even when the
 * clear fails.
 */
class ClearOctomap : public ServiceLeaf
{
public:
    ClearOctomap(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       tick() override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__CLEAR_OCTOMAP_HPP_
