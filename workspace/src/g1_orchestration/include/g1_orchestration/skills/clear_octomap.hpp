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
 * @brief Wipes the octomap in MoveIt's planning scene; ClearCostmaps does not touch it.
 *
 * Its voxels never decay, so a later plan avoids things no longer there. Succeeds even if the
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
