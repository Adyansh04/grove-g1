#ifndef G1_ORCHESTRATION__SKILLS__CLEAR_OCTOMAP_HPP_
#define G1_ORCHESTRATION__SKILLS__CLEAR_OCTOMAP_HPP_

#include <string>

#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

/// Wipes MoveIt's octomap. The 3D counterpart to ClearCostmaps, and a different subsystem: that
/// one clears Nav2's 2D grids, which the base plans over, and never touches the planning scene
/// the arm plans against.
///
/// Manipulating beside a surface leaves the octomap holding the arm and whatever it picked up,
/// where they used to be. Those voxels do not decay, so a later plan can route around -- or
/// collide with -- a ghost of something that was cleared out of the world minutes ago.
///
/// Succeeds even when the clear fails, for ClearCostmaps' reason: housekeeping, not a
/// precondition.
class ClearOctomap : public ServiceLeaf
{
public:
    ClearOctomap(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       tick() override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__CLEAR_OCTOMAP_HPP_
