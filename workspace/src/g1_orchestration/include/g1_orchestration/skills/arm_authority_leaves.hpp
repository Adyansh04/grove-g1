#ifndef G1_ORCHESTRATION__SKILLS__ARM_AUTHORITY_LEAVES_HPP_
#define G1_ORCHESTRATION__SKILLS__ARM_AUTHORITY_LEAVES_HPP_

/**
 * @file arm_authority_leaves.hpp
 * @brief BT leaves that take the arm and hands, and hand them back.
 */

#include <string>

#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

/**
 * @brief Swaps the arm from its freeze to its trajectory controller, then activates each hand.
 *
 * Idempotent: an arm already held by its trajectory controller is not switched again.
 */
class AcquireArm : public ServiceLeaf
{
public:
    AcquireArm(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       tick() override;
};

/**
 * @brief Hands the arm and hands back.
 *
 * Always SUCCESS, so cleanup cannot fail the tree; the executor releases again on exit anyway.
 */
class ReleaseArm : public ServiceLeaf
{
public:
    ReleaseArm(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       tick() override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__ARM_AUTHORITY_LEAVES_HPP_
