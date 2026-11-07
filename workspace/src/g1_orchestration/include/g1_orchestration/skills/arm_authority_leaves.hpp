#ifndef G1_ORCHESTRATION__SKILLS__ARM_AUTHORITY_LEAVES_HPP_
#define G1_ORCHESTRATION__SKILLS__ARM_AUTHORITY_LEAVES_HPP_

/**
 * @file arm_authority_leaves.hpp
 * @brief BT leaves that take the arm and hands, and hand them back.
 */

#include <memory>
#include <string>

#include "g1_orchestration/authority_port.hpp"
#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

/**
 * @brief Swaps the arm from its freeze to its trajectory controller, then activates each hand.
 *
 * Idempotent: an arm already held by its trajectory controller is not switched again, and is then
 * not recorded as this run's to release.
 */
class AcquireArm : public ServiceLeaf
{
public:
    AcquireArm(const std::string& name, const BT::NodeConfig& config, const RosContext& context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       call() override;

private:
    std::shared_ptr<ArmClaim> claim_;
};

/**
 * @brief Hands the arm and hands back, unless a hand stands closed on something.
 *
 * FAILURE when a hand may hold something, since a released hand drops it, or when
 * controller_manager would not take the arms back: they stay held either way.
 */
class ReleaseArm : public ServiceLeaf
{
public:
    ReleaseArm(const std::string& name, const BT::NodeConfig& config, const RosContext& context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       call() override;

private:
    std::shared_ptr<ArmClaim> claim_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__ARM_AUTHORITY_LEAVES_HPP_
