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
 * @brief Activates the arm component and controller, then each hand.
 *
 * Idempotent: the arm is very often already acquired, so finding it so costs nothing.
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
 * Always SUCCESS: a release that reported failure would fail the tree it is cleaning up after,
 * and the executor releases again on its way out regardless.
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
