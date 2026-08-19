/**
 * @file arm_authority_leaves.cpp
 * @brief Ports and ticks for the AcquireArm and ReleaseArm leaves.
 */

#include "g1_orchestration/skills/arm_authority_leaves.hpp"

#include <string>
#include <utility>

#include "g1_orchestration/arm_authority.hpp"
#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

namespace
{
constexpr double      kAuthorityTimeoutS   = 15.0;
constexpr const char* kAuthorityTimeoutDoc = "Per-step service budget.";
}  // namespace

AcquireArm::AcquireArm(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : ServiceLeaf(name, config, std::move(context))
{}

BT::PortsList AcquireArm::providedPorts()
{
    return { ports::serviceTimeout(kAuthorityTimeoutS, kAuthorityTimeoutDoc) };
}

BT::NodeStatus AcquireArm::tick()
{
    const double timeout_s = getInput<double>("timeout_s").value_or(kAuthorityTimeoutS);
    return acquireArm(node_->get_logger(), timeout_s) ? BT::NodeStatus::SUCCESS :
                                                        BT::NodeStatus::FAILURE;
}

ReleaseArm::ReleaseArm(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : ServiceLeaf(name, config, std::move(context))
{}

BT::PortsList ReleaseArm::providedPorts()
{
    return { ports::serviceTimeout(kAuthorityTimeoutS, kAuthorityTimeoutDoc) };
}

BT::NodeStatus ReleaseArm::tick()
{
    releaseArm(node_->get_logger(), getInput<double>("timeout_s").value_or(kAuthorityTimeoutS));
    return BT::NodeStatus::SUCCESS;
}

}  // namespace g1_orchestration
