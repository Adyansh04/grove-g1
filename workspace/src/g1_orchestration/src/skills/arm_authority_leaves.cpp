/**
 * @file arm_authority_leaves.cpp
 * @brief Ports and ticks for the AcquireArm and ReleaseArm leaves.
 */

#include "g1_orchestration/skills/arm_authority_leaves.hpp"

#include <string>

#include "g1_orchestration/arm_authority.hpp"
#include "g1_orchestration/ports.hpp"

namespace g1_orchestration
{

namespace
{
constexpr double      kAuthorityTimeoutS   = 15.0;
constexpr const char* kAuthorityTimeoutDoc = "Per-step service budget.";
}  // namespace

AcquireArm::AcquireArm(
    const std::string& name, const BT::NodeConfig& config, const RosContext& context)
  : ServiceLeaf(name, config, context)
  , claim_(context.arm_claim)
{}

BT::PortsList AcquireArm::providedPorts()
{
    return { ports::serviceTimeout(kAuthorityTimeoutS, kAuthorityTimeoutDoc) };
}

BT::NodeStatus AcquireArm::call()
{
    const double   timeout_s = getInput<double>("timeout_s").value_or(kAuthorityTimeoutS);
    const Acquired got       = acquireArm(node_->get_logger(), timeout_s);
    if (got == Acquired::kFailed)
    {
        setFailureText("the arms could not be acquired; is the control stack up?");
        return BT::NodeStatus::FAILURE;
    }
    if (got == Acquired::kTaken && claim_ != nullptr)
    {
        claim_->taken = true;
    }
    return BT::NodeStatus::SUCCESS;
}

ReleaseArm::ReleaseArm(
    const std::string& name, const BT::NodeConfig& config, const RosContext& context)
  : ServiceLeaf(name, config, context)
  , claim_(context.arm_claim)
{}

BT::PortsList ReleaseArm::providedPorts()
{
    return { ports::serviceTimeout(kAuthorityTimeoutS, kAuthorityTimeoutDoc) };
}

BT::NodeStatus ReleaseArm::call()
{
    const double timeout_s = getInput<double>("timeout_s").value_or(kAuthorityTimeoutS);
    const bool   released  = claim_ == nullptr ?
                                 releaseArm(node_->get_logger(), timeout_s) :
                                 releaseUnlessHolding(node_->get_logger(), timeout_s, claim_->fingers);
    if (!released)
    {
        setFailureText(
            "the arms stay held: a hand may hold something, or controller_manager would not take "
            "them back");
        return BT::NodeStatus::FAILURE;
    }
    if (claim_ != nullptr)
    {
        claim_->taken = false;
    }
    return BT::NodeStatus::SUCCESS;
}

}  // namespace g1_orchestration
