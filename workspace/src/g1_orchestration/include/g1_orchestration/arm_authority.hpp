#ifndef G1_ORCHESTRATION__ARM_AUTHORITY_HPP_
#define G1_ORCHESTRATION__ARM_AUTHORITY_HPP_

/**
 * @file arm_authority.hpp
 * @brief Acquiring and releasing the arm and hands, in g1_bringup's activate_arm order.
 *
 * A component goes active before its controller and inactive after it, since command interfaces
 * exist only while their component is active. test_authority_drift keeps the names in step.
 * Acquired once per tree, not per skill, so the hands never let go between pick and place; the
 * executor releases on every exit, success or failure.
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace g1_orchestration
{

/**
 * @brief A controller to activate, and what it has to displace to get there.
 */
struct ControlledPart
{
    /// Component to activate first. Empty when it is already active for other reasons.
    std::string component;
    std::string controller;
    /// Controller holding these joints now, swapped out in the same switch. Empty if none.
    std::string displaces;
};

/**
 * @brief The arm, then each hand.
 *
 * @return The parts in acquisition order; releasing walks them in reverse.
 */
const std::vector<ControlledPart>& controlledParts();

/**
 * @brief What a paired switch should ask controller_manager for, given what it reports now.
 */
struct ArmSwitchPlan
{
    /// False when the incoming controller cannot take the joints; the outgoing one then stays,
    /// since an unclaimed joint is unpowered.
    bool possible{ false };
    bool already_held{ false };  ///< Incoming already holds them; there is nothing to ask for.
    bool displace{ false };      ///< Outgoing must be displaced, in the same switch.
};

/**
 * @brief Decides the paired arm switch from the two controllers' current states.
 *
 * The switch is then made `STRICT`: `BEST_EFFORT` applies whichever half it can, which can
 * deactivate the freeze alone and leave the arm joints unclaimed.
 *
 * @param incoming_state State of the controller that should end up holding the joints, as
 *        controller_manager reports it. Empty if it does not know the controller.
 * @param outgoing_state State of the controller currently holding them, same convention.
 * @return The switch to request. `possible` is false when nothing may be asked for.
 */
ArmSwitchPlan planArmSwitch(const std::string& incoming_state, const std::string& outgoing_state);

/**
 * @brief Takes the arm, then each hand.
 *
 * The arm is required; each hand is best-effort, so a missing or unpowered Dex3 leaves the arm
 * usable. Blocks, on a node of its own: spin_until_future_complete throws on an executor's node.
 *
 * @param logger Where progress and failures are reported.
 * @param timeout_s Per-step service budget.
 * @return false only if the arm could not be acquired; a hand that fails only warns.
 */
bool acquireArm(const rclcpp::Logger& logger, double timeout_s);

/**
 * @brief Hands each part back, in reverse: controllers first, then components.
 *
 * Best-effort, and does not stop partway: it also runs on the failure path.
 *
 * @param logger Where failures are reported.
 * @param timeout_s Per-step service budget.
 */
void releaseArm(const rclcpp::Logger& logger, double timeout_s);

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__ARM_AUTHORITY_HPP_
