#ifndef G1_ORCHESTRATION__ARM_AUTHORITY_HPP_
#define G1_ORCHESTRATION__ARM_AUTHORITY_HPP_

/**
 * @file arm_authority.hpp
 * @brief Acquiring and releasing the arm and hands, in the mandatory order.
 *
 * The same sequence as g1_bringup's activate_arm / deactivate_arm scripts, and it has to stay
 * the same: Humble ties command-interface availability to hardware component state, so the
 * component goes active before its controller and inactive after it. `test_authority_drift`
 * asserts the names here still match those scripts.
 *
 * Owned by the executor rather than by a skill. control-mode rule 4 asks that a
 * skill release cleanly on success *or* failure, and at mission scope the only place that can
 * be guaranteed is around the whole tree -- a skill that acquired per goal would hand the
 * hands back between pick and place and drop what it was carrying.
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace g1_orchestration
{

/// One hardware component and the controller that claims its interfaces.
struct ControlledPart
{
    std::string component;
    std::string controller;
};

/// The arm, then each hand. Order matters: see `acquireArm`.
const std::vector<ControlledPart>& controlledParts();

/**
 * @brief Activates the arm component and controller, then each hand.
 *
 * The arm is required; a hand is best-effort. A Dex3 that is absent, unpowered or not
 * publishing state must not stop the arm from being usable, which is exactly what
 * activate_arm does and why.
 *
 * Spins a node of its own for the duration rather than borrowing the executor's: these are
 * blocking service calls, and spin_until_future_complete on a node an executor already owns
 * throws rather than waiting.
 *
 * @return false only if the ARM failed. A hand that did not come up warns and returns true.
 */
bool acquireArm(const rclcpp::Logger& logger, double timeout_s);

/**
 * @brief Deactivates controllers then components, in reverse.
 *
 * Best-effort throughout and never throws: this runs on the failure path too, where giving up
 * partway would leave a controller claiming interfaces of an inactive component.
 */
void releaseArm(const rclcpp::Logger& logger, double timeout_s);

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__ARM_AUTHORITY_HPP_
