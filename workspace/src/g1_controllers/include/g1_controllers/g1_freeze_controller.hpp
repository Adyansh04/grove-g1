#ifndef G1_CONTROLLERS__G1_FREEZE_CONTROLLER_HPP_
#define G1_CONTROLLERS__G1_FREEZE_CONTROLLER_HPP_

/**
 * @file g1_freeze_controller.hpp
 * @brief Capture-and-hold controller for joints on the rt/lowcmd component.
 */

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace g1_controllers
{

/**
 * @brief Holds every claimed joint at the position it had when this controller activated, or at
 * a configured target reached by a linear ramp from there.
 *
 * One `kp` and `kd` for every claimed joint. `target_positions` (one per joint, radians) and
 * `ramp_duration_s` (seconds, positive whenever targets are given) are optional; without them the
 * activation pose is held. The lowcmd component leaves unclaimed joints unpowered, so this is what
 * holds the body when no policy is running.
 */
class G1FreezeController : public controller_interface::ControllerInterface
{
public:
    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type
    update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    std::vector<std::string> joint_names_;
    double                   kp_              = 0.0;
    double                   kd_              = 0.0;
    double                   ramp_duration_s_ = 0.0;

    /// Configured targets; empty holds the activation pose.
    std::vector<double> target_positions_;
    /// Captured in on_activate: where the ramp starts.
    std::vector<double> frozen_positions_;
    /// Where the ramp ends and the hold stays: the targets, or the activation pose.
    std::vector<double> goal_positions_;
    /// Seconds since activation.
    double elapsed_s_ = 0.0;

    std::vector<std::size_t> position_state_indices_;
    std::vector<std::size_t> position_command_indices_;
    std::vector<std::size_t> velocity_command_indices_;
    std::vector<std::size_t> effort_command_indices_;
    std::vector<std::size_t> kp_command_indices_;
    std::vector<std::size_t> kd_command_indices_;
};

}  // namespace g1_controllers

#endif  // G1_CONTROLLERS__G1_FREEZE_CONTROLLER_HPP_
