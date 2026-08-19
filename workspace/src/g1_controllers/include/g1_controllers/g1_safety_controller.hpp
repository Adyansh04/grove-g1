#ifndef G1_CONTROLLERS__G1_SAFETY_CONTROLLER_HPP_
#define G1_CONTROLLERS__G1_SAFETY_CONTROLLER_HPP_

/**
 * @file g1_safety_controller.hpp
 * @brief Chainable blend and slew stage between a policy and the rt/lowcmd component.
 *
 * Keeps the upstream reference-interface layout and naming so a third-party controller chains onto
 * this unchanged. The upstream strategy registry and gravity compensation are left out: one
 * implementation needs no registry, and there is no inverse-dynamics solver here. See the package
 * README.
 */

#include <atomic>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace g1_controllers
{

/**
 * @brief One joint's blend-and-slew step.
 *
 * Blends from the activation pose toward the commanded one, then limits how far the output may
 * move from where it currently sits. The clamp is applied to the blended target rather than to the
 * command, so it bounds joint speed however far the blend ratio jumps in one tick.
 *
 * @param activation   Pose captured when the controller activated.
 * @param commanded    Position the upstream policy asked for.
 * @param blend_ratio  0 holds the activation pose, 1 follows the command outright.
 * @param integrated   Position commanded last tick, the slew reference.
 * @param max_velocity Per-joint rad/s ceiling; non-positive leaves the joint unclamped.
 * @param dt           Tick period in seconds.
 * @return The position to command this tick, which becomes the next tick's `integrated`.
 */
[[nodiscard]] double blendAndSlew(
    double activation, double commanded, double blend_ratio, double integrated, double max_velocity,
    double dt) noexcept;

/**
 * @brief Ramps a policy's joint targets in from the pose held at activation, and clamps their rate.
 *
 * Two ramps compose. The blend ratio moves the target from the activation pose to the policy's
 * command at `max_blend_ratio_speed` per second; the per-joint velocity clamp then limits how fast
 * the commanded position may actually move. The clamp holds regardless of how far the blend ratio
 * jumps, so a runtime parameter change stays safe.
 *
 * If joint velocities leave the range the policy was trained in, the blend is frozen at the last
 * safe pose and an emergency controller is switched in.
 */
class G1SafetyController : public controller_interface::ChainableControllerInterface
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

    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
    controller_interface::return_type                 update_reference_from_subscribers(
                        const rclcpp::Time& time, const rclcpp::Duration& period) override;
    controller_interface::return_type
    update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    /**
     * @brief Reference-interface slot order within one joint's block of kInterfacesPerJoint.
     */
    enum Slot : std::size_t
    {
        kPosition = 0,
        kVelocity = 1,
        kEffort   = 2,
        kKp       = 3,
        kKd       = 4,
    };

    /**
     * @brief Whether this tick's state has left the range the policy was trained on.
     *
     * @return true if the tick's joint velocities are outside the trained range.
     */
    [[nodiscard]] bool outOfDomain() const;

    /**
     * @brief Latches the emergency state, which nothing but a reactivation clears.
     */
    void latchEmergency(const char* reason);

    /**
     * @brief Asks controller_manager for the emergency controller, off the update thread.
     */
    void requestEmergencySwitch();

    std::vector<std::string> joint_names_;
    std::vector<double>      fallback_kp_;
    std::vector<double>      fallback_kd_;
    /// Non-positive disables the clamp for that joint, which is what the whole lower body wants:
    /// a balance policy needs its fast corrections unthrottled.
    std::vector<double> max_velocity_;

    /// Settable at runtime. Read once per tick, so a change mid-tick cannot tear.
    std::atomic<double> target_blend_ratio_{ 0.0 };
    double              blend_ratio_           = 0.0;
    double              max_blend_ratio_speed_ = 1.0;

    double mean_velocity_limit_ = 0.0;
    double max_velocity_limit_  = 0.0;
    /// Empty disables the switch, leaving the frozen-pose hold as the only response.
    std::string emergency_controller_;

    /// Pose the blend starts from, captured at activation.
    std::vector<double> activation_position_;
    /// The slew reference, carried tick to tick so the velocity clamp integrates rather than
    /// tracking the measurement and inheriting its noise.
    std::vector<double> integrated_position_;

    std::atomic<bool> emergency_latched_{ false };
    bool              emergency_switch_sent_ = false;

    std::vector<std::size_t> position_state_indices_;
    std::vector<std::size_t> velocity_state_indices_;
    std::vector<std::size_t> position_command_indices_;
    std::vector<std::size_t> velocity_command_indices_;
    std::vector<std::size_t> effort_command_indices_;
    std::vector<std::size_t> kp_command_indices_;
    std::vector<std::size_t> kd_command_indices_;

    /// Keeps `blend_ratio` live, which is how an operator brings the policy in on hardware.
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
    /// Runs on the controller_manager executor rather than the update thread, so the blocking
    /// parts of a service call stay off the control loop.
    rclcpp::TimerBase::SharedPtr emergency_timer_;
};

}  // namespace g1_controllers

#endif  // G1_CONTROLLERS__G1_SAFETY_CONTROLLER_HPP_
