#ifndef G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_
#define G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_

#include <array>
#include <string>
#include <vector>

#include "g1_hardware_interface/arm_ramp_engine.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace g1_hardware_interface
{

// ros2_control System bridging the G1's 14 arm joints onto Unitree's
// weight-blended rt/arm_sdk DDS channel; legs/waist/hands stay with the
// onboard controller. See the package README for the safety/authority model
// this class enforces (single writer, ramp-not-snap, self-gated lifecycle).
class G1ArmSdkSystem : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareInfo& info) override;

    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn
    on_error(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type
    read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type
    write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Per-joint config parsed from HardwareInfo in on_init; index order is the
    // single source of truth shared by the state/command storage below,
    // motor_index_/kp_/kd_, and the ramp engine's per-joint arrays.
    std::array<std::string, kNumArmJoints> joint_names_{};
    std::array<int, kNumArmJoints>         motor_index_{};
    std::array<double, kNumArmJoints>      kp_{};
    std::array<double, kNumArmJoints>      kd_{};

    // System-level tunables parsed from HardwareInfo in on_init (see README's
    // param table for units/meaning).
    double command_publish_rate_hz_{ 0.0 };
    double blend_ramp_up_s_{ 0.0 };
    double blend_ramp_down_s_{ 0.0 };
    double emergency_ramp_down_s_{ 0.0 };
    double max_joint_velocity_rad_s_{ 0.0 };
    double lowstate_timeout_s_{ 0.0 };

    // Backing storage for exported state/command interfaces.
    std::array<double, kNumArmJoints> state_position_{};
    std::array<double, kNumArmJoints> state_velocity_{};
    std::array<double, kNumArmJoints> state_effort_{};
    std::array<double, kNumArmJoints> command_position_{};

    ArmRampEngine ramp_engine_{ RampConfig{} };
};

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_
