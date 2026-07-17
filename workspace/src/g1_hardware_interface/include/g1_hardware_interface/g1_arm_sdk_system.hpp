#ifndef G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_
#define G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "g1_hardware_interface/arm_ramp_engine.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace g1_hardware_interface
{

// ros2_control System bridging the G1's 14 arm joints onto Unitree's
// weight-blended rt/arm_sdk DDS channel; legs/waist/hands stay with the
// onboard controller. See the package README for the safety/authority model
// this class enforces (single writer, ramp-not-snap, self-gated lifecycle).
class G1ArmSdkSystem : public hardware_interface::SystemInterface
{
public:
    // Belt-and-braces: controller_manager doesn't guarantee on_cleanup runs
    // before the process exits (e.g. an ungraceful SIGKILL, or a plugin
    // unload with a component left inactive-but-configured). Without this,
    // a joinable executor_thread_ at destruction calls std::terminate() --
    // observed directly during manual sim validation.
    ~G1ArmSdkSystem() override;

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

    // LowState carries no timestamp field of its own, so freshness is judged
    // against a steady-clock stamp taken when the subscription callback
    // received it (immune to wall-clock jumps, unlike system_clock).
    struct StampedLowState
    {
        unitree_hg::msg::LowState             state;
        std::chrono::steady_clock::time_point arrival{};
    };

    static std::string makeInternalNodeName();
    void               lowstateCallback(const unitree_hg::msg::LowState::SharedPtr msg);
    // Cancels the executor, joins its thread, and tears down the node/sub.
    // Idempotent -- safe to call from both on_cleanup and the destructor.
    void shutdownInternalNode();

    // Hidden node + single-threaded executor for DDS I/O, torn down in
    // on_cleanup. Never added to the controller_manager's own executor: its
    // only job is servicing /lowstate and (from the next commit) the
    // /arm_sdk publisher's background thread and the advisory
    // publisher-count timer.
    rclcpp::Node::SharedPtr                                    node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr       executor_;
    std::thread                                                executor_thread_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
    realtime_tools::RealtimeBuffer<StampedLowState>            lowstate_buffer_;
};

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_
