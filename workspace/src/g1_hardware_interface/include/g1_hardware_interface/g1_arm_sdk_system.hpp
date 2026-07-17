#ifndef G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_
#define G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <chrono>
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
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace g1_hardware_interface
{

// G1Arm7JointIndex::NOT_USED_JOINT (unitree_ros2's example/src/include/g1/g1.hpp)
// -- the reserved motor_cmd slot the motion service reads as the arm_sdk
// blend weight, ramped so arm control authority hands off without a snap.
inline constexpr std::size_t kWeightMotorIndex = 29;

// Fills every arm slot's q (from `position`) and per-joint kp/kd, plus the
// weight slot (motor_cmd[kWeightMotorIndex].q), on `cmd`; dq/tau on arm
// slots are set to 0. Every other motor slot is left exactly as `cmd`
// already had it -- callers rely on a preallocated, zero-initialized LowCmd
// never being touched anywhere else, so legs/waist/hands stay provably
// inert. Mirrors exactly what Unitree's own g1_arm_sdk_dds_example touches
// on the outgoing message (arm q/dq/tau/kp/kd and the weight slot) and
// nothing else -- notably, it does not set mode_pr/mode_machine or any
// motor's `mode` field, and neither does this. Kept as a standalone
// function so the assembly logic is unit-testable without a live hardware
// component.
void assembleLowCmd(
    unitree_hg::msg::LowCmd& cmd, const std::array<int, kNumArmJoints>& motor_index,
    const std::array<double, kNumArmJoints>& position, const std::array<double, kNumArmJoints>& kp,
    const std::array<double, kNumArmJoints>& kd, float weight);

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
    // Cancels the executor, joins its thread, and tears down the node/sub/pub.
    // Idempotent -- safe to call from on_configure (which rebuilds from
    // scratch every time), on_cleanup, and the destructor.
    void        shutdownInternalNode();
    static bool isStale(
        const std::chrono::steady_clock::time_point& arrival,
        std::chrono::steady_clock::duration          timeout);
    std::chrono::steady_clock::duration lowstateTimeoutDuration() const;

    // Off-RT-thread only (the internal executor's timer, or a lifecycle
    // callback: on_deactivate/on_error/on_shutdown). Publishes directly via
    // arm_sdk_rt_pub_ -- never called from write().
    //
    // Why this blocks and publishes itself rather than asking write() to do
    // it: resource_manager serializes a hardware component's lifecycle
    // transitions against its own read()/write() calls (confirmed directly
    // during manual sim validation -- write() provably never ticks while a
    // transition callback is running), so a transition that *waited* for
    // write() to ramp the weight down would deadlock. Whichever context
    // currently holds the floor -- write() during normal ACTIVE ticking, or
    // this function during a transition -- is the sole writer for that
    // window, so there is no concurrent second writer to hand off to or
    // race against. Claiming writer_token_claimed_ regardless is cheap
    // defense-in-depth against that locking behavior ever changing.
    void checkPublisherCount();
    void rampDownSynchronously(BlendMode target_mode);

    // Hidden node + single-threaded executor for DDS I/O, torn down in
    // on_cleanup. Never added to the controller_manager's own executor: its
    // only job is servicing /lowstate, running the /arm_sdk publisher's own
    // background thread, and the ~1 Hz advisory publisher-count timer.
    rclcpp::Node::SharedPtr                                             node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr                executor_;
    std::thread                                                         executor_thread_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr          lowstate_sub_;
    realtime_tools::RealtimeBuffer<StampedLowState>                     lowstate_buffer_;
    realtime_tools::RealtimePublisherSharedPtr<unitree_hg::msg::LowCmd> arm_sdk_rt_pub_;
    rclcpp::TimerBase::SharedPtr                                        publisher_count_timer_;
    // One-shot per detected conflict: joined and re-armed on the next
    // on_activate. Runs rampDownSynchronously() off the internal executor
    // thread so /lowstate reception and the timer keep servicing while it
    // blocks.
    std::thread conflict_ramp_thread_;

    // The single writer-authority state machine (see the package README).
    // ACTIVE/RAMP_DOWN/EMERGENCY_RAMP_DOWN all imply active publication;
    // INACTIVE means write() self-gates -- Humble still calls write() on an
    // inactive component, so "commands only flow while active" is enforced
    // here, not assumed from the framework.
    std::atomic<BlendMode> mode_{ BlendMode::kInactive };
    // Claimed by rampDownSynchronously() before it publishes anything, and
    // cleared only by a fresh on_activate. write() checks this FIRST, before
    // anything else, so it provably never publishes while a transition
    // callback might also be publishing -- see rampDownSynchronously()'s
    // comment for why that's normally impossible anyway.
    std::atomic<bool> writer_token_claimed_{ false };

    // RT-thread-only (never touched off write()): accumulates elapsed time
    // toward the next throttled /arm_sdk publish.
    double time_since_last_publish_s_{ 0.0 };
};

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_
