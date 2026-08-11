#ifndef G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_
#define G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_

/**
 * @file g1_arm_sdk_system.hpp
 * @brief ros2_control hardware interface bridging the G1 arm joints onto Unitree's rt/arm_sdk.
 */

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

/**
 * @brief G1Arm7JointIndex::NOT_USED_JOINT (unitree_ros2's
 * example/src/include/g1/g1.hpp) -- the reserved motor_cmd slot the
 * motion service reads as the arm_sdk blend weight, ramped so arm
 * control authority hands off without a snap.
 */
inline constexpr std::size_t kWeightMotorIndex = 29;

/**
 * @brief The three waist motors, which /arm_sdk hands over along with the arms.
 *
 * Not planned joints and not exposed to MoveIt: they are held where they were found. Unitree's
 * own arm_sdk example (unitree_ros2 example/src/src/g1/high_level/g1_arm_sdk_dds_example.cpp,
 * G1Arm7JointIndex list) commands seventeen motors, the last three being WAIST_YAW, WAIST_ROLL
 * and WAIST_PITCH at four times the arm gains. Leaving them out of the LowCmd sends them
 * kp=kd=0 while the blend weight is up, which is a torso with no stiffness under arm load.
 */
inline constexpr std::size_t                      kNumWaistJoints  = 3;
inline constexpr std::array<int, kNumWaistJoints> kWaistMotorIndex = { 12, 13, 14 };

/**
 * @brief Fills the 14 arm slots and the 3 waist slots on `cmd`, plus the weight slot
 * (motor_cmd[kWeightMotorIndex].q); dq/tau are set to 0 on all of them.
 *
 * Arms take `position`; the waist takes `waist_hold`, which is where it was measured when
 * authority was acquired. LEGS AND HANDS are left exactly as `cmd` already had it -- callers
 * rely on a preallocated, zero-initialized LowCmd never being touched anywhere else, so those
 * stay provably inert. This is the same set of slots Unitree's own g1_arm_sdk_dds_example
 * touches, waist included: /arm_sdk hands over the arms AND the waist, and leaving the waist
 * slots zero commands it at zero stiffness. Like that example it sets neither mode_pr,
 * mode_machine, nor any motor's `mode`. Kept as a standalone function so the assembly logic is
 * unit-testable without a live hardware component.
 *
 * @param cmd          LowCmd message filled in place.
 * @param motor_index  Per-joint motor_cmd slot index for each arm joint.
 * @param position     Per-joint commanded position (q) for each arm joint.
 * @param kp           Per-joint position gain for each arm joint.
 * @param kd           Per-joint velocity gain for each arm joint.
 * @param weight       Arm-sdk blend weight written to the weight slot.
 * @param waist_hold   Latched waist position, one per kWaistMotorIndex entry.
 * @param waist_kp     Position gain for all three waist motors.
 * @param waist_kd     Velocity gain for all three waist motors.
 */
/**
 * @brief The waist positions to hold, read out of a LowState at authority acquisition.
 *
 * Free function for the same reason assembleLowCmd is: reading the wrong slots here would put
 * stiff gains on a leg, and that is worth asserting without a live hardware component.
 */
std::array<double, kNumWaistJoints> waistHoldFrom(const unitree_hg::msg::LowState& state);

void assembleLowCmd(
    unitree_hg::msg::LowCmd& cmd, const std::array<int, kNumArmJoints>& motor_index,
    const std::array<double, kNumArmJoints>& position, const std::array<double, kNumArmJoints>& kp,
    const std::array<double, kNumArmJoints>& kd, float weight,
    const std::array<double, kNumWaistJoints>& waist_hold, double waist_kp, double waist_kd);

/**
 * @brief ros2_control System bridging the G1's 14 arm joints onto Unitree's weight-blended
 * rt/arm_sdk DDS channel, holding the 3 waist joints that channel hands over with them; legs
 * and hands stay with the onboard controller.
 *
 * See the package README for the safety/authority model this class
 * enforces (single writer, ramp-not-snap, self-gated lifecycle).
 */
class G1ArmSdkSystem : public hardware_interface::SystemInterface
{
public:
    /**
     * @brief Belt-and-braces: controller_manager doesn't guarantee
     * on_cleanup runs before the process exits (e.g. an ungraceful
     * SIGKILL, or a plugin unload with a component left
     * inactive-but-configured).
     *
     * Without this, a joinable executor_thread_ at destruction calls
     * std::terminate() -- observed directly during manual sim validation.
     */
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
    /// Per-joint config parsed from HardwareInfo in on_init; index order is the
    /// single source of truth shared by the state/command storage below,
    /// motor_index_/kp_/kd_, and the ramp engine's per-joint arrays.
    std::array<std::string, kNumArmJoints> joint_names_{};
    std::array<int, kNumArmJoints>         motor_index_{};
    std::array<double, kNumArmJoints>      kp_{};
    std::array<double, kNumArmJoints>      kd_{};
    /// Where the waist was when the blend engaged. Latched, never commanded to a new value:
    /// nothing in this stack plans the waist, and snapping it is a fall.
    std::array<double, kNumWaistJoints> waist_hold_{};
    double                              waist_kp_{};
    double                              waist_kd_{};

    /// System-level tunables parsed from HardwareInfo in on_init (see README's
    /// param table for units/meaning).
    double command_publish_rate_hz_{ 0.0 };
    double blend_ramp_up_s_{ 0.0 };
    double blend_ramp_down_s_{ 0.0 };
    double emergency_ramp_down_s_{ 0.0 };
    double max_joint_velocity_rad_s_{ 0.0 };
    double lowstate_timeout_s_{ 0.0 };

    /// Backing storage for exported state/command interfaces.
    std::array<double, kNumArmJoints> state_position_{};
    std::array<double, kNumArmJoints> state_velocity_{};
    std::array<double, kNumArmJoints> state_effort_{};
    std::array<double, kNumArmJoints> command_position_{};

    ArmRampEngine ramp_engine_{ RampConfig{} };

    /**
     * @brief LowState carries no timestamp field of its own, so freshness
     * is judged against a steady-clock stamp taken when the subscription
     * callback received it (immune to wall-clock jumps, unlike
     * system_clock).
     */
    struct StampedLowState
    {
        unitree_hg::msg::LowState             state;
        std::chrono::steady_clock::time_point arrival{};
    };

    static std::string makeInternalNodeName();
    void               lowstateCallback(const unitree_hg::msg::LowState::ConstSharedPtr& msg);
    /**
     * @brief Cancels the executor, joins its thread, and tears down the node/sub/pub.
     *
     * @note Idempotent -- safe to call from on_configure (which rebuilds
     *       from scratch every time), on_cleanup, and the destructor.
     */
    void        shutdownInternalNode();
    static bool isStale(
        const std::chrono::steady_clock::time_point& arrival,
        std::chrono::steady_clock::duration          timeout);
    std::chrono::steady_clock::duration lowstateTimeoutDuration() const;

    /**
     * @brief Advisory publisher-count timer callback (~1 Hz).
     *
     * Escalate mode_ to kEmergencyRampDown if a second publisher is detected on /arm_sdk.
     */
    void checkPublisherCount();

    /**
     * @brief Synchronously ramps down the arm weight during lifecycle transitions.
     * @param target_mode Target blend mode for the ramp.
     */
    void rampDownSynchronously(BlendMode target_mode);

    /// Internal node and single-threaded executor for DDS I/O.
    rclcpp::Node::SharedPtr                                             node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr                executor_;
    std::thread                                                         executor_thread_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr          lowstate_sub_;
    realtime_tools::RealtimeBuffer<StampedLowState>                     lowstate_buffer_;
    realtime_tools::RealtimePublisherSharedPtr<unitree_hg::msg::LowCmd> arm_sdk_rt_pub_;
    rclcpp::TimerBase::SharedPtr                                        publisher_count_timer_;

    /// Single writer-authority state machine.
    std::atomic<BlendMode> mode_{ BlendMode::kInactive };

    /// RT-thread-only (never touched off write()): accumulates elapsed time
    /// toward the next throttled /arm_sdk publish.
    double time_since_last_publish_s_{ 0.0 };
};

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__G1_ARM_SDK_SYSTEM_HPP_
