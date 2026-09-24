#ifndef G1_HARDWARE_INTERFACE__G1_LOWCMD_SYSTEM_HPP_
#define G1_HARDWARE_INTERFACE__G1_LOWCMD_SYSTEM_HPP_

/**
 * @file g1_lowcmd_system.hpp
 * @brief ros2_control SystemInterface owning all 29 G1 body motors over rt/lowcmd.
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "g1_hardware_interface/lowcmd_assembly.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace g1_hardware_interface
{

/// Interface names upstream controllers claim for per-joint gains. Must match upstream exactly.
inline constexpr std::string_view kHwIfKp{ "kp" };
inline constexpr std::string_view kHwIfKd{ "kd" };

/// Joint names in SDK motor index order. Position here IS the wire motor index, so the URDF
/// needs no per-joint motor_index param. g1_description's test_motor_order pins the order.
extern const std::array<std::string, kNumBodyMotors> kG1JointNames;

/**
 * @brief Everything the component tracks for one joint.
 */
struct JointData
{
    std::string name;

    double position_state = 0.0;
    double velocity_state = 0.0;
    double effort_state   = 0.0;

    JointCommand    command;
    InterfaceClaims claims;

    /// Applied in kPositionOnly, where no controller supplies gains. Per joint, from the URDF.
    PositionOnlyGains position_only_gains;

    std::int16_t surface_temperature = 0;
    std::int16_t winding_temperature = 0;

    /// No atomic needed: controller_manager runs perform_command_mode_switch, read and write
    /// on one thread, so the switch can never land mid-tick.
    JointControlMode mode = JointControlMode::kDisabled;

    /// Latched when a release ramp starts, so the ramp holds a pose instead of chasing the fall.
    double release_hold_position = 0.0;
    double release_kp            = 0.0;

    /// -1 when the URDF names a joint absent from kG1JointNames.
    int sdk_index = -1;
};

/**
 * @brief Backing storage for the IMU sensor interfaces, named as the broadcaster expects.
 */
struct ImuData
{
    std::string name;

    double orientation_w = 1.0;
    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;

    double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;

    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;
};

/**
 * @brief ros2_control System publishing rt/lowcmd and subscribing rt/lowstate over unitree_sdk2.
 *
 * Owns the whole body: nothing else may publish rt/lowcmd while this is active, and no onboard
 * balance runs underneath it. Joint order, kp/kd interfaces and mode branches follow the upstream
 * component, so its controllers bind unchanged.
 */
class G1LowCmdSystem : public hardware_interface::SystemInterface
{
public:
    ~G1LowCmdSystem() override;

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn
    on_error(const rclcpp_lifecycle::State& previous_state) override;
    /// Runs the release ramp too: controller_manager does not guarantee on_deactivate before exit.
    hardware_interface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type
    read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type
    write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    /**
     * @brief LowState_ carries no timestamp, so freshness is judged from arrival.
     */
    struct StampedLowState
    {
        unitree_hg::msg::dds_::LowState_      state{};
        std::chrono::steady_clock::time_point arrival{};
    };

    /**
     * @brief Opens the SDK channels and waits for the first low state.
     */
    bool initializeSdk();

    /**
     * @brief Closes the SDK channels.
     */
    void shutdownSdk();

    /**
     * @brief Releases whatever onboard mode holds the motors. Blocking; never on the update path.
     *
     * @return false if a mode survived every release attempt.
     */
    bool releaseOnboardMotionMode();

    /**
     * @brief Builds the joint table from the URDF, in the order the component was given.
     */
    void registerJoints(const hardware_interface::HardwareInfo& info);

    /**
     * @brief Exports the pelvis IMU as a sensor, when the URDF declares one.
     */
    void registerImuSensor(const hardware_interface::HardwareInfo& info);

    /**
     * @brief Maps each joint to its motor index on the wire.
     */
    void buildJointSdkMapping();

    /**
     * @brief SDK subscription callback; stamps the state with its arrival time.
     */
    void lowStateCallback(const void* message);

    /**
     * @brief Packs and writes one command frame.
     *
     * @return false if the DDS write was refused, which write() escalates to a component error.
     */
    bool publishLowCmd();

    /**
     * @brief Ramps stiffness to zero over release_ramp_s with release_kd held, then sends one
     *        disabled frame. Blocks for the ramp; returns at once if not active.
     */
    void releaseSynchronously();

    rclcpp::Logger logger_{ rclcpp::get_logger("g1_lowcmd_system") };

    std::vector<JointData>                       joint_data_;
    std::unordered_map<std::string, std::size_t> joint_name_to_index_;
    ImuData                                      imu_data_;
    bool                                         has_imu_ = false;

    std::string  network_interface_;
    int          domain_id_                 = 0;
    int          motor_temp_warn_threshold_ = 120;
    double       lowstate_timeout_s_        = 0.0;
    double       release_ramp_s_            = 0.0;
    double       release_kd_                = 0.0;
    bool         release_motion_mode_       = false;
    std::uint8_t mode_machine_              = 0;

    /// Preallocated and zeroed once: the checksum covers this struct's padding, so a per-tick
    /// stack object would checksum whatever the stack held.
    unitree_hg::msg::dds_::LowCmd_ low_cmd_{};

    realtime_tools::RealtimeBuffer<StampedLowState> lowstate_buffer_;
    std::atomic<bool>                               sdk_initialized_{ false };
    std::atomic<bool>                               first_state_received_{ false };
    std::atomic<bool>                               active_{ false };
    /// Set for the body of write(), so the release ramp on the executor thread can wait out an
    /// in-flight write before it fills low_cmd_.
    std::atomic<bool> in_write_{ false };

    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>    lowcmd_publisher_;

    /// Spun by controller_manager's own executor, which on_init receives in its params.
    rclcpp::Node::SharedPtr                                             node_;
    rclcpp::Executor::WeakPtr                                           executor_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr                                        diagnostics_timer_;
};

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__G1_LOWCMD_SYSTEM_HPP_
