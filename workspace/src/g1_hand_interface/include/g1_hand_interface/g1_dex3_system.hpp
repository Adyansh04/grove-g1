#ifndef G1_HAND_INTERFACE__G1_DEX3_SYSTEM_HPP_
#define G1_HAND_INTERFACE__G1_DEX3_SYSTEM_HPP_

/**
 * @file g1_dex3_system.hpp
 * @brief ros2_control SystemInterface for one Unitree Dex3-1 hand, over its own SDK channels.
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace g1_hand_interface
{

/// Seven joints per hand. HandCmd_::motor_cmd is an unbounded sequence, so it must be resized to
/// this before publishing; an empty sequence is accepted and silently does nothing.
inline constexpr std::size_t kNumHandJoints = 7;

/// Wire order, identical for both hands and the same as the URDF's. Do not copy Unitree's
/// Dex3_1_Right_JointIndex enum: it lists index before middle, against their documented order.
inline constexpr std::array<const char*, kNumHandJoints> kJointSuffixes = {
    "thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1",
};

/**
 * @brief The packed `mode` byte the motors expect: id in bits 0-3, status in 4-6, timeout in 7.
 *
 * @param index Must equal the motor's own index; 15 would broadcast to every motor.
 */
inline constexpr std::uint8_t packMode(std::size_t index, std::uint8_t status, bool timeout)
{
    return static_cast<std::uint8_t>(
        (index & 0x0F) | ((status & 0x07) << 4) | ((timeout ? 1U : 0U) << 7));
}

inline constexpr std::uint8_t kStatusLock = 0x00;  ///< motor held, gains ignored
inline constexpr std::uint8_t kStatusFoc  = 0x01;  ///< driven

/**
 * @brief Fills one motor slot of a HandCmd_.
 * @param motor Slot to write, already sized by the caller.
 * @param index Motor's own index; it goes in the mode byte and must not be 15.
 * @param driven False emits the release command: Lock status, timeout armed, zero gains.
 * @param position Target while driven, or the measured position to freeze at on release.
 * @param kp Stiffness while driven; ignored on release.
 * @param kd Damping while driven; ignored on release.
 */
inline void packHandMotor(
    unitree_hg::msg::dds_::MotorCmd_& motor, std::size_t index, bool driven, double position,
    double kp, double kd)
{
    // Timeout armed on release only: while driving it would stop the fingers a second after any
    // hiccup in the control loop.
    motor.mode() = packMode(index, driven ? kStatusFoc : kStatusLock, !driven);
    motor.q()    = static_cast<float>(position);
    motor.dq()   = 0.0F;
    motor.tau()  = 0.0F;  // feedforward; Unitree's own examples leave it at zero
    motor.kp()   = static_cast<float>(driven ? kp : 0.0);
    motor.kd()   = static_cast<float>(driven ? kd : 0.0);
}

/**
 * @brief One Dex3-1 hand: position commands, clamped and slewed, on rt/dex3/<side>/{cmd,state}.
 *
 * Separate from the body component, one per hand, so a hand fault cannot take the arms down.
 */
class G1Dex3System : public hardware_interface::SystemInterface
{
public:
    G1Dex3System()                               = default;
    G1Dex3System(const G1Dex3System&)            = delete;
    G1Dex3System& operator=(const G1Dex3System&) = delete;
    G1Dex3System(G1Dex3System&&)                 = delete;
    G1Dex3System& operator=(G1Dex3System&&)      = delete;

    ~G1Dex3System() override;

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override;
    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous) override;
    /**
     * @brief Releases the hand. read() can error the component without on_deactivate running, and
     *        only the release frame arms the motors' own timeout.
     */
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type
    read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type
    write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    /**
     * @brief HandState_ carries no timestamp, so freshness is judged from arrival.
     */
    struct StampedHandState
    {
        unitree_hg::msg::dds_::HandState_     state{};
        std::chrono::steady_clock::time_point arrival{};
    };

    /**
     * @brief Opens the SDK channels and waits for the first hand state.
     *
     * @return false if the channels could not be opened or no state arrived in time.
     */
    bool initializeSdk();

    /**
     * @brief Closes the SDK channels.
     */
    void shutdownSdk();

    /**
     * @brief The one teardown path: release frame, then channels.
     *
     * Shared by deactivate, error and shutdown so an error can never skip the release.
     */
    void releaseAndShutdown();

    /**
     * @brief SDK subscription callback; stamps the state with its arrival time.
     */
    void handStateCallback(const void* message);

    /**
     * @brief Fills every motor slot and writes.
     *
     * @param driven false emits the release command: status Lock, timeout armed, zero gains.
     */
    void publish(bool driven);

    rclcpp::Logger logger_{ rclcpp::get_logger("g1_dex3_system") };
    /// Member rather than a per-call make_shared, which would allocate on the write path.
    rclcpp::Clock clock_{ RCL_STEADY_TIME };

    std::string side_;  ///< "left" or "right"; picks the channel pair and the joint prefix.

    /// Parameterised because the robot also carries a lower-rate rt/lf/dex3/<side>/state, and
    /// Unitree's own code disagrees about which to read.
    std::string state_topic_;
    std::string cmd_topic_;

    /// Per joint, in wire order.
    std::array<double, kNumHandJoints> position_command_{};
    std::array<double, kNumHandJoints> ramped_command_{};
    std::array<double, kNumHandJoints> position_state_{};
    std::array<double, kNumHandJoints> velocity_state_{};
    std::array<double, kNumHandJoints> effort_state_{};
    std::array<double, kNumHandJoints> lower_limit_{};
    std::array<double, kNumHandJoints> upper_limit_{};

    /// From the <param> tags. Finger-motor gains, far below the arm's.
    double kp_{ 1.5 };
    double kd_{ 0.2 };
    double command_publish_rate_{ 100.0 };
    double max_joint_velocity_{ 3.0 };
    double state_timeout_s_{ 0.2 };

    /// Must match the body component's: ChannelFactory is per process and only its first Init
    /// applies.
    std::string network_interface_;
    int         domain_id_{ 0 };

    /// True once commands are seeded from the measurement; write() sends nothing before. Atomic:
    /// lifecycle callbacks clear it while read() and write() run on the update thread.
    std::atomic<bool> seeded_{ false };
    /// Raised for the duration of write(), so a release can wait it out.
    std::atomic<bool>                     in_write_{ false };
    std::chrono::steady_clock::time_point last_publish_{};

    /// Preallocated and resized once, so the write path never allocates.
    unitree_hg::msg::dds_::HandCmd_ hand_cmd_{};

    realtime_tools::RealtimeBuffer<StampedHandState> state_buffer_;
    std::atomic<bool>                                sdk_initialized_{ false };
    std::atomic<bool>                                first_state_received_{ false };

    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::HandState_> handstate_subscriber_;
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::HandCmd_>    handcmd_publisher_;
};

}  // namespace g1_hand_interface

#endif  // G1_HAND_INTERFACE__G1_DEX3_SYSTEM_HPP_
