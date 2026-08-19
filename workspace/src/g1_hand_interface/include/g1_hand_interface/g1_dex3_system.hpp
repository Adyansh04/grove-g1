#ifndef G1_HAND_INTERFACE__G1_DEX3_SYSTEM_HPP_
#define G1_HAND_INTERFACE__G1_DEX3_SYSTEM_HPP_

/**
 * @file g1_dex3_system.hpp
 * @brief ros2_control SystemInterface for one Unitree Dex3-1 hand, over its own SDK channels.
 *
 * Real hardware code. It speaks Unitree's published Dex3 contract and carries to the robot
 * unchanged; the simulator answers the same channels.
 *
 * Deliberately not part of the body component. The hand is a separate device on separate
 * channels with separate authority, and one component per hand keeps a hand fault from taking
 * the arms down with it.
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

/// Wire order, identical for both hands: thumb_0, thumb_1, thumb_2, middle_0, middle_1,
/// index_0, index_1. This is also the URDF's own document order, so the map is positional.
///
/// Do NOT copy Unitree's Dex3_1_Right_JointIndex enum from xr_teleoperate/avp_teleoperate: it
/// lists index before middle, contradicting their documented order. It is inert there (every
/// use is enumerate(), so the value never permutes anything) but transcribing it here would
/// close the wrong fingers.
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
    // timeout armed on release only: while driving, the controller is the heartbeat, and arming
    // it would stop the fingers a second after any hiccup in the control loop.
    motor.mode() = packMode(index, driven ? kStatusFoc : kStatusLock, !driven);
    motor.q()    = static_cast<float>(position);
    motor.dq()   = 0.0F;
    motor.tau()  = 0.0F;  // feedforward; Unitree's own examples leave it at zero
    motor.kp()   = static_cast<float>(driven ? kp : 0.0);
    motor.kd()   = static_cast<float>(driven ? kd : 0.0);
}

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
     * @brief Releases the hand on the error path.
     *
     * read() can error the component straight to a failed state without on_deactivate ever
     * running, and the release frame is the only one that arms the motor's own timeout.
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

    /// Which state channel to read. Parameterised because the robot carries both
    /// rt/dex3/<side>/state and a lower-rate rt/lf/ variant, and Unitree's own code
    /// disagrees about which one to use.
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

    /// From <param> tags on the ros2_control block. kp/kd are ~300x smaller than the arm's:
    /// these are finger motors, and arm gains here would be violent.
    double kp_{ 1.5 };
    double kd_{ 0.2 };
    double command_publish_rate_{ 100.0 };
    double max_joint_velocity_{ 3.0 };
    double state_timeout_s_{ 0.2 };

    /// Must match the body component's: ChannelFactory is per process and only the first Init
    /// takes effect, so a disagreement here would silently put one of them on the wrong domain.
    std::string network_interface_;
    int         domain_id_{ 0 };

    /// The first write takes full authority: unlike a blended interface, there is no weight to
    /// bring up. So the ramp is ours, and it is the only thing between a large command step and
    /// a fast finger. Atomic because the lifecycle callbacks clear it from the executor thread
    /// while read() and write() are reading it on the update thread.
    std::atomic<bool>                     seeded_{ false };
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
