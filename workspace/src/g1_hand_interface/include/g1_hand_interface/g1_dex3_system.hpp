#ifndef G1_HAND_INTERFACE__G1_DEX3_SYSTEM_HPP_
#define G1_HAND_INTERFACE__G1_DEX3_SYSTEM_HPP_

/**
 * @file g1_dex3_system.hpp
 * @brief ros2_control SystemInterface for one Unitree Dex3-1 hand, over its own DDS topics.
 *
 * Real hardware code. It speaks Unitree's published Dex3 contract and carries to the robot
 * unchanged; the simulator is expected to answer the same topics.
 *
 * Deliberately not part of G1ArmSdkSystem. The hand is a separate device on separate topics
 * with separate authority, and one component per hand keeps a hand
 * fault from taking the arms down with it.
 */

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "unitree_hg/msg/hand_cmd.hpp"
#include "unitree_hg/msg/hand_state.hpp"

namespace g1_hand_interface
{

/// Seven joints per hand. HandCmd.motor_cmd is an unbounded sequence, so it must be resized to
/// this before publishing -- an empty sequence is accepted and silently does nothing.
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

/// The packed `mode` byte Unitree's motors expect: id in bits 0-3, status in 4-6, timeout in 7.
/// `id` must equal the motor's own index; 15 would broadcast to every motor.
inline constexpr std::uint8_t packMode(std::size_t index, std::uint8_t status, bool timeout)
{
    return static_cast<std::uint8_t>(
        (index & 0x0F) | ((status & 0x07) << 4) | ((timeout ? 1U : 0U) << 7));
}

inline constexpr std::uint8_t kStatusLock = 0x00;  ///< motor held, gains ignored
inline constexpr std::uint8_t kStatusFoc  = 0x01;  ///< driven

class G1Dex3System : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override;
    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type
    read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type
    write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    void handStateCallback(const unitree_hg::msg::HandState::ConstSharedPtr& msg);

    /// Fills every motor slot and publishes. `driven` false emits the release command: status
    /// Lock, timeout armed, zero gains.
    void publish(bool driven);

    std::string side_;  ///< "left" or "right"; picks the topic pair and the joint prefix.

    /// Which state topic to read. Parameterised because the robot carries both
    /// /dex3/<side>/state and a lower-rate /lf/ variant, and Unitree's own code
    /// disagrees about which one to use.
    std::string state_topic_;

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

    /// There is no blend weight on this interface -- unlike rt/arm_sdk, the first publish takes
    /// full authority. So the ramp is ours to do, and it is the only thing standing between a
    /// large command step and a fast finger.
    bool                                  seeded_{ false };
    std::chrono::steady_clock::time_point last_publish_{};
    std::chrono::steady_clock::time_point last_state_{};

    rclcpp::Node::SharedPtr                                                      node_;
    rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr                  state_sub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<unitree_hg::msg::HandCmd>> cmd_pub_;
    rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr                       cmd_pub_raw_;
    std::thread                                                                  spin_thread_;
    std::atomic<bool>                                                            spinning_{ false };

    /// Written by the subscription thread, read by read(). Small and trivially copyable, and
    /// the executor is single-threaded, so a seqlock would buy nothing here.
    std::array<double, kNumHandJoints> rx_position_{};
    std::array<double, kNumHandJoints> rx_velocity_{};
    std::array<double, kNumHandJoints> rx_effort_{};
    std::atomic<bool>                  rx_valid_{ false };
};

}  // namespace g1_hand_interface

#endif  // G1_HAND_INTERFACE__G1_DEX3_SYSTEM_HPP_
