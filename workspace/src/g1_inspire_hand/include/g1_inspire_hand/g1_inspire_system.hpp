#ifndef G1_INSPIRE_HAND__G1_INSPIRE_SYSTEM_HPP_
#define G1_INSPIRE_HAND__G1_INSPIRE_SYSTEM_HPP_

/**
 * @file g1_inspire_system.hpp
 * @brief ros2_control SystemInterface for one Inspire RH56DFQ hand on its own RS485 line.
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include "g1_inspire_hand/inspire_hand.hpp"
#include "g1_inspire_hand/inspire_protocol.hpp"
#include "g1_inspire_hand/serial_port.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace g1_inspire_hand
{

/// The six actuated joints, without their L_ or R_ prefix, in register order.
inline constexpr std::array<const char*, kNumActuators> kActuatorJointSuffixes = {
    "pinky_proximal_joint", "ring_proximal_joint",        "middle_proximal_joint",
    "index_proximal_joint", "thumb_proximal_pitch_joint", "thumb_proximal_yaw_joint",
};

/**
 * @brief One Inspire hand: six position-commanded actuators, slewed, over RS485.
 *
 * @details
 * The line is opened at activation and closed at deactivation, so an absent or unpowered hand fails
 * when the arm bracket activates it, not at startup. A thread of its own runs the bus at
 * `io_rate_hz`: it writes the targets when they change and reads angle and force each cycle, and
 * read() and write() only exchange values with it, without blocking.
 *
 * A stop sends nothing more. The hand's lead screws hold where its last target left them, which
 * the slew keeps within one step of the measured position, so a stopped hand keeps what it holds.
 */
class G1InspireSystem : public hardware_interface::SystemInterface
{
public:
    G1InspireSystem()                                  = default;
    G1InspireSystem(const G1InspireSystem&)            = delete;
    G1InspireSystem& operator=(const G1InspireSystem&) = delete;
    G1InspireSystem(G1InspireSystem&&)                 = delete;
    G1InspireSystem& operator=(G1InspireSystem&&)      = delete;
    ~G1InspireSystem() override;

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
    /**
     * @brief Opens the line, checks the hand answers, clears its errors and sets its speed and
     *        force limits, then seeds the commands from the measured angles and starts the bus.
     */
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override;
    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /// Errors once active if no good state has arrived for `state_timeout_ms`.
    hardware_interface::return_type
    read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type
    write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    /**
     * @brief The bus loop: targets out when changed, angle and force in, diagnostics now and then.
     */
    void runBus(const std::stop_token& stop);

    /// Reads the diagnostics block and logs any actuator whose error flags changed.
    void checkDiagnostics(std::array<std::uint8_t, kNumActuators>& last_errors);

    /// The one teardown path: stops the bus thread, then closes the line.
    void stopBus();

    /// Retries a read of the measured angles until one succeeds or @p wait passes.
    [[nodiscard]] ReplyError
    readAnglesWithin(std::chrono::milliseconds wait, std::array<std::int16_t, kNumActuators>& out);

    rclcpp::Logger logger_{ rclcpp::get_logger("g1_inspire_system") };

    std::string               side_;
    std::string               serial_port_;
    Framing                   framing_{ Framing::kInspire };
    int                       baud_rate_{ 115200 };
    std::uint8_t              slave_id_{ 1 };
    double                    io_rate_hz_{ 50.0 };
    std::chrono::microseconds reply_timeout_{ 25000 };
    std::chrono::nanoseconds  state_timeout_{ std::chrono::milliseconds(300) };
    int                       diagnostics_every_{ 10 };
    std::int16_t              speed_{ 1000 };
    std::int16_t              force_limit_g_{ 300 };
    double                    max_joint_velocity_rad_s_{ 3.0 };

    /// Per actuator, in register order.
    std::array<ActuatorMap, kNumActuators> maps_{};
    std::array<double, kNumActuators>      lower_rad_{};
    std::array<double, kNumActuators>      upper_rad_{};

    /// Owned by the update thread, exported to the controllers.
    std::array<double, kNumActuators> position_command_{};
    std::array<double, kNumActuators> ramped_command_{};
    std::array<double, kNumActuators> position_state_{};
    std::array<double, kNumActuators> velocity_state_{};
    std::array<double, kNumActuators> effort_state_{};
    std::int64_t                      last_seen_state_ns_{ 0 };

    /// Between the update thread and the bus thread. A torn set mixes two valid cycles, which is
    /// harmless, so each value is its own atomic and nothing locks.
    std::array<std::atomic<std::int16_t>, kNumActuators> target_angle_{};
    std::array<std::atomic<std::int16_t>, kNumActuators> measured_angle_{};
    std::array<std::atomic<std::int16_t>, kNumActuators> measured_force_g_{};
    /// Steady-clock time of the newest good state, published after the values it covers.
    std::atomic<std::int64_t> state_stamp_ns_{ 0 };

    /// True between a successful activation and the next stop; read() and write() idle outside.
    std::atomic<bool> active_{ false };

    std::unique_ptr<ByteStream>  stream_;
    std::unique_ptr<InspireHand> hand_;
    std::jthread                 bus_thread_;
};

}  // namespace g1_inspire_hand

#endif  // G1_INSPIRE_HAND__G1_INSPIRE_SYSTEM_HPP_
