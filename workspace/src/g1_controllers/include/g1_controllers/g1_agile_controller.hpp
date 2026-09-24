#ifndef G1_CONTROLLERS__G1_AGILE_CONTROLLER_HPP_
#define G1_CONTROLLERS__G1_AGILE_CONTROLLER_HPP_

/**
 * @file g1_agile_controller.hpp
 * @brief Runs the AGILE velocity policy over the rt/lowcmd component's state and command interfaces.
 */

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "g1_controllers/agile_policy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/bool.hpp"

namespace g1_controllers
{

/**
 * @brief Velocity-tracking locomotion policy for the 12 leg joints plus waist roll and pitch.
 *
 * Observes all 29 body joints and the pelvis IMU, tracks the Twist on `cmd_vel_topic`, and writes
 * joint targets with the policy's per-joint gains. Arms and waist yaw are left for other
 * controllers. Normally chained onto a G1SafetyController so the policy ramps in.
 */
class G1AgileController : public controller_interface::ControllerInterface
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

    controller_interface::return_type
    update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    /**
     * @brief Reads the state interfaces into `observation_` in the policy's own joint ordering.
     */
    void packObservation();

    /**
     * @brief Binds every command and state interface this controller needs.
     *
     * @return false, having logged, if a required interface was not claimed.
     */
    [[nodiscard]] bool resolveInterfaces();

    /**
     * @brief Command-interface names for one interface type, over the policy's action joints.
     *
     * @param type Interface type, e.g. "position".
     * @return One name per action joint, prefixed and suffixed for whatever this chains onto.
     */
    [[nodiscard]] std::vector<std::string> commandNamesFor(std::string_view type) const;

    /// Empty writes straight to the component; otherwise the chained controller's name, whose
    /// reference interfaces are `<prefix>/<joint>/<type><suffix>`.
    std::string command_prefix_;
    std::string command_suffix_;
    /// Empty resolves to the policy shipped in this package's share directory.
    std::string model_path_;
    std::string cmd_vel_topic_;
    std::string imu_sensor_name_;

    int decimation_       = kPolicyDecimation;
    int decimation_count_ = 0;
    /// Zeroes the command when cmd_vel goes stale, so a dead publisher halts rather than coasts.
    double cmd_vel_timeout_s_ = 0.5;
    double max_linear_speed_  = 0.0;
    double max_angular_speed_ = 0.0;

    std::unique_ptr<AgilePolicy> policy_;
    PolicyObservation            observation_;
    PolicyAction                 action_;

    std::vector<std::size_t> position_state_indices_;
    std::vector<std::size_t> velocity_state_indices_;
    /// Orientation w,x,y,z then angular velocity x,y,z, in that order.
    std::vector<std::size_t> imu_state_indices_;

    std::vector<std::size_t> position_command_indices_;
    std::vector<std::size_t> velocity_command_indices_;
    std::vector<std::size_t> effort_command_indices_;
    std::vector<std::size_t> kp_command_indices_;
    std::vector<std::size_t> kd_command_indices_;

    realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist>  cmd_vel_buffer_;
    std::atomic<double>                                        last_cmd_vel_seconds_{ 0.0 };
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    /// `~/inferring`, transient local: true from the first successful inference, so a test can wait
    /// for the policy to be commanding rather than merely active.
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inferring_publisher_;
    bool                                              inferring_ = false;
};

}  // namespace g1_controllers

#endif  // G1_CONTROLLERS__G1_AGILE_CONTROLLER_HPP_
