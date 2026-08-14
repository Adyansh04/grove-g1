/**
 * @file g1_dex3_system.cpp
 * @brief One Dex3-1 hand as a ros2_control system, over /dex3/<side>/{cmd,state}.
 */

#include "g1_hand_interface/g1_dex3_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace g1_hand_interface
{
namespace
{
double paramOr(const hardware_interface::HardwareInfo& info, const std::string& key, double fallback)
{
    const auto it = info.hardware_parameters.find(key);
    return it == info.hardware_parameters.end() ? fallback : std::stod(it->second);
}

std::string paramOr(
    const hardware_interface::HardwareInfo& info, const std::string& key,
    const std::string& fallback)
{
    const auto it = info.hardware_parameters.find(key);
    return it == info.hardware_parameters.end() || it->second.empty() ? fallback : it->second;
}
}  // namespace

hardware_interface::CallbackReturn
G1Dex3System::on_init(const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto side_it = info.hardware_parameters.find("side");
    if (side_it == info.hardware_parameters.end() ||
        (side_it->second != "left" && side_it->second != "right"))
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("G1Dex3System"),
            "the 'side' parameter must be exactly \"left\" or \"right\"; it picks both "
            "the topic pair and the joint prefix");
        return hardware_interface::CallbackReturn::ERROR;
    }
    side_ = side_it->second;

    if (info.joints.size() != kNumHandJoints)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("G1Dex3System"),
            "expected %zu joints for the %s hand but got %zu",
            kNumHandJoints,
            side_.c_str(),
            info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // The wire is a positional array, so the joints must be declared in wire order. Checked
    // rather than assumed: a reordered URDF would otherwise close the wrong fingers, and that
    // is exactly the failure Unitree's own mislabelled enum causes.
    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        const std::string expected = side_ + "_hand_" + kJointSuffixes[i] + "_joint";
        if (info.joints[i].name != expected)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("G1Dex3System"),
                "joint %zu is '%s' but the Dex3 wire order needs '%s' there",
                i,
                info.joints[i].name.c_str(),
                expected.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Clamp to the URDF, which agrees with Unitree's published spec. Their SDK example
        // disagrees on thumb_1 (0.724 vs 0.611) and its right hand says 0.742, which looks
        // like a transposed digit. The conservative pair is the right one to trust.
        const auto& limits = info.joints[i].parameters;
        lower_limit_[i]    = limits.contains("min") ? std::stod(limits.at("min")) : -3.15;
        upper_limit_[i]    = limits.contains("max") ? std::stod(limits.at("max")) : 3.15;
    }

    kp_                   = paramOr(info, "kp", 1.5);
    kd_                   = paramOr(info, "kd", 0.2);
    command_publish_rate_ = paramOr(info, "command_publish_rate", 100.0);
    max_joint_velocity_   = paramOr(info, "max_joint_velocity_rad_s", 3.0);
    state_timeout_s_      = paramOr(info, "state_timeout_ms", 200.0) / 1000.0;
    state_topic_          = paramOr(info, "state_topic", "/dex3/" + side_ + "/state");

    if (command_publish_rate_ <= 0.0 || max_joint_velocity_ <= 0.0 || state_timeout_s_ <= 0.0)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("G1Dex3System"),
            "command_publish_rate, max_joint_velocity_rad_s and state_timeout_ms must "
            "all be strictly positive");
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1Dex3System::on_configure(const rclcpp_lifecycle::State&)
{
    node_ = std::make_shared<rclcpp::Node>("g1_dex3_" + side_ + "_system");

    // Sensor QoS both ways: this is a device stream, and it is what Unitree's own tooling uses.
    const auto qos = rclcpp::SensorDataQoS();
    state_sub_     = node_->create_subscription<unitree_hg::msg::HandState>(
        state_topic_,
        qos,
        [this](const unitree_hg::msg::HandState::ConstSharedPtr& msg) { handStateCallback(msg); });
    cmd_pub_raw_ =
        node_->create_publisher<unitree_hg::msg::HandCmd>("/dex3/" + side_ + "/cmd", qos);
    cmd_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<unitree_hg::msg::HandCmd>>(cmd_pub_raw_);

    // motor_cmd is an unbounded sequence, not a fixed array. Publishing it unresized is
    // accepted by DDS and silently moves nothing, which is a miserable thing to debug.
    cmd_pub_->msg_.motor_cmd.resize(kNumHandJoints);

    spinning_    = true;
    spin_thread_ = std::thread([this] {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node_);
        while (rclcpp::ok() && spinning_)
        {
            executor.spin_some(std::chrono::milliseconds(10));
        }
    });
    return hardware_interface::CallbackReturn::SUCCESS;
}

void G1Dex3System::handStateCallback(const unitree_hg::msg::HandState::ConstSharedPtr& msg)
{
    if (msg->motor_state.size() < kNumHandJoints)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "HandState carried %zu motors, need %zu -- ignoring",
            msg->motor_state.size(),
            kNumHandJoints);
        return;
    }
    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        rx_position_[i] = msg->motor_state[i].q;
        rx_velocity_[i] = msg->motor_state[i].dq;
        rx_effort_[i]   = msg->motor_state[i].tau_est;
    }
    last_state_ = std::chrono::steady_clock::now();
    rx_valid_   = true;
}

hardware_interface::CallbackReturn G1Dex3System::on_activate(const rclcpp_lifecycle::State&)
{
    // Refuse to drive fingers we cannot see. Same gate the arm bridge applies, and for the same
    // reason: seeding from measured is what makes the first command a no-op instead of a jump.
    if (!rx_valid_ ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_state_).count() >
            state_timeout_s_)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "no fresh HandState on %s -- refusing to activate",
            state_topic_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        position_state_[i]   = rx_position_[i];
        position_command_[i] = rx_position_[i];
        ramped_command_[i]   = rx_position_[i];
    }
    seeded_       = true;
    last_publish_ = std::chrono::steady_clock::now();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1Dex3System::on_deactivate(const rclcpp_lifecycle::State&)
{
    // Release: Lock status with zero gains, and timeout armed so the motor stops on its own if
    // anything downstream keeps the last frame alive.
    publish(false);
    seeded_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> G1Dex3System::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(kNumHandJoints * 3);
    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &position_state_[i]);
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &velocity_state_[i]);
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &effort_state_[i]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> G1Dex3System::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(kNumHandJoints);
    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &position_command_[i]);
    }
    return interfaces;
}

hardware_interface::return_type G1Dex3System::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    if (rx_valid_)
    {
        for (std::size_t i = 0; i < kNumHandJoints; ++i)
        {
            position_state_[i] = rx_position_[i];
            velocity_state_[i] = rx_velocity_[i];
            effort_state_[i]   = rx_effort_[i];
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
G1Dex3System::write(const rclcpp::Time&, const rclcpp::Duration& period)
{
    if (!seeded_)
    {
        return hardware_interface::return_type::OK;
    }

    // Slew toward the commanded position. There is no blend weight on this interface: the very
    // first publish takes full authority, so this ramp is the only thing between a large
    // trajectory step and a finger moving as fast as the motor can manage.
    const double step = max_joint_velocity_ * period.seconds();
    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        const double target = std::clamp(position_command_[i], lower_limit_[i], upper_limit_[i]);
        ramped_command_[i] += std::clamp(target - ramped_command_[i], -step, step);
    }

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_publish_).count() < 1.0 / command_publish_rate_)
    {
        return hardware_interface::return_type::OK;
    }
    last_publish_ = now;
    publish(true);
    return hardware_interface::return_type::OK;
}

void G1Dex3System::publish(bool driven)
{
    if (!cmd_pub_ || !cmd_pub_->trylock())
    {
        return;
    }
    auto& msg = cmd_pub_->msg_;
    msg.motor_cmd.resize(kNumHandJoints);
    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        auto& motor = msg.motor_cmd[i];
        // timeout armed on release only: while driving, the controller is the heartbeat, and
        // arming it would stop the fingers a second after any hiccup in the control loop.
        motor.mode = packMode(i, driven ? kStatusFoc : kStatusLock, !driven);
        motor.q    = static_cast<float>(driven ? ramped_command_[i] : position_state_[i]);
        motor.dq   = 0.0F;
        motor.tau  = 0.0F;  // feedforward; Unitree's own examples leave it at zero
        motor.kp   = static_cast<float>(driven ? kp_ : 0.0);
        motor.kd   = static_cast<float>(driven ? kd_ : 0.0);
    }
    cmd_pub_->unlockAndPublish();
}

}  // namespace g1_hand_interface

PLUGINLIB_EXPORT_CLASS(g1_hand_interface::G1Dex3System, hardware_interface::SystemInterface)
