/**
 * @file g1_dex3_system.cpp
 * @brief One Dex3-1 hand as a ros2_control system, over rt/dex3/<side>/{cmd,state}.
 */

#include "g1_hand_interface/g1_dex3_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <unitree/robot/channel/channel_factory.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace g1_hand_interface
{
namespace
{
/// Long enough for the hand's firmware to come up after the body component has claimed the wire.
constexpr auto kFirstStateTimeout = std::chrono::seconds(5);

/// Falls back on an absent, empty or unparseable value: std::stod throws, and on_init reports
/// failure through its return value rather than by letting an exception escape a lifecycle call.
double paramOr(const hardware_interface::HardwareInfo& info, const std::string& key, double fallback)
{
    const auto it = info.hardware_parameters.find(key);
    if (it == info.hardware_parameters.end())
    {
        return fallback;
    }
    try
    {
        return std::stod(it->second);
    }
    catch (const std::exception&)
    {
        return fallback;
    }
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
G1Dex3System::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto& info = get_hardware_info();

    const auto side_it = info.hardware_parameters.find("side");
    if (side_it == info.hardware_parameters.end() ||
        (side_it->second != "left" && side_it->second != "right"))
    {
        RCLCPP_FATAL(
            logger_,
            "the 'side' parameter must be exactly \"left\" or \"right\"; it picks both "
            "the channel pair and the joint prefix");
        return hardware_interface::CallbackReturn::ERROR;
    }
    side_   = side_it->second;
    logger_ = rclcpp::get_logger("g1_dex3_" + side_ + "_system");

    if (info.joints.size() != kNumHandJoints)
    {
        RCLCPP_FATAL(
            logger_,
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
                logger_,
                "joint %zu is '%s' but the Dex3 wire order needs '%s' there",
                i,
                info.joints[i].name.c_str(),
                expected.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Clamp to the URDF, which agrees with the published spec; the SDK example disagrees on
        // thumb_1 (0.724 vs 0.611) and the conservative pair is the one to trust. Required
        // rather than defaulted, so a renamed param fails here instead of widening the range.
        const auto& limits = info.joints[i].parameters;
        if (!limits.contains("min") || !limits.contains("max"))
        {
            RCLCPP_FATAL(logger_, "joint '%s' needs min and max params", info.joints[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        lower_limit_[i] = std::stod(limits.at("min"));
        upper_limit_[i] = std::stod(limits.at("max"));
        // std::clamp is undefined when the bounds are transposed, and libstdc++ answers a
        // transposed pair by snapping every command to max rather than clamping.
        if (!(lower_limit_[i] < upper_limit_[i]))
        {
            RCLCPP_FATAL(logger_, "joint '%s' has min >= max", info.joints[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    kp_                   = paramOr(info, "kp", 1.5);
    kd_                   = paramOr(info, "kd", 0.2);
    command_publish_rate_ = paramOr(info, "command_publish_rate", 100.0);
    max_joint_velocity_   = paramOr(info, "max_joint_velocity_rad_s", 3.0);
    state_timeout_s_      = paramOr(info, "state_timeout_ms", 200.0) / 1000.0;
    state_topic_          = paramOr(info, "state_topic", "rt/dex3/" + side_ + "/state");
    cmd_topic_            = "rt/dex3/" + side_ + "/cmd";

    // Deliberately empty by default: a non-empty interface makes the SDK build its own inline
    // CycloneDDS config and discard CYCLONEDDS_URI, which is what pins us to loopback.
    network_interface_ = paramOr(info, "network_interface", std::string{});

    // Required, not defaulted: it has to agree with the body component's, and a wrong domain
    // shows up as a hand that never reports state rather than as anything nameable.
    if (!info.hardware_parameters.contains("domain_id"))
    {
        RCLCPP_FATAL(logger_, "<hardware> needs a domain_id param");
        return hardware_interface::CallbackReturn::ERROR;
    }
    domain_id_ = std::stoi(info.hardware_parameters.at("domain_id"));

    // kp and kd are in here for the same reason the rest are: kp 0 is fingers that report as
    // driven and hold nothing.
    if (command_publish_rate_ <= 0.0 || max_joint_velocity_ <= 0.0 || state_timeout_s_ <= 0.0 ||
        kp_ <= 0.0 || kd_ <= 0.0)
    {
        RCLCPP_FATAL(
            logger_,
            "kp, kd, command_publish_rate, max_joint_velocity_rad_s and state_timeout_ms must "
            "all be strictly positive");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // motor_cmd is an unbounded sequence, not a fixed array. Writing it unresized is accepted by
    // DDS and silently moves nothing, which is a miserable thing to debug.
    hand_cmd_.motor_cmd().resize(kNumHandJoints);
    return hardware_interface::CallbackReturn::SUCCESS;
}

G1Dex3System::~G1Dex3System() { shutdownSdk(); }

bool G1Dex3System::initializeSdk()
{
    try
    {
        // Third caller in this process, after the body component and the other hand. The SDK
        // guards with its own mInited flag, so only the first domain_id/interface pair applies.
        unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);

        handstate_subscriber_ =
            std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::HandState_>>(
                state_topic_);
        handstate_subscriber_->InitChannel(
            [this](const void* message) { handStateCallback(message); },
            1);

        handcmd_publisher_ =
            std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>>(
                cmd_topic_);
        handcmd_publisher_->InitChannel();

        // Refuse to drive fingers we cannot see: seeding from measured is what makes the first
        // command a no-op instead of a jump.
        const auto deadline = std::chrono::steady_clock::now() + kFirstStateTimeout;
        while (!first_state_received_.load())
        {
            if (std::chrono::steady_clock::now() > deadline)
            {
                RCLCPP_ERROR(
                    logger_,
                    "no HandState on %s -- is the hand powered?",
                    state_topic_.c_str());
                shutdownSdk();
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        sdk_initialized_ = true;
        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "SDK init failed: %s", e.what());
        shutdownSdk();
        return false;
    }
}

/// Idempotent, and unguarded on purpose: a failed initializeSdk leaves channels open with
/// sdk_initialized_ still false, and those have to go too.
void G1Dex3System::shutdownSdk()
{
    handstate_subscriber_.reset();
    handcmd_publisher_.reset();
    sdk_initialized_      = false;
    first_state_received_ = false;
}

void G1Dex3System::handStateCallback(const void* message)
{
    StampedHandState sample;
    sample.state   = *static_cast<const unitree_hg::msg::dds_::HandState_*>(message);
    sample.arrival = std::chrono::steady_clock::now();
    state_buffer_.writeFromNonRT(sample);
    first_state_received_ = true;
}

hardware_interface::CallbackReturn G1Dex3System::on_activate(const rclcpp_lifecycle::State&)
{
    if (!initializeSdk())
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const StampedHandState* sample = state_buffer_.readFromRT();
    if (sample->state.motor_state().size() < kNumHandJoints)
    {
        RCLCPP_ERROR(
            logger_,
            "HandState carried %zu motors, need %zu -- refusing to activate",
            sample->state.motor_state().size(),
            kNumHandJoints);
        shutdownSdk();
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        position_state_[i]   = sample->state.motor_state()[i].q();
        position_command_[i] = position_state_[i];
        ramped_command_[i]   = position_state_[i];
    }
    seeded_       = true;
    last_publish_ = std::chrono::steady_clock::now();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1Dex3System::on_deactivate(const rclcpp_lifecycle::State&)
{
    releaseAndShutdown();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1Dex3System::on_error(const rclcpp_lifecycle::State&)
{
    releaseAndShutdown();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1Dex3System::on_shutdown(const rclcpp_lifecycle::State&)
{
    releaseAndShutdown();
    return hardware_interface::CallbackReturn::SUCCESS;
}

void G1Dex3System::releaseAndShutdown()
{
    // Release: Lock status with zero gains, and timeout armed so the motor stops on its own if
    // anything downstream keeps the last frame alive. Clearing seeded_ first stops write()
    // assembling a driven frame into the same buffer.
    seeded_ = false;
    publish(false);
    shutdownSdk();
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
    if (!sdk_initialized_.load())
    {
        return hardware_interface::return_type::OK;
    }

    const StampedHandState* sample = state_buffer_.readFromRT();

    // Ahead of the size check on purpose. The callback stamps arrival for every frame, short ones
    // included, so a hand that regresses to short frames would keep this reading fresh for ever
    // while write() drove on from a frozen position_state_.
    const auto age = std::chrono::steady_clock::now() - sample->arrival;
    if (seeded_ && age > std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                             std::chrono::duration<double>(state_timeout_s_)))
    {
        RCLCPP_ERROR(logger_, "%s went stale while active", state_topic_.c_str());
        return hardware_interface::return_type::ERROR;
    }

    if (sample->state.motor_state().size() < kNumHandJoints)
    {
        RCLCPP_WARN_THROTTLE(
            logger_,
            clock_,
            2000,
            "HandState carried %zu motors, need %zu -- holding the last reading",
            sample->state.motor_state().size(),
            kNumHandJoints);
        return hardware_interface::return_type::OK;
    }

    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        const auto& motor  = sample->state.motor_state()[i];
        position_state_[i] = motor.q();
        velocity_state_[i] = motor.dq();
        effort_state_[i]   = motor.tau_est();
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

    // Slew toward the commanded position: the only thing between a large trajectory step and a
    // finger moving as fast as the motor can. period comes from controller_manager, so a stalled
    // loop would widen step past limiting and a negative one would transpose std::clamp's
    // bounds, which is undefined. Cap it at 20 update ticks.
    const double step = max_joint_velocity_ * std::clamp(period.seconds(), 0.0, 0.1);
    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        // std::clamp passes NaN straight through, and ramped_command_ carries state, so one NaN
        // setpoint would latch this finger at NaN for good: it never recovers, even once the
        // controller resumes sending valid targets. Hold the last good value instead.
        if (!std::isfinite(position_command_[i]))
        {
            continue;
        }
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
    if (!handcmd_publisher_)
    {
        return;
    }
    for (std::size_t i = 0; i < kNumHandJoints; ++i)
    {
        packHandMotor(
            hand_cmd_.motor_cmd()[i],
            i,
            driven,
            driven ? ramped_command_[i] : position_state_[i],
            kp_,
            kd_);
    }
    handcmd_publisher_->Write(hand_cmd_);
}

}  // namespace g1_hand_interface

PLUGINLIB_EXPORT_CLASS(g1_hand_interface::G1Dex3System, hardware_interface::SystemInterface)
