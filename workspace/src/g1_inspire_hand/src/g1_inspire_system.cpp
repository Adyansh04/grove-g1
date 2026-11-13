/**
 * @file g1_inspire_system.cpp
 * @brief One Inspire RH56DFQ hand as a ros2_control system, over its RS485 line.
 */

#include "g1_inspire_hand/g1_inspire_system.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <optional>
#include <string_view>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace g1_inspire_hand
{
namespace
{

/// Long enough for a hand that has just been powered to answer.
constexpr auto kProbeWait = std::chrono::milliseconds(2000);

/// Names for logs, in register order.
constexpr std::array<std::string_view, kNumActuators> kActuatorNames = {
    "pinky", "ring", "middle", "index", "thumb bend", "thumb rotation",
};

std::optional<double> toDouble(const std::string& text)
{
    try
    {
        std::size_t  used  = 0;
        const double value = std::stod(text, &used);
        return used == text.size() && std::isfinite(value) ? std::optional{ value } : std::nullopt;
    }
    catch (const std::exception&)
    {
        return std::nullopt;
    }
}

std::int64_t steadyNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::string errorFlags(std::uint8_t flags)
{
    std::string text;
    const auto  add = [&](ErrorBit bit, std::string_view name) {
        if ((flags & static_cast<std::uint8_t>(bit)) != 0)
        {
            text += text.empty() ? "" : ", ";
            text += name;
        }
    };
    add(ErrorBit::kLockedRotor, "locked rotor");
    add(ErrorBit::kOverTemp, "over temperature");
    add(ErrorBit::kOverCurrent, "over current");
    add(ErrorBit::kMotorAbnormal, "motor fault");
    add(ErrorBit::kComms, "communication fault");
    return text;
}

/// A hardware parameter, required, so a renamed one fails here rather than defaulting.
std::optional<std::string> paramText(
    const hardware_interface::HardwareInfo& info, const rclcpp::Logger& logger,
    const std::string& key)
{
    const auto it = info.hardware_parameters.find(key);
    if (it == info.hardware_parameters.end() || it->second.empty())
    {
        RCLCPP_FATAL(logger, "<hardware> needs a '%s' param", key.c_str());
        return std::nullopt;
    }
    return it->second;
}

std::optional<double> paramNumber(
    const hardware_interface::HardwareInfo& info, const rclcpp::Logger& logger,
    const std::string& key, double low, double high)
{
    const std::optional<std::string> raw   = paramText(info, logger, key);
    const std::optional<double>      value = raw ? toDouble(*raw) : std::nullopt;
    if (!value || *value < low || *value > high)
    {
        RCLCPP_FATAL(
            logger,
            "'%s' must be a number from %g to %g, not '%s'",
            key.c_str(),
            low,
            high,
            raw.value_or("").c_str());
        return std::nullopt;
    }
    return value;
}

}  // namespace

G1InspireSystem::~G1InspireSystem() { stopBus(); }

hardware_interface::CallbackReturn
G1InspireSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto& info   = get_hardware_info();
    const auto  text   = [&](const char* key) { return paramText(info, logger_, key); };
    const auto  number = [&](const char* key, double low, double high) {
        return paramNumber(info, logger_, key, low, high);
    };

    const std::optional<std::string> side = text("side");
    if (!side || (*side != "left" && *side != "right"))
    {
        RCLCPP_FATAL(logger_, "'side' must be exactly \"left\" or \"right\"");
        return hardware_interface::CallbackReturn::ERROR;
    }
    side_   = *side;
    logger_ = rclcpp::get_logger("g1_inspire_" + side_ + "_system");

    const std::optional<std::string> port     = text("serial_port");
    const std::optional<std::string> protocol = text("protocol");
    const std::optional<Framing>     framing = protocol ? framingFromName(*protocol) : std::nullopt;
    if (!port || !framing)
    {
        RCLCPP_FATAL(logger_, "'serial_port' must be set and 'protocol' be inspire or modbus_rtu");
        return hardware_interface::CallbackReturn::ERROR;
    }
    serial_port_ = *port;
    framing_     = *framing;

    const auto baud     = number("baud_rate", 9600, 921600);
    const auto id       = number("slave_id", 1, 247);
    const auto rate     = number("io_rate_hz", 1.0, 500.0);
    const auto reply    = number("response_timeout_ms", 1.0, 1000.0);
    const auto stale    = number("state_timeout_ms", 10.0, 10000.0);
    const auto diag     = number("diagnostics_every", 1, 10000);
    const auto speed    = number("speed", 0, kAngleOpen);
    const auto force    = number("force_limit_g", 0, 3000);
    const auto velocity = number("max_joint_velocity_rad_s", 1e-3, 100.0);
    if (!baud || !id || !rate || !reply || !stale || !diag || !speed || !force || !velocity)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    baud_rate_                = static_cast<int>(*baud);
    slave_id_                 = static_cast<std::uint8_t>(*id);
    io_rate_hz_               = *rate;
    reply_timeout_            = std::chrono::microseconds(std::lround(*reply * 1000.0));
    state_timeout_            = std::chrono::nanoseconds(std::llround(*stale * 1.0e6));
    diagnostics_every_        = static_cast<int>(*diag);
    speed_                    = static_cast<std::int16_t>(*speed);
    force_limit_g_            = static_cast<std::int16_t>(*force);
    max_joint_velocity_rad_s_ = *velocity;

    if (info.joints.size() != kNumActuators)
    {
        RCLCPP_FATAL(
            logger_,
            "expected %zu joints for the %s hand but got %zu",
            kNumActuators,
            side_.c_str(),
            info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    // The registers are positional, so a xacro in any other order would drive the wrong finger.
    const std::string prefix = side_ == "left" ? "L_" : "R_";
    for (std::size_t i = 0; i < kNumActuators; ++i)
    {
        const auto&       joint    = info.joints[i];
        const std::string expected = prefix + kActuatorJointSuffixes[i];
        if (joint.name != expected)
        {
            RCLCPP_FATAL(
                logger_,
                "joint %zu is '%s' but the register order needs '%s' there",
                i,
                joint.name.c_str(),
                expected.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        const auto read = [&](const char* key) -> std::optional<double> {
            const auto it = joint.parameters.find(key);
            return it == joint.parameters.end() ? std::nullopt : toDouble(it->second);
        };
        const std::optional<double> open   = read("open_rad");
        const std::optional<double> closed = read("closed_rad");
        const std::optional<double> lever  = read("lever_arm_m");
        if (!open || !closed || !lever || *open == *closed || *lever <= 0.0)
        {
            RCLCPP_FATAL(
                logger_,
                "joint '%s' needs numeric open_rad and closed_rad that differ, and lever_arm_m > 0",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        maps_[i] = ActuatorMap{ .open_rad = *open, .closed_rad = *closed, .lever_arm_m = *lever };
        lower_rad_[i] = std::min(*open, *closed);
        upper_rad_[i] = std::max(*open, *closed);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

ReplyError G1InspireSystem::readAnglesWithin(
    std::chrono::milliseconds wait, std::array<std::int16_t, kNumActuators>& out)
{
    const auto deadline = std::chrono::steady_clock::now() + wait;
    ReplyError error    = hand_->readWords(reg::kAngleAct, out);
    while (!error.empty() && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        error = hand_->readWords(reg::kAngleAct, out);
    }
    return error;
}

hardware_interface::CallbackReturn G1InspireSystem::on_activate(const rclcpp_lifecycle::State&)
{
    std::string error;
    stream_ = SerialPort::open(serial_port_, baud_rate_, error);
    if (!stream_)
    {
        RCLCPP_ERROR(logger_, "the %s hand: %s", side_.c_str(), error.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    hand_ = std::make_unique<InspireHand>(*stream_, framing_, slave_id_, reply_timeout_);

    std::array<std::int16_t, kNumActuators> angles{};
    const ReplyError                        probe = readAnglesWithin(kProbeWait, angles);
    if (!probe.empty())
    {
        RCLCPP_ERROR(
            logger_,
            "the %s hand on %s gave %.*s to a read in the %s framing: check its power, the A and B "
            "wires, serial_port and protocol",
            side_.c_str(),
            serial_port_.c_str(),
            static_cast<int>(probe.size()),
            probe.data(),
            framing_ == Framing::kInspire ? "inspire" : "modbus_rtu");
        stopBus();
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Volatile registers: the hand powers up at its own defaults, so these go out every time.
    std::array<std::int16_t, kNumActuators> speeds{};
    std::array<std::int16_t, kNumActuators> forces{};
    speeds.fill(speed_);
    forces.fill(force_limit_g_);
    const char* what   = "clearing its errors";
    ReplyError  result = hand_->clearErrors();
    if (result.empty())
    {
        what   = "setting its speed";
        result = hand_->writeWords(reg::kSpeedSet, speeds);
    }
    if (result.empty())
    {
        what   = "setting its force limit";
        result = hand_->writeWords(reg::kForceSet, forces);
    }
    if (!result.empty())
    {
        RCLCPP_ERROR(
            logger_,
            "the %s hand refused %s: %.*s",
            side_.c_str(),
            what,
            static_cast<int>(result.size()),
            result.data());
        stopBus();
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Seeded from the measurement, so the first command is a no-op rather than a jump. Nothing is
    // written until a command differs, which also leaves a hand closed on an object pressing.
    for (std::size_t i = 0; i < kNumActuators; ++i)
    {
        position_state_[i]   = jointFromAngle(angles[i], maps_[i]);
        velocity_state_[i]   = 0.0;
        effort_state_[i]     = 0.0;
        position_command_[i] = position_state_[i];
        ramped_command_[i]   = position_state_[i];
        target_angle_[i].store(angles[i], std::memory_order_relaxed);
        measured_angle_[i].store(angles[i], std::memory_order_relaxed);
        measured_force_g_[i].store(0, std::memory_order_relaxed);
    }
    last_seen_state_ns_ = steadyNs();
    state_stamp_ns_.store(last_seen_state_ns_, std::memory_order_release);
    active_.store(true, std::memory_order_release);
    bus_thread_ = std::jthread([this](const std::stop_token& stop) { runBus(stop); });
    RCLCPP_INFO(
        logger_,
        "the %s hand answers on %s; bus at %.0f Hz",
        side_.c_str(),
        serial_port_.c_str(),
        io_rate_hz_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1InspireSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
    stopBus();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1InspireSystem::on_error(const rclcpp_lifecycle::State&)
{
    stopBus();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1InspireSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
    stopBus();
    return hardware_interface::CallbackReturn::SUCCESS;
}

void G1InspireSystem::stopBus()
{
    active_.store(false, std::memory_order_release);
    if (bus_thread_.joinable())
    {
        bus_thread_.request_stop();
        bus_thread_.join();
    }
    hand_.reset();
    stream_.reset();
}

void G1InspireSystem::runBus(const std::stop_token& stop)
{
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / io_rate_hz_));
    std::array<std::int16_t, kNumActuators> sent{};
    for (std::size_t i = 0; i < kNumActuators; ++i)
    {
        sent[i] = target_angle_[i].load(std::memory_order_relaxed);
    }
    std::array<std::uint8_t, kNumActuators> last_errors{};
    bool                                    sent_ok  = true;
    int                                     cycle    = 0;
    int                                     failures = 0;
    auto                                    next     = std::chrono::steady_clock::now();

    while (!stop.stop_requested())
    {
        std::array<std::int16_t, kNumActuators> want{};
        for (std::size_t i = 0; i < kNumActuators; ++i)
        {
            want[i] = target_angle_[i].load(std::memory_order_relaxed);
        }
        if (!sent_ok || want != sent)
        {
            const ReplyError error = hand_->writeWords(reg::kAngleSet, want);
            sent_ok                = error.empty();
            if (sent_ok)
            {
                sent = want;
            }
        }

        std::array<std::int16_t, kNumActuators> angles{};
        std::array<std::int16_t, kNumActuators> forces{};
        ReplyError                              error = hand_->readWords(reg::kAngleAct, angles);
        if (error.empty())
        {
            error = hand_->readWords(reg::kForceAct, forces);
        }
        if (error.empty())
        {
            for (std::size_t i = 0; i < kNumActuators; ++i)
            {
                measured_angle_[i].store(angles[i], std::memory_order_relaxed);
                measured_force_g_[i].store(forces[i], std::memory_order_relaxed);
            }
            state_stamp_ns_.store(steadyNs(), std::memory_order_release);
            failures = 0;
        }
        else if (failures++ % 50 == 0)
        {
            RCLCPP_WARN(
                logger_,
                "the %s hand: %.*s (%d in a row)",
                side_.c_str(),
                static_cast<int>(error.size()),
                error.data(),
                failures);
        }

        if (cycle % diagnostics_every_ == 0)
        {
            checkDiagnostics(last_errors);
        }
        ++cycle;

        // Behind schedule, the next cycle starts now rather than in a catch-up burst.
        next = std::max(next + period, std::chrono::steady_clock::now());
        std::this_thread::sleep_until(next);
    }
}

void G1InspireSystem::checkDiagnostics(std::array<std::uint8_t, kNumActuators>& last_errors)
{
    std::array<std::uint8_t, kDiagnosticsBytes> block{};
    if (!hand_->read(reg::kCurrent, block).empty())
    {
        return;
    }
    constexpr std::size_t kErrorOffset  = reg::kError - reg::kCurrent;
    constexpr std::size_t kStatusOffset = reg::kStatus - reg::kCurrent;
    constexpr std::size_t kTempOffset   = reg::kTemp - reg::kCurrent;
    for (std::size_t i = 0; i < kNumActuators; ++i)
    {
        const std::uint8_t flags = block[kErrorOffset + i];
        if (flags == last_errors[i])
        {
            continue;
        }
        last_errors[i] = flags;
        if (flags == 0)
        {
            RCLCPP_INFO(
                logger_,
                "the %s hand's %s is clear of errors",
                side_.c_str(),
                kActuatorNames[i].data());
            continue;
        }
        // Latched until cleared: the actuator ignores targets meanwhile. Cleared at activation.
        RCLCPP_ERROR(
            logger_,
            "the %s hand's %s reports %s (status %u, %u deg C); it ignores targets until the hand "
            "is reactivated",
            side_.c_str(),
            kActuatorNames[i].data(),
            errorFlags(flags).c_str(),
            static_cast<unsigned>(block[kStatusOffset + i]),
            static_cast<unsigned>(block[kTempOffset + i]));
    }
}

std::vector<hardware_interface::StateInterface> G1InspireSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(kNumActuators * 3);
    for (std::size_t i = 0; i < kNumActuators; ++i)
    {
        const std::string& joint = info_.joints[i].name;
        interfaces.emplace_back(joint, hardware_interface::HW_IF_POSITION, &position_state_[i]);
        interfaces.emplace_back(joint, hardware_interface::HW_IF_VELOCITY, &velocity_state_[i]);
        interfaces.emplace_back(joint, hardware_interface::HW_IF_EFFORT, &effort_state_[i]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> G1InspireSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(kNumActuators);
    for (std::size_t i = 0; i < kNumActuators; ++i)
    {
        interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &position_command_[i]);
    }
    return interfaces;
}

hardware_interface::return_type G1InspireSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    if (!active_.load(std::memory_order_acquire))
    {
        return hardware_interface::return_type::OK;
    }
    const std::int64_t stamp = state_stamp_ns_.load(std::memory_order_acquire);
    if (steadyNs() - stamp > state_timeout_.count())
    {
        RCLCPP_ERROR(
            logger_,
            "no state from the %s hand for over %.0f ms",
            side_.c_str(),
            std::chrono::duration<double, std::milli>(state_timeout_).count());
        return hardware_interface::return_type::ERROR;
    }
    const bool   fresh = stamp != last_seen_state_ns_;
    const double dt_s  = static_cast<double>(stamp - last_seen_state_ns_) * 1.0e-9;
    for (std::size_t i = 0; i < kNumActuators; ++i)
    {
        const double position =
            jointFromAngle(measured_angle_[i].load(std::memory_order_relaxed), maps_[i]);
        if (fresh && dt_s > 0.0)
        {
            velocity_state_[i] = (position - position_state_[i]) / dt_s;
        }
        position_state_[i] = position;
        effort_state_[i] =
            effortFromForce(measured_force_g_[i].load(std::memory_order_relaxed), maps_[i]);
    }
    last_seen_state_ns_ = stamp;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
G1InspireSystem::write(const rclcpp::Time&, const rclcpp::Duration& period)
{
    if (!active_.load(std::memory_order_acquire))
    {
        return hardware_interface::return_type::OK;
    }
    // The slew bounds the commanded speed. period is clamped because a stalled loop would widen
    // the step and a negative one would transpose std::clamp's bounds.
    const double step = max_joint_velocity_rad_s_ * std::clamp(period.seconds(), 0.0, 0.1);
    for (std::size_t i = 0; i < kNumActuators; ++i)
    {
        // A non-finite target holds: std::clamp passes NaN through and the slew never recovers.
        if (!std::isfinite(position_command_[i]))
        {
            continue;
        }
        const double target = std::clamp(position_command_[i], lower_rad_[i], upper_rad_[i]);
        ramped_command_[i] += std::clamp(target - ramped_command_[i], -step, step);
        target_angle_[i].store(
            angleFromJoint(ramped_command_[i], maps_[i]),
            std::memory_order_relaxed);
    }
    return hardware_interface::return_type::OK;
}

}  // namespace g1_inspire_hand

PLUGINLIB_EXPORT_CLASS(g1_inspire_hand::G1InspireSystem, hardware_interface::SystemInterface)
