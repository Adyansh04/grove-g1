/**
 * @file g1_lowcmd_system.cpp
 * @brief ros2_control SystemInterface owning the G1's 29 body motors over rt/lowcmd.
 */

#include "g1_hardware_interface/g1_lowcmd_system.hpp"

#include <algorithm>
#include <bit>
#include <functional>
#include <pluginlib/class_list_macros.hpp>
#include <string_view>
#include <thread>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace g1_hardware_interface
{

const std::array<std::string, kNumBodyMotors> kG1JointNames = {
    "left_hip_pitch_joint",      "left_hip_roll_joint",        "left_hip_yaw_joint",
    "left_knee_joint",           "left_ankle_pitch_joint",     "left_ankle_roll_joint",
    "right_hip_pitch_joint",     "right_hip_roll_joint",       "right_hip_yaw_joint",
    "right_knee_joint",          "right_ankle_pitch_joint",    "right_ankle_roll_joint",
    "waist_yaw_joint",           "waist_roll_joint",           "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",   "left_shoulder_yaw_joint",
    "left_elbow_joint",          "left_wrist_roll_joint",      "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",      "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",  "right_elbow_joint",          "right_wrist_roll_joint",
    "right_wrist_pitch_joint",   "right_wrist_yaw_joint"
};

namespace
{
/// mode_pr selects the ankle parameterisation; 0 is pitch/roll, which is what our URDF models.
constexpr std::uint8_t kModePr = 0;

/// Retry budget for handing control over from the onboard service. The upstream numbers.
constexpr int                       kMotionSwitchAttempts = 5;
constexpr float                     kMotionSwitchTimeoutS = 5.0F;
constexpr std::chrono::seconds      kMotionSwitchBackoff{ 2 };
constexpr std::chrono::milliseconds kReleaseTickPeriod{ 5 };
constexpr std::chrono::seconds      kFirstStateTimeout{ 10 };

/// Clears the flag on every exit from write(), including the error return. Pointer rather than
/// reference so the type stays assignable, which clang-tidy requires of a data member.
struct InWriteGuard
{
    std::atomic<bool>* flag;
    ~InWriteGuard() { flag->store(false, std::memory_order_release); }
};

bool parseDouble(
    const std::unordered_map<std::string, std::string>& params, const std::string& key, double& out)
{
    const auto it = params.find(key);
    if (it == params.end())
    {
        return false;
    }
    try
    {
        out = std::stod(it->second);
    }
    catch (const std::exception&)
    {
        return false;
    }
    return true;
}

bool parseInt(
    const std::unordered_map<std::string, std::string>& params, const std::string& key, int& out)
{
    const auto it = params.find(key);
    if (it == params.end())
    {
        return false;
    }
    try
    {
        out = std::stoi(it->second);
    }
    catch (const std::exception&)
    {
        return false;
    }
    return true;
}

bool parseBool(
    const std::unordered_map<std::string, std::string>& params, const std::string& key, bool& out)
{
    const auto it = params.find(key);
    if (it == params.end())
    {
        return false;
    }
    out = it->second == "true" || it->second == "True" || it->second == "1";
    return true;
}

/// Splits "joint_name/interface_type" as ros2_control hands it to a mode switch.
std::pair<std::string, std::string> splitInterface(const std::string& interface)
{
    const auto pos = interface.rfind('/');
    if (pos == std::string::npos)
    {
        return { "", "" };
    }
    return { interface.substr(0, pos), interface.substr(pos + 1) };
}
}  // namespace

G1LowCmdSystem::~G1LowCmdSystem() { shutdownSdk(); }

hardware_interface::CallbackReturn
G1LowCmdSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto& info = get_hardware_info();
    executor_        = params.executor;

    const auto& hw = info.hardware_parameters;

    // Deliberately empty by default: a non-empty interface makes the SDK build its own inline
    // CycloneDDS config and discard CYCLONEDDS_URI, which is what pins us to loopback.
    if (const auto it = hw.find("network_interface"); it != hw.end())
    {
        network_interface_ = it->second;
    }
    if (!parseInt(hw, "domain_id", domain_id_))
    {
        RCLCPP_ERROR(logger_, "<hardware> needs a domain_id param");
        return hardware_interface::CallbackReturn::ERROR;
    }
    parseInt(hw, "motor_temp_warn_threshold", motor_temp_warn_threshold_);
    parseBool(hw, "release_motion_mode", release_motion_mode_);

    double lowstate_timeout_ms = 0.0;
    if (!parseDouble(hw, "lowstate_timeout_ms", lowstate_timeout_ms) ||
        !parseDouble(hw, "release_ramp_s", release_ramp_s_) ||
        !parseDouble(hw, "release_kd", release_kd_))
    {
        RCLCPP_ERROR(logger_, "<hardware> is missing or has an unparseable <param>");
        return hardware_interface::CallbackReturn::ERROR;
    }
    lowstate_timeout_s_ = lowstate_timeout_ms / 1000.0;

    if (lowstate_timeout_s_ <= 0.0 || release_ramp_s_ <= 0.0 || release_kd_ <= 0.0)
    {
        RCLCPP_ERROR(logger_, "lowstate_timeout_ms, release_ramp_s and release_kd must be > 0");
        return hardware_interface::CallbackReturn::ERROR;
    }

    registerJoints(info);
    registerImuSensor(info);
    buildJointSdkMapping();

    for (const auto& jd : joint_data_)
    {
        if (jd.sdk_index < 0)
        {
            RCLCPP_ERROR(logger_, "joint '%s' is not a G1 body motor", jd.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        // These default to zero, so a missing or unparseable <param> would put the joint on the
        // position-only branch enabled but with no stiffness: owned by a controller and limp.
        if (jd.position_only_gains.kp <= 0.0 || jd.position_only_gains.kd <= 0.0)
        {
            RCLCPP_ERROR(
                logger_,
                "joint '%s' needs position_only_kp and position_only_kd params > 0",
                jd.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // The checksum covers this struct's padding, so zero all 1004 bytes once here and only ever
    // write named fields afterwards. bit_cast reaches the padding that value-init leaves alone.
    low_cmd_ = std::bit_cast<unitree_hg::msg::dds_::LowCmd_>(
        std::array<std::uint32_t, sizeof(unitree_hg::msg::dds_::LowCmd_) / 4>{});

    node_ = std::make_shared<rclcpp::Node>(
        "g1_lowcmd_system_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
    if (auto executor = executor_.lock())
    {
        executor->add_node(node_->get_node_base_interface());
    }

    RCLCPP_INFO(logger_, "G1LowCmdSystem holds %zu joints on rt/lowcmd", joint_data_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
}

void G1LowCmdSystem::registerJoints(const hardware_interface::HardwareInfo& info)
{
    joint_data_.clear();
    joint_data_.reserve(info.joints.size());
    joint_name_to_index_.clear();

    for (const auto& joint : info.joints)
    {
        JointData jd;
        jd.name = joint.name;
        parseDouble(joint.parameters, "position_only_kp", jd.position_only_gains.kp);
        parseDouble(joint.parameters, "position_only_kd", jd.position_only_gains.kd);

        joint_name_to_index_[joint.name] = joint_data_.size();
        joint_data_.push_back(std::move(jd));
    }
}

void G1LowCmdSystem::registerImuSensor(const hardware_interface::HardwareInfo& info)
{
    for (const auto& sensor : info.sensors)
    {
        const bool has_orientation = std::any_of(
            sensor.state_interfaces.begin(),
            sensor.state_interfaces.end(),
            [](const auto& iface) { return iface.name.find("orientation") != std::string::npos; });
        if (has_orientation)
        {
            imu_data_.name = sensor.name;
            has_imu_       = true;
            RCLCPP_INFO(logger_, "IMU sensor '%s' registered", sensor.name.c_str());
            return;
        }
    }
}

void G1LowCmdSystem::buildJointSdkMapping()
{
    for (auto& jd : joint_data_)
    {
        for (std::size_t i = 0; i < kG1JointNames.size(); ++i)
        {
            if (jd.name == kG1JointNames[i])
            {
                jd.sdk_index = static_cast<int>(i);
                break;
            }
        }
    }
}

std::vector<hardware_interface::StateInterface> G1LowCmdSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(joint_data_.size() * 3 + (has_imu_ ? 10 : 0));

    for (auto& jd : joint_data_)
    {
        interfaces.emplace_back(jd.name, hardware_interface::HW_IF_POSITION, &jd.position_state);
        interfaces.emplace_back(jd.name, hardware_interface::HW_IF_VELOCITY, &jd.velocity_state);
        interfaces.emplace_back(jd.name, hardware_interface::HW_IF_EFFORT, &jd.effort_state);
    }

    if (has_imu_)
    {
        interfaces.emplace_back(imu_data_.name, "orientation.x", &imu_data_.orientation_x);
        interfaces.emplace_back(imu_data_.name, "orientation.y", &imu_data_.orientation_y);
        interfaces.emplace_back(imu_data_.name, "orientation.z", &imu_data_.orientation_z);
        interfaces.emplace_back(imu_data_.name, "orientation.w", &imu_data_.orientation_w);
        interfaces.emplace_back(imu_data_.name, "angular_velocity.x", &imu_data_.angular_velocity_x);
        interfaces.emplace_back(imu_data_.name, "angular_velocity.y", &imu_data_.angular_velocity_y);
        interfaces.emplace_back(imu_data_.name, "angular_velocity.z", &imu_data_.angular_velocity_z);
        interfaces.emplace_back(
            imu_data_.name,
            "linear_acceleration.x",
            &imu_data_.linear_acceleration_x);
        interfaces.emplace_back(
            imu_data_.name,
            "linear_acceleration.y",
            &imu_data_.linear_acceleration_y);
        interfaces.emplace_back(
            imu_data_.name,
            "linear_acceleration.z",
            &imu_data_.linear_acceleration_z);
    }

    return interfaces;
}

std::vector<hardware_interface::CommandInterface> G1LowCmdSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(joint_data_.size() * 5);

    for (auto& jd : joint_data_)
    {
        interfaces.emplace_back(jd.name, hardware_interface::HW_IF_POSITION, &jd.command.position);
        interfaces.emplace_back(jd.name, hardware_interface::HW_IF_VELOCITY, &jd.command.velocity);
        interfaces.emplace_back(jd.name, hardware_interface::HW_IF_EFFORT, &jd.command.effort);
        interfaces.emplace_back(jd.name, std::string(kHwIfKp), &jd.command.kp);
        interfaces.emplace_back(jd.name, std::string(kHwIfKd), &jd.command.kd);
    }

    return interfaces;
}

bool G1LowCmdSystem::releaseOnboardMotionMode()
{
    unitree::robot::b2::MotionSwitcherClient switcher;
    switcher.SetTimeout(kMotionSwitchTimeoutS);
    switcher.Init();

    std::string form;
    std::string name;
    for (int attempt = 0; attempt < kMotionSwitchAttempts; ++attempt)
    {
        switcher.CheckMode(form, name);
        if (name.empty())
        {
            RCLCPP_INFO(logger_, "no onboard motion mode holds the motors, rt/lowcmd is ours");
            return true;
        }
        RCLCPP_WARN(logger_, "onboard mode '%s' still active, releasing", name.c_str());
        switcher.ReleaseMode();
        std::this_thread::sleep_for(kMotionSwitchBackoff);
    }

    RCLCPP_ERROR(
        logger_,
        "onboard mode '%s' survived %d release attempts -- refusing to fight it for the motors",
        name.c_str(),
        kMotionSwitchAttempts);
    return false;
}

bool G1LowCmdSystem::initializeSdk()
{
    try
    {
        unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_interface_);

        if (release_motion_mode_ && !releaseOnboardMotionMode())
        {
            return false;
        }

        lowstate_subscriber_ =
            std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
                "rt/lowstate");
        lowstate_subscriber_->InitChannel(
            [this](const void* message) { lowStateCallback(message); },
            1);

        lowcmd_publisher_ =
            std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(
                "rt/lowcmd");
        lowcmd_publisher_->InitChannel();

        const auto deadline = std::chrono::steady_clock::now() + kFirstStateTimeout;
        while (!first_state_received_.load())
        {
            if (std::chrono::steady_clock::now() > deadline)
            {
                RCLCPP_ERROR(logger_, "no rt/lowstate within 10 s -- is the robot or sim up?");
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
/// sdk_initialized_ still false, and those have to go too: the subscriber's handler captures
/// this, so an outliving channel writes into a destroyed component.
void G1LowCmdSystem::shutdownSdk()
{
    lowstate_subscriber_.reset();
    lowcmd_publisher_.reset();
    // first_state_received_ is cleared so a re-activation waits for a frame on the new
    // subscriber rather than seeding from whatever the buffer holds from the previous session.
    sdk_initialized_      = false;
    first_state_received_ = false;
}

void G1LowCmdSystem::lowStateCallback(const void* message)
{
    StampedLowState sample;
    sample.state   = *static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
    sample.arrival = std::chrono::steady_clock::now();
    lowstate_buffer_.writeFromNonRT(sample);
    first_state_received_ = true;
}

hardware_interface::CallbackReturn
G1LowCmdSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (!initializeSdk())
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const StampedLowState* sample = lowstate_buffer_.readFromRT();
    mode_machine_                 = sample->state.mode_machine();

    // Seed at the measurement with zero gains: a controller that claims a joint must supply its
    // own stiffness before anything moves.
    for (auto& jd : joint_data_)
    {
        const auto& motor = sample->state.motor_state()[static_cast<std::size_t>(jd.sdk_index)];
        jd.position_state = motor.q();
        jd.velocity_state = motor.dq();
        jd.effort_state   = motor.tau_est();
        jd.command        = JointCommand{ motor.q(), 0.0, 0.0, 0.0, 0.0 };
        jd.mode           = JointControlMode::kDisabled;
        jd.claims         = InterfaceClaims{};
    }

    diagnostics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics",
        rclcpp::SystemDefaultsQoS());
    diagnostics_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), [this] {
        diagnostic_msgs::msg::DiagnosticArray msg;
        msg.header.stamp = node_->now();
        for (const auto& jd : joint_data_)
        {
            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name        = jd.name;
            status.hardware_id = "unitree_g1";
            status.level       = jd.winding_temperature >= motor_temp_warn_threshold_ ?
                                     diagnostic_msgs::msg::DiagnosticStatus::WARN :
                                     diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "surface " + std::to_string(jd.surface_temperature) + " C, winding " +
                             std::to_string(jd.winding_temperature) + " C";

            // The upstream key names, so a consumer written against that stack reads ours.
            diagnostic_msgs::msg::KeyValue surface;
            surface.key   = "surface_temperature_C";
            surface.value = std::to_string(jd.surface_temperature);
            diagnostic_msgs::msg::KeyValue winding;
            winding.key   = "winding_temperature_C";
            winding.value = std::to_string(jd.winding_temperature);
            status.values = { surface, winding };

            msg.status.push_back(status);
        }
        diagnostics_pub_->publish(msg);
    });

    active_ = true;
    RCLCPP_WARN(logger_, "rt/lowcmd is now ours: no onboard balance runs underneath this component");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
G1LowCmdSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    releaseSynchronously();
    diagnostics_timer_.reset();
    diagnostics_pub_.reset();
    shutdownSdk();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
G1LowCmdSystem::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
    releaseSynchronously();
    shutdownSdk();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
G1LowCmdSystem::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
    // Idempotent: releaseSynchronously() returns immediately if a deactivate already ran.
    releaseSynchronously();
    shutdownSdk();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type G1LowCmdSystem::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    // kp and kd move as a pair: one alone does not define an impedance, so the flag only changes
    // when both appear on the same side of the switch.
    const auto listed =
        [](const std::vector<std::string>& list, const std::string& joint, std::string_view type) {
            const std::string full = joint + "/" + std::string(type);
            return std::find(list.begin(), list.end(), full) != list.end();
        };

    const auto apply = [&](const std::vector<std::string>& list, bool held) {
        for (const auto& interface : list)
        {
            const auto [joint_name, type] = splitInterface(interface);
            const auto it                 = joint_name_to_index_.find(joint_name);
            if (it == joint_name_to_index_.end())
            {
                continue;
            }
            auto& claims = joint_data_[it->second].claims;

            if (type == hardware_interface::HW_IF_POSITION)
            {
                claims.position = held;
                if (held)
                {
                    // Take over from where the joint is, so claiming it does not step the setpoint.
                    joint_data_[it->second].command.position =
                        joint_data_[it->second].position_state;
                }
            }
            else if (type == hardware_interface::HW_IF_VELOCITY)
            {
                claims.velocity = held;
            }
            else if (type == hardware_interface::HW_IF_EFFORT)
            {
                claims.effort = held;
            }
            else if (
                (type == kHwIfKp || type == kHwIfKd) && listed(list, joint_name, kHwIfKp) &&
                listed(list, joint_name, kHwIfKd))
            {
                claims.impedance = held;
            }
        }
    };

    apply(stop_interfaces, false);
    apply(start_interfaces, true);

    for (auto& jd : joint_data_)
    {
        jd.mode = resolveJointMode(jd.claims);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
G1LowCmdSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    const StampedLowState* sample = lowstate_buffer_.readFromRT();

    for (auto& jd : joint_data_)
    {
        const auto& motor = sample->state.motor_state()[static_cast<std::size_t>(jd.sdk_index)];
        jd.position_state = motor.q();
        jd.velocity_state = motor.dq();
        jd.effort_state   = motor.tau_est();
        jd.surface_temperature = motor.temperature()[0];
        jd.winding_temperature = motor.temperature()[1];
    }

    if (has_imu_)
    {
        const auto& imu = sample->state.imu_state();
        // Unitree order the quaternion w, x, y, z.
        imu_data_.orientation_w = imu.quaternion()[0];
        imu_data_.orientation_x = imu.quaternion()[1];
        imu_data_.orientation_y = imu.quaternion()[2];
        imu_data_.orientation_z = imu.quaternion()[3];

        imu_data_.angular_velocity_x = imu.gyroscope()[0];
        imu_data_.angular_velocity_y = imu.gyroscope()[1];
        imu_data_.angular_velocity_z = imu.gyroscope()[2];

        imu_data_.linear_acceleration_x = imu.accelerometer()[0];
        imu_data_.linear_acceleration_y = imu.accelerometer()[1];
        imu_data_.linear_acceleration_z = imu.accelerometer()[2];
    }

    mode_machine_ = sample->state.mode_machine();

    const auto age = std::chrono::steady_clock::now() - sample->arrival;
    if (active_.load() && age > std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(lowstate_timeout_s_)))
    {
        RCLCPP_ERROR(logger_, "rt/lowstate went stale while active");
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

bool G1LowCmdSystem::publishLowCmd()
{
    low_cmd_.mode_pr()      = kModePr;
    low_cmd_.mode_machine() = mode_machine_;
    computeLowCmdCrc(low_cmd_);
    return lowcmd_publisher_->Write(low_cmd_);
}

hardware_interface::return_type
G1LowCmdSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!active_.load() || !sdk_initialized_.load())
    {
        return hardware_interface::return_type::OK;
    }

    // Announced for the whole body, so the release ramp cannot start filling low_cmd_ underneath
    // this tick. Two relaxed-ish atomic stores, no allocation and no lock on the 200 Hz path.
    in_write_.store(true, std::memory_order_release);
    const InWriteGuard guard{ &in_write_ };

    for (const auto& jd : joint_data_)
    {
        fillMotorCmd(
            low_cmd_.motor_cmd()[static_cast<std::size_t>(jd.sdk_index)],
            jd.mode,
            jd.command,
            jd.position_only_gains,
            jd.position_state);
    }

    if (!publishLowCmd())
    {
        RCLCPP_ERROR(logger_, "rt/lowcmd write refused -- the robot is no longer being commanded");
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

void G1LowCmdSystem::releaseSynchronously()
{
    if (!active_.exchange(false) || !sdk_initialized_.load())
    {
        return;
    }

    // active_ is now false, so no further write() can enter; wait out the one that may already
    // be inside. Without this the ramp's first frame can interleave with a controller command
    // mid-CRC, and shutdownSdk() can reset the publisher while write() is dereferencing it.
    while (in_write_.load(std::memory_order_acquire))
    {
        std::this_thread::yield();
    }

    for (auto& jd : joint_data_)
    {
        jd.release_hold_position = jd.position_state;
        jd.release_kp =
            jd.mode == JointControlMode::kImpedance ? jd.command.kp : jd.position_only_gains.kp;
    }

    // At least one tick: release_ramp_s_ is only validated positive, and a sub-tick value would
    // divide by zero below and put NaN gains on every motor.
    const int ticks = std::max(
        1,
        static_cast<int>(
            release_ramp_s_ / std::chrono::duration<double>(kReleaseTickPeriod).count()));
    for (int tick = 0; tick <= ticks; ++tick)
    {
        const double scale = 1.0 - static_cast<double>(tick) / static_cast<double>(ticks);
        for (const auto& jd : joint_data_)
        {
            fillReleaseCmd(
                low_cmd_.motor_cmd()[static_cast<std::size_t>(jd.sdk_index)],
                jd.release_hold_position,
                jd.release_kp,
                scale,
                release_kd_);
        }
        publishLowCmd();
        std::this_thread::sleep_for(kReleaseTickPeriod);
    }

    for (const auto& jd : joint_data_)
    {
        fillMotorCmd(
            low_cmd_.motor_cmd()[static_cast<std::size_t>(jd.sdk_index)],
            JointControlMode::kDisabled,
            jd.command,
            jd.position_only_gains,
            jd.position_state);
    }
    publishLowCmd();
}

}  // namespace g1_hardware_interface

PLUGINLIB_EXPORT_CLASS(g1_hardware_interface::G1LowCmdSystem, hardware_interface::SystemInterface)
