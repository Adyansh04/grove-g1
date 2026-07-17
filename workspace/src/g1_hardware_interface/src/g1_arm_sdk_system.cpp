#include "g1_hardware_interface/g1_arm_sdk_system.hpp"

#include <atomic>
#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace g1_hardware_interface
{

namespace
{
constexpr char kLoggerName[] = "g1_arm_sdk_system";

// on_init runs once, off the RT path: std::sto* exceptions on a malformed
// <param> are caught here and turned into a logged FAILURE rather than an
// exception escaping a pluginlib-loaded on_init.
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
}  // namespace

G1ArmSdkSystem::~G1ArmSdkSystem() { shutdownInternalNode(); }

hardware_interface::CallbackReturn
G1ArmSdkSystem::on_init(const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints.size() != kNumArmJoints)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLoggerName),
            "expected exactly %zu arm joints, URDF declares %zu",
            kNumArmJoints,
            info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        const auto& joint = info_.joints[i];

        if (joint.command_interfaces.size() != 1 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(kLoggerName),
                "joint '%s' must export exactly one position command interface",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        bool has_position = false;
        bool has_velocity = false;
        bool has_effort   = false;
        for (const auto& state_interface : joint.state_interfaces)
        {
            has_position |= state_interface.name == hardware_interface::HW_IF_POSITION;
            has_velocity |= state_interface.name == hardware_interface::HW_IF_VELOCITY;
            has_effort |= state_interface.name == hardware_interface::HW_IF_EFFORT;
        }
        if (joint.state_interfaces.size() != 3 || !has_position || !has_velocity || !has_effort)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(kLoggerName),
                "joint '%s' must export exactly position/velocity/effort state interfaces",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        joint_names_[i] = joint.name;
        if (!parseInt(joint.parameters, "motor_index", motor_index_[i]) ||
            !parseDouble(joint.parameters, "kp", kp_[i]) ||
            !parseDouble(joint.parameters, "kd", kd_[i]))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(kLoggerName),
                "joint '%s' is missing or has an unparseable motor_index/kp/kd param",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    const std::string index_error = validateMotorIndexMap(motor_index_);
    if (!index_error.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "%s", index_error.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto& hw_params           = info_.hardware_parameters;
    double      lowstate_timeout_ms = 0.0;
    const bool  params_ok =
        parseDouble(hw_params, "command_publish_rate", command_publish_rate_hz_) &&
        parseDouble(hw_params, "blend_ramp_up_s", blend_ramp_up_s_) &&
        parseDouble(hw_params, "blend_ramp_down_s", blend_ramp_down_s_) &&
        parseDouble(hw_params, "emergency_ramp_down_s", emergency_ramp_down_s_) &&
        parseDouble(hw_params, "max_joint_velocity_rad_s", max_joint_velocity_rad_s_) &&
        parseDouble(hw_params, "lowstate_timeout_ms", lowstate_timeout_ms);
    if (!params_ok)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLoggerName),
            "<hardware> is missing or has an unparseable <param>");
        return hardware_interface::CallbackReturn::ERROR;
    }
    lowstate_timeout_s_ = lowstate_timeout_ms / 1000.0;

    if (command_publish_rate_hz_ <= 0.0 || blend_ramp_up_s_ <= 0.0 || blend_ramp_down_s_ <= 0.0 ||
        emergency_ramp_down_s_ <= 0.0 || max_joint_velocity_rad_s_ <= 0.0 ||
        lowstate_timeout_s_ <= 0.0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger(kLoggerName),
            "all system <param> tunables must be strictly positive");
        return hardware_interface::CallbackReturn::ERROR;
    }

    ramp_engine_ = ArmRampEngine(RampConfig{ blend_ramp_up_s_,
                                             blend_ramp_down_s_,
                                             emergency_ramp_down_s_,
                                             max_joint_velocity_rad_s_ });

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_configure(const rclcpp_lifecycle::State&)
{
    // Idempotent: on_configure can run more than once per process (e.g. an
    // error-triggered reset to UNCONFIGURED followed by a fresh
    // configure+activate). Tearing down any still-running executor/thread
    // from a previous configure *before* replacing the members is required
    // -- overwriting a live executor_ shared_ptr while its thread is still
    // inside spin() destroys the executor out from under it (observed
    // directly as a segfault during manual sim validation).
    shutdownInternalNode();

    node_ = std::make_shared<rclcpp::Node>(makeInternalNodeName());

    // Best-effort: compatible with either a best-effort or reliable
    // publisher (the sim's /lowstate happens to be RELIABLE; real hardware
    // may differ) and only the newest sample ever matters at ~500 Hz.
    const auto lowstate_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    lowstate_sub_           = node_->create_subscription<unitree_hg::msg::LowState>(
        "/lowstate",
        lowstate_qos,
        [this](const unitree_hg::msg::LowState::SharedPtr msg) { lowstateCallback(msg); });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this] { executor_->spin(); });

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_cleanup(const rclcpp_lifecycle::State&)
{
    shutdownInternalNode();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_activate(const rclcpp_lifecycle::State&)
{
    // Freshness-gated hold-in-place seeding and the weight ramp-up land with
    // the arm_sdk command path commit.
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_error(const rclcpp_lifecycle::State&)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> G1ArmSdkSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(kNumArmJoints * 3);
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_POSITION,
            &state_position_[i]);
        interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_VELOCITY,
            &state_velocity_[i]);
        interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &state_effort_[i]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> G1ArmSdkSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(kNumArmJoints);
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_POSITION,
            &command_position_[i]);
    }
    return interfaces;
}

hardware_interface::return_type
G1ArmSdkSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // readFromRT never blocks (try_lock internally): worst case this tick
    // sees the previous sample again, never a torn one.
    const StampedLowState* sample = lowstate_buffer_.readFromRT();
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        const auto& motor  = sample->state.motor_state[static_cast<std::size_t>(motor_index_[i])];
        state_position_[i] = motor.q;
        state_velocity_[i] = motor.dq;
        state_effort_[i]   = motor.tau_est;
    }

    // Staleness isn't checked yet: CM calls read() on every loaded component
    // every cycle regardless of lifecycle state (confirmed during manual sim
    // validation), so immediately after on_configure the internal node
    // hasn't had time for DDS discovery + a first /lowstate sample -- an
    // unconditional ERROR return here was tried and is demonstrably wrong:
    // resource_manager reacts to a read() ERROR by driving the component
    // through on_error back to UNCONFIGURED with no automatic recovery, so
    // it fired on essentially every configure. The freshness check belongs
    // here only once "meaningfully active" is a real, checkable concept
    // (the mode_ atomic lands with the arm_sdk command path commit).
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
G1ArmSdkSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // arm_sdk command path lands with the ramp/slew wiring commit.
    return hardware_interface::return_type::OK;
}

std::string G1ArmSdkSystem::makeInternalNodeName()
{
    // Suffixed so multiple instances in one process (e.g. a test harness)
    // never collide on the node name; controller_manager itself only ever
    // loads one.
    static std::atomic<std::uint64_t> counter{ 0 };
    return "g1_arm_sdk_system_internal_" + std::to_string(counter.fetch_add(1));
}

void G1ArmSdkSystem::lowstateCallback(const unitree_hg::msg::LowState::SharedPtr msg)
{
    StampedLowState sample;
    sample.state   = *msg;
    sample.arrival = std::chrono::steady_clock::now();
    lowstate_buffer_.writeFromNonRT(sample);
}

void G1ArmSdkSystem::shutdownInternalNode()
{
    if (executor_)
    {
        executor_->cancel();
    }
    if (executor_thread_.joinable())
    {
        executor_thread_.join();
    }
    lowstate_sub_.reset();
    if (executor_ && node_)
    {
        executor_->remove_node(node_);
    }
    executor_.reset();
    node_.reset();
}

}  // namespace g1_hardware_interface

PLUGINLIB_EXPORT_CLASS(g1_hardware_interface::G1ArmSdkSystem, hardware_interface::SystemInterface)
