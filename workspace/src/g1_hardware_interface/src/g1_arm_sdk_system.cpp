#include "g1_hardware_interface/g1_arm_sdk_system.hpp"

#include <atomic>
#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <thread>
#include <unordered_map>

#include "g1_hardware_interface/motor_crc_hg.hpp"
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

// Fixed step for rampDownSynchronously()'s own loop -- matches the normal
// /arm_sdk publish cadence (command_publish_rate's default), but this path
// doesn't read that param: it runs during on_deactivate/on_error/
// on_shutdown/the advisory conflict guard, all off the RT path, where a
// simple fixed period is clearer than deriving one.
constexpr std::chrono::milliseconds kRampDownTickPeriod{ 10 };
}  // namespace

void assembleLowCmd(
    unitree_hg::msg::LowCmd& cmd, const std::array<int, kNumArmJoints>& motor_index,
    const std::array<double, kNumArmJoints>& position, const std::array<double, kNumArmJoints>& kp,
    const std::array<double, kNumArmJoints>& kd, float weight)
{
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        auto& motor = cmd.motor_cmd[static_cast<std::size_t>(motor_index[i])];
        motor.q     = static_cast<float>(position[i]);
        motor.dq    = 0.0F;
        motor.tau   = 0.0F;
        motor.kp    = static_cast<float>(kp[i]);
        motor.kd    = static_cast<float>(kd[i]);
    }
    cmd.motor_cmd[kWeightMotorIndex].q = weight;
}

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

    // Reliable: we're the sole authority on this channel (single writer by
    // construction -- see the README), so the DDS layer should retry rather
    // than silently drop a command; keep-last(1) because only the newest
    // ramp/slew tick ever matters.
    const auto arm_sdk_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    auto arm_sdk_pub = node_->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", arm_sdk_qos);
    arm_sdk_rt_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<unitree_hg::msg::LowCmd>>(arm_sdk_pub);

    publisher_count_timer_ =
        node_->create_wall_timer(std::chrono::seconds(1), [this] { checkPublisherCount(); });

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
    // A previous activation's conflict guard may still be winding down.
    if (conflict_ramp_thread_.joinable())
    {
        conflict_ramp_thread_.join();
    }

    // on_activate runs on the CM's lifecycle (non-RT) thread, never
    // read()/write()'s RT thread, so the non-RT accessor is the correct one
    // here (readFromRT() is reserved for the RT side).
    const StampedLowState* sample = lowstate_buffer_.readFromNonRT();
    if (isStale(sample->arrival, lowstateTimeoutDuration()))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "refusing to activate: /lowstate is older than lowstate_timeout_ms -- nothing "
            "published");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Hold-in-place seed: the exported command interface starts at exactly
    // the measured position (no unimplementable "first command arrived"
    // detection, no slew toward some other q) -- JTC's own activation covers
    // the controller side.
    std::array<double, kNumArmJoints> measured{};
    for (std::size_t i = 0; i < kNumArmJoints; ++i)
    {
        measured[i] = sample->state.motor_state[static_cast<std::size_t>(motor_index_[i])].q;
        command_position_[i] = measured[i];
    }
    ramp_engine_.seedFromMeasured(measured);

    time_since_last_publish_s_ = 0.0;
    writer_token_claimed_.store(false, std::memory_order_release);
    // Publishing authority is acquired last, only once everything it needs
    // (seed, cleared token) is already in place.
    mode_.store(BlendMode::kActive, std::memory_order_release);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
    rampDownSynchronously(BlendMode::kRampDown);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
    // Belt-and-braces: confirmed directly (manual sim validation) that
    // Humble's controller_manager already runs on_deactivate before
    // on_shutdown on a SIGTERM/SIGINT while active, so by the time this runs
    // mode_ is normally already kInactive and rampDownSynchronously() below
    // is a no-op. Kept in case some kill path reaches shutdown without
    // deactivate.
    rampDownSynchronously(BlendMode::kEmergencyRampDown);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_error(const rclcpp_lifecycle::State&)
{
    rampDownSynchronously(BlendMode::kEmergencyRampDown);
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

    // Belt-and-braces only: write() is the primary protection and
    // autonomously ramps down on stale feedback regardless of what read()
    // reports here (see write()). Gated on ACTIVE specifically -- CM calls
    // read() on every loaded component every cycle regardless of lifecycle
    // state (confirmed during manual sim validation), and staleness before
    // the first /lowstate sample arrives (e.g. right after on_configure) is
    // expected, not an error; an unconditional ERROR return here was tried
    // and is demonstrably wrong (resource_manager reacts to any read()
    // ERROR by driving the component through on_error back to UNCONFIGURED
    // with no automatic recovery, so it fired on essentially every
    // configure).
    if (mode_.load(std::memory_order_relaxed) == BlendMode::kActive &&
        isStale(sample->arrival, lowstateTimeoutDuration()))
    {
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
G1ArmSdkSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    if (writer_token_claimed_.load(std::memory_order_acquire))
    {
        // Ownership has been claimed by a lifecycle/advisory transition
        // currently running rampDownSynchronously() -- never touch the
        // publisher again this activation. In practice this codepath is
        // defense-in-depth: resource_manager serializes those transitions
        // against write() anyway (see rampDownSynchronously()'s comment),
        // so write() shouldn't be running concurrently with one at all.
        return hardware_interface::return_type::OK;
    }

    const BlendMode requested = mode_.load(std::memory_order_acquire);
    if (requested == BlendMode::kInactive)
    {
        // Self-gated: Humble may still call write() while inactive.
        return hardware_interface::return_type::OK;
    }

    const StampedLowState* sample    = lowstate_buffer_.readFromRT();
    const bool             stale     = isStale(sample->arrival, lowstateTimeoutDuration());
    const BlendMode        effective = resolveEffectiveMode(requested, stale);
    if (effective != requested)
    {
        // Autonomous emergency escalation: write() doesn't wait for
        // resource_manager to notice read()'s ERROR and call on_error --
        // it protects the arm itself, the instant it notices stale
        // feedback, and publishes the escalation back to the shared atomic
        // so any lifecycle thread watching it sees the true state too.
        mode_.store(effective, std::memory_order_release);
    }

    const double dt     = period.seconds();
    const double weight = ramp_engine_.step(effective, command_position_, dt);

    // Throttle to command_publish_rate_hz_ (independent of the CM's own
    // update_rate) using the slower-hardware-comms period-guard pattern:
    // ramp/slew state still advances every tick above (correct at any CM
    // rate), only the DDS publish itself is throttled.
    time_since_last_publish_s_ += dt;
    const double publish_period_s = 1.0 / command_publish_rate_hz_;
    const bool   due              = time_since_last_publish_s_ >= publish_period_s;
    // Force a publish on the exact tick the ramp reaches its target so the
    // true terminal weight (0) is always actually transmitted at least
    // once before self-gating off below -- otherwise the throttle could
    // silently swallow that last, safety-relevant sample.
    const bool ramp_finished = effective != BlendMode::kActive && weight <= 0.0;

    if (due || ramp_finished)
    {
        if (due)
        {
            time_since_last_publish_s_ -= publish_period_s;
        }
        if (arm_sdk_rt_pub_ && arm_sdk_rt_pub_->trylock())
        {
            assembleLowCmd(
                arm_sdk_rt_pub_->msg_,
                motor_index_,
                ramp_engine_.publishedPositions(),
                kp_,
                kd_,
                static_cast<float>(weight));
            vendored::computeLowCmdCrc(arm_sdk_rt_pub_->msg_);
            arm_sdk_rt_pub_->unlockAndPublish();
        }
    }

    if (ramp_finished)
    {
        // Reached here only via the autonomous emergency-escalation path
        // above (a lifecycle-triggered ramp-down is driven and finished
        // entirely by rampDownSynchronously() instead, which write() never
        // runs concurrently with): write() noticed the staleness itself,
        // ramped itself down over these ticks, and now self-terminates --
        // from the next tick on, self-gating above stops publishing
        // entirely.
        mode_.store(BlendMode::kInactive, std::memory_order_release);
    }

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
    // Any in-flight advisory ramp-down touches arm_sdk_rt_pub_ off this same
    // internal-executor-spawned thread -- it must finish before the members
    // below are reset out from under it.
    if (conflict_ramp_thread_.joinable())
    {
        conflict_ramp_thread_.join();
    }
    if (executor_)
    {
        executor_->cancel();
    }
    if (executor_thread_.joinable())
    {
        executor_thread_.join();
    }
    publisher_count_timer_.reset();
    lowstate_sub_.reset();
    arm_sdk_rt_pub_.reset();
    if (executor_ && node_)
    {
        executor_->remove_node(node_);
    }
    executor_.reset();
    node_.reset();
}

bool G1ArmSdkSystem::isStale(
    const std::chrono::steady_clock::time_point& arrival,
    std::chrono::steady_clock::duration          timeout)
{
    return (std::chrono::steady_clock::now() - arrival) > timeout;
}

std::chrono::steady_clock::duration G1ArmSdkSystem::lowstateTimeoutDuration() const
{
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(lowstate_timeout_s_));
}

void G1ArmSdkSystem::checkPublisherCount()
{
    if (mode_.load(std::memory_order_relaxed) == BlendMode::kInactive)
    {
        return;
    }
    if (conflict_ramp_thread_.joinable())
    {
        return;  // already handling a previously detected conflict
    }
    if (node_->count_publishers("/arm_sdk") <= 1)
    {
        return;
    }

    // Advisory only: real cross-process arbitration is a future
    // behavior-tree authority arbiter. This just refuses to let two
    // publishers command the arms at once -- two publishers owning one
    // low-level channel is unsafe -- by ramping ourselves down. Off this timer's
    // own thread (not the internal executor thread directly) so /lowstate
    // reception and this same timer keep servicing while it blocks.
    RCLCPP_ERROR(
        node_->get_logger(),
        "second /arm_sdk publisher detected while active -- ramping down (advisory guard only)");
    conflict_ramp_thread_ =
        std::thread([this] { rampDownSynchronously(BlendMode::kEmergencyRampDown); });
}

void G1ArmSdkSystem::rampDownSynchronously(BlendMode target_mode)
{
    if (mode_.load(std::memory_order_acquire) == BlendMode::kInactive)
    {
        return;  // already down -- an earlier ramp-down already got here
    }

    // See this method's declaration for why claiming unconditionally (not
    // just after detecting a conflict) is correct here rather than a race.
    writer_token_claimed_.store(true, std::memory_order_release);
    mode_.store(target_mode, std::memory_order_release);

    const double dt     = std::chrono::duration<double>(kRampDownTickPeriod).count();
    double       weight = ramp_engine_.weight();
    while (weight > 0.0)
    {
        weight = ramp_engine_.step(target_mode, command_position_, dt);
        if (arm_sdk_rt_pub_)
        {
            // A blocking lock (not trylock) is fine here: this isn't the RT
            // path, and nothing else can be publishing concurrently once
            // the token above is claimed.
            arm_sdk_rt_pub_->lock();
            assembleLowCmd(
                arm_sdk_rt_pub_->msg_,
                motor_index_,
                ramp_engine_.publishedPositions(),
                kp_,
                kd_,
                static_cast<float>(weight));
            vendored::computeLowCmdCrc(arm_sdk_rt_pub_->msg_);
            arm_sdk_rt_pub_->unlockAndPublish();
        }
        std::this_thread::sleep_for(kRampDownTickPeriod);
    }

    mode_.store(BlendMode::kInactive, std::memory_order_release);
}

}  // namespace g1_hardware_interface

PLUGINLIB_EXPORT_CLASS(g1_hardware_interface::G1ArmSdkSystem, hardware_interface::SystemInterface)
