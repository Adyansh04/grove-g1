/**
 * @file motion_service_sim_node.cpp
 * @brief Sim-only bridge that turns /arm_sdk weighted commands into /lowcmd for unitree_mujoco,
 * and a protocol-only responder for the LocoClient wire contract (/api/sport/request,
 * /api/sport/response).
 */

#include "g1_bringup/motion_service_sim_node.hpp"

#include <cmath>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "g1_bringup/blend_math.hpp"
#include "g1_hardware_interface/motor_crc_hg.hpp"

namespace g1_bringup
{

namespace
{
// LocoClient wire API ids this responder answers, duplicated from g1_locomotion's own
// loco_api_ids.hpp rather than depending on that package: those headers are an unexported
// internal build artifact (see g1_locomotion/CMakeLists.txt), and these are stable wire-protocol
// integers, not logic worth sharing a build target over.
constexpr std::int64_t kApiIdGetFsmId    = 7001;
constexpr std::int64_t kApiIdSetFsmId    = 7101;
constexpr std::int64_t kApiIdSetVelocity = 7105;
constexpr std::int64_t kApiIdSetArmTask  = 7106;

/// UT_ROBOT_TASK_UNKNOWN_ERROR -- matches g1_locomotion::kCodeTaskUnknownError.
constexpr std::int32_t kCodeTaskUnknownError = -2;

/// Parses `{"data": <int>}` -- the shape both 7101's request parameter and 7001's response data
/// share. Malformed JSON or a non-integer `data` field are both reported as nullopt, never as an
/// exception escaping to the caller.
std::optional<int> parseIntDataField(const std::string& json_text)
{
    try
    {
        const auto js = nlohmann::json::parse(json_text);
        if (!js.contains("data") || !js["data"].is_number_integer())
        {
            return std::nullopt;
        }
        return js["data"].get<int>();
    }
    catch (const nlohmann::json::parse_error&)
    {
        return std::nullopt;
    }
}

/// A parsed 7105 SET_VELOCITY parameter: {"velocity":[vx,vy,vyaw],"duration":d}.
struct SetVelocityPayload
{
    double vx{ 0.0 };
    double vy{ 0.0 };
    double vyaw{ 0.0 };
    double duration_s{ 0.0 };
};

/// Parses a 7105 SET_VELOCITY parameter, rejecting anything that isn't one. Returns nullopt for
/// malformed JSON, a missing/short velocity array, or non-numeric entries -- never throws.
std::optional<SetVelocityPayload> parseSetVelocityPayload(const std::string& json_text)
{
    try
    {
        const auto js = nlohmann::json::parse(json_text);
        if (!js.contains("velocity") || !js["velocity"].is_array() || js["velocity"].size() < 3 ||
            !js.contains("duration") || !js["duration"].is_number())
        {
            return std::nullopt;
        }
        const auto& v = js["velocity"];
        for (std::size_t i = 0; i < 3; ++i)
        {
            if (!v[i].is_number())
            {
                return std::nullopt;
            }
        }
        return SetVelocityPayload{ v[0].get<double>(),
                                   v[1].get<double>(),
                                   v[2].get<double>(),
                                   js["duration"].get<double>() };
    }
    catch (const nlohmann::json::parse_error&)
    {
        return std::nullopt;
    }
}
}  // namespace

/**
 * @brief Declare parameters, wire the /lowstate, /arm_sdk, and /lowcmd topics, and start the
 * publish timer for this sim-only bridge.
 *
 * @throws std::invalid_argument If publish_rate_hz, arm_sdk_timeout_ms, or timeout_ramp_down_s
 *         resolve to a non-positive value.
 */
MotionServiceSim::MotionServiceSim(const rclcpp::NodeOptions& options)
  : rclcpp::Node("motion_service_sim", options)
{
    publish_rate_hz_                = declare_parameter("publish_rate_hz", 500.0);
    leg_kp_                         = declare_parameter("leg_kp", 100.0);
    leg_kd_                         = declare_parameter("leg_kd", 1.0);
    waist_kp_                       = declare_parameter("waist_kp", 50.0);
    waist_kd_                       = declare_parameter("waist_kd", 1.0);
    arm_hold_kp_                    = declare_parameter("arm_hold_kp", 40.0);
    arm_hold_kd_                    = declare_parameter("arm_hold_kd", 1.0);
    const double arm_sdk_timeout_ms = declare_parameter("arm_sdk_timeout_ms", 500.0);
    arm_sdk_timeout_s_              = arm_sdk_timeout_ms / 1000.0;
    timeout_ramp_down_s_            = declare_parameter("timeout_ramp_down_s", 1.0);

    // Fail fast on a nonsensical rate or duration, mirroring G1ArmSdkSystem::on_init's
    // strictly-positive gate: publish_rate_hz <= 0 makes the wall-timer period's duration_cast
    // undefined, and a non-positive arm_sdk_timeout_s/timeout_ramp_down_s turns the no-snap
    // staleness decay into a snap (or an upward ramp) via stepEffectiveWeight's max_step.
    //
    // Gains are left unchecked, same as on_init -- only the rate and duration tunables can cause
    // UB or a snap when misconfigured.
    if (publish_rate_hz_ <= 0.0 || arm_sdk_timeout_s_ <= 0.0 || timeout_ramp_down_s_ <= 0.0)
    {
        RCLCPP_FATAL(
            get_logger(),
            "publish_rate_hz (%f), arm_sdk_timeout_ms (%f s), and timeout_ramp_down_s (%f) must "
            "all be strictly positive",
            publish_rate_hz_,
            arm_sdk_timeout_s_,
            timeout_ramp_down_s_);
        throw std::invalid_argument(
            "motion_service_sim: publish_rate_hz/arm_sdk_timeout_ms/timeout_ramp_down_s must be "
            "strictly positive");
    }

    // Best-effort: matches unitree_mujoco's rt/lowstate publisher QoS family (verified RELIABLE
    // in the milestone-1 spike, which a best-effort reader is still compatible with); only the
    // newest of the ~900 Hz stream ever matters.
    const auto lowstate_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    lowstate_sub_           = create_subscription<unitree_hg::msg::LowState>(
        "/lowstate",
        lowstate_qos,
        [this](const unitree_hg::msg::LowState::ConstSharedPtr& msg) { lowstateCallback(msg); });

    // Reliable: matches g1_hardware_interface's G1ArmSdkSystem, the sole /arm_sdk publisher in
    // this stack.
    const auto arm_sdk_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    arm_sdk_sub_           = create_subscription<unitree_hg::msg::LowCmd>(
        "/arm_sdk",
        arm_sdk_qos,
        [this](const unitree_hg::msg::LowCmd::ConstSharedPtr& msg) { armSdkCallback(msg); });

    // Best-effort: matches unitree_mujoco's rt/lowcmd subscription exactly (verified in the
    // milestone-1 spike). This bridge is the sim's only /lowcmd publisher, so there's no
    // reliability contention to protect against, and only the newest tick matters.
    const auto lowcmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    lowcmd_pub_           = create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", lowcmd_qos);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] {
            publishTick();
        });

    // Vendor-matched RELIABILITY/DURABILITY -- must not deviate, that's what hardware endpoint
    // compatibility depends on (see g1_locomotion's README for the identical rule on the bridge
    // side). HISTORY depth isn't RxO-matched, so it's ours to pick per side: the response
    // publisher stays at depth 1 (only the newest reply matters), but this request reader goes
    // deeper -- two requests landing in the same DDS batch (e.g. a SET_VELOCITY re-issue and a
    // GET_FSM_ID heartbeat poll) must not overwrite each other in a depth-1 KEEP_LAST cache
    // before this callback drains them.
    const auto sport_request_qos  = rclcpp::QoS(10).reliable().durability_volatile();
    const auto sport_response_qos = rclcpp::QoS(1).reliable().durability_volatile();
    sport_request_sub_            = create_subscription<unitree_api::msg::Request>(
        "/api/sport/request",
        sport_request_qos,
        [this](const unitree_api::msg::Request::ConstSharedPtr& msg) { sportRequestCallback(msg); });
    sport_response_pub_ =
        create_publisher<unitree_api::msg::Response>("/api/sport/response", sport_response_qos);

    // Base linear velocity for the policy observation, which /lowstate does not carry. Same
    // best-effort family as /lowstate: it comes from the same 1 kHz sim bridge and only the newest
    // sample matters.
    const auto sportmodestate_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    sportmodestate_sub_ = create_subscription<unitree_go::msg::SportModeState>(
        "/sportmodestate",
        sportmodestate_qos,
        [this](const unitree_go::msg::SportModeState::ConstSharedPtr& msg) {
            sportModeStateCallback(msg);
        });

    if (setUpWalkPolicy())
    {
        RCLCPP_WARN(
            get_logger(),
            "motion_service_sim is SIM-ONLY: it owns /lowcmd in this process, walks the robot with "
            "an RL policy, and answers /api/sport/*; must never run against real hardware (see "
            "README.md).");
    }
    else
    {
        RCLCPP_WARN(
            get_logger(),
            "motion_service_sim is SIM-ONLY: it owns /lowcmd in this process and answers "
            "/api/sport/* (walking policy disabled -- legs stiff-hold); must never run against "
            "real hardware (see README.md).");
    }
}

bool MotionServiceSim::setUpWalkPolicy()
{
    walk_policy_enabled_          = declare_parameter("walk_policy.enabled", false);
    const auto   model_path_param = declare_parameter("walk_policy.onnx_model_path", std::string{});
    const double rate_hz          = declare_parameter("walk_policy.rate_hz", 50.0);
    walk_policy_staleness_timeout_s_ =
        declare_parameter("walk_policy.staleness_timeout_ms", 100.0) / 1000.0;
    const auto joint_names =
        declare_parameter("walk_policy.joint_names", std::vector<std::string>{});
    const auto default_joint_pos =
        declare_parameter("walk_policy.default_joint_pos", std::vector<double>{});
    const auto action_scales =
        declare_parameter("walk_policy.action_scales", std::vector<double>{});
    const auto lower_kp = declare_parameter("walk_policy.lower_kp", std::vector<double>{});
    const auto lower_kd = declare_parameter("walk_policy.lower_kd", std::vector<double>{});
    const auto max_velocity =
        declare_parameter("walk_policy.max_velocity", std::vector<double>{ 1.0, 0.8, 2.0 });
    const auto thresholds = declare_parameter(
        "walk_policy.gait_initiation_threshold",
        std::vector<double>{ 0.4, 0.5, 1.5 });
    walk_policy_config_.velocity_duration_max_s =
        declare_parameter("walk_policy.velocity_duration_max_s", 2.0);

    if (!walk_policy_enabled_)
    {
        RCLCPP_INFO(
            get_logger(),
            "walking policy disabled -- legs and waist stiff-hold the captured pose (this is what "
            "pin_pelvis:=true selects, since the weld and the policy cannot both own the legs)");
        return false;
    }

    // Every gain and target below is indexed by DDS motor number, so a permuted joint list would
    // silently drive the wrong joints. Refuse rather than guess.
    const auto order_problem = checkJointOrder(joint_names);
    if (!order_problem.empty())
    {
        RCLCPP_ERROR(
            get_logger(),
            "walk_policy.joint_names does not match the Unitree DDS motor order (%s) -- disabling "
            "the policy; legs will stiff-hold",
            order_problem.c_str());
        walk_policy_enabled_ = false;
        return false;
    }
    if (default_joint_pos.size() != kNumBodyMotors || action_scales.size() != kNumBodyMotors ||
        lower_kp.size() != kNumLowerMotors || lower_kd.size() != kNumLowerMotors ||
        max_velocity.size() != 3 || thresholds.size() != 3 || rate_hz <= 0.0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "walk_policy parameters are malformed (expected %d default_joint_pos/action_scales, "
            "%d lower_kp/lower_kd, 3 max_velocity, 3 gait_initiation_threshold, positive rate_hz) "
            "-- disabling the policy; legs will stiff-hold",
            kNumBodyMotors,
            kNumLowerMotors);
        walk_policy_enabled_ = false;
        return false;
    }

    std::copy(
        default_joint_pos.begin(),
        default_joint_pos.end(),
        walk_policy_config_.default_joint_pos.begin());
    std::copy(action_scales.begin(), action_scales.end(), walk_policy_config_.action_scales.begin());
    std::copy(lower_kp.begin(), lower_kp.end(), walk_policy_config_.lower_kp.begin());
    std::copy(lower_kd.begin(), lower_kd.end(), walk_policy_config_.lower_kd.begin());
    std::copy(max_velocity.begin(), max_velocity.end(), walk_policy_config_.max_velocity.begin());
    std::copy(
        thresholds.begin(),
        thresholds.end(),
        walk_policy_config_.gait_initiation_threshold.begin());

    // The weights are an external file ONNX Runtime resolves against the model path, so the
    // default points at the installed share directory where both were installed together.
    const std::string model_path =
        model_path_param.empty() ?
            ament_index_cpp::get_package_share_directory("g1_bringup") + "/policy/walker.onnx" :
            model_path_param;
    try
    {
        walk_policy_session_ = std::make_unique<WalkPolicySession>(model_path);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(
            get_logger(),
            "failed to load the walking policy from '%s' (%s) -- disabling the policy; legs will "
            "stiff-hold",
            model_path.c_str(),
            e.what());
        walk_policy_enabled_ = false;
        return false;
    }

    const auto policy_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    walk_policy_timer_ = create_wall_timer(policy_period, [this] { walkPolicyTick(); });

    RCLCPP_INFO(
        get_logger(),
        "walking policy loaded from '%s' (warm-up %.1f ms), running at %.1f Hz and owning motors "
        "0-%d; /arm_sdk keeps the arms",
        model_path.c_str(),
        walk_policy_session_->warmupSeconds() * 1000.0,
        rate_hz,
        kNumLowerMotors - 1);
    return true;
}

void MotionServiceSim::lowstateCallback(const unitree_hg::msg::LowState::ConstSharedPtr& msg)
{
    // Refreshed every sample, unlike hold_q_ below: the policy observes live joint state.
    for (std::size_t i = 0; i < kNumBodyMotors; ++i)
    {
        walk_inputs_.joint_pos[i] = msg->motor_state[i].q;
        walk_inputs_.joint_vel[i] = msg->motor_state[i].dq;
    }
    for (std::size_t i = 0; i < 4; ++i)
    {
        walk_inputs_.base_quat[i] = msg->imu_state.quaternion[i];
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        walk_inputs_.base_ang_vel_body[i] = msg->imu_state.gyroscope[i];
    }

    if (!hold_pose_captured_)
    {
        for (int i = 0; i < kNumBodyMotors; ++i)
        {
            hold_q_[static_cast<std::size_t>(i)] = msg->motor_state[static_cast<std::size_t>(i)].q;
        }
        hold_pose_captured_ = true;
    }
}

void MotionServiceSim::sportModeStateCallback(
    const unitree_go::msg::SportModeState::ConstSharedPtr& msg)
{
    // World-frame; assembleObservation() rotates it into the base frame.
    for (std::size_t i = 0; i < 3; ++i)
    {
        walk_inputs_.base_lin_vel_world[i] = msg->velocity[i];
    }
    sportmodestate_received_ = true;
}

void MotionServiceSim::walkPolicyTick()
{
    // Both inputs must be live before the first inference: /lowstate for joints and orientation,
    // /sportmodestate for base linear velocity. Running on a half-populated observation would feed
    // the policy an assumed-zero base velocity, which is only true while the robot is at rest.
    // Starting earlier was tried and is measurably worse, not better: entering the policy at the
    // straight-legged spawn pose is a bigger step to the crouch than entering it after a brief
    // stiff hold (peak lower-body 14.6 rad/s vs 12.5).
    if (!walk_policy_session_ || !hold_pose_captured_ || !sportmodestate_received_)
    {
        return;
    }

    const auto now     = std::chrono::steady_clock::now();
    const auto command = activeCommand(walk_velocity_, now);
    const auto observation =
        assembleObservation(walk_inputs_, walk_policy_config_, walk_last_action_, command);

    walk_last_action_  = walk_policy_session_->run(observation);
    const auto targets = actionToJointTargets(walk_last_action_, walk_policy_config_);
    std::copy(targets.begin(), targets.begin() + kNumLowerMotors, walk_target_q_.begin());
    walk_target_valid_ = true;
    walk_target_stamp_ = now;
}

void MotionServiceSim::armSdkCallback(const unitree_hg::msg::LowCmd::ConstSharedPtr& msg)
{
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumArmMotors); ++i)
    {
        const auto& motor = msg->motor_cmd[static_cast<std::size_t>(kFirstArmMotor) + i];
        arm_cmd_q_[i]     = motor.q;
        arm_cmd_kp_[i]    = motor.kp;
        arm_cmd_kd_[i]    = motor.kd;
    }
    arm_cmd_weight_   = msg->motor_cmd[kWeightMotorIndex].q;
    arm_sdk_received_ = true;
    arm_sdk_arrival_  = std::chrono::steady_clock::now();
}

void MotionServiceSim::publishTick()
{
    if (!hold_pose_captured_)
    {
        return;  // nothing to hold yet -- no /lowstate sample has arrived
    }

    const auto   now = std::chrono::steady_clock::now();
    const double dt  = last_tick_.time_since_epoch().count() == 0 ?
                           1.0 / publish_rate_hz_ :
                           std::chrono::duration<double>(now - last_tick_).count();
    last_tick_       = now;

    // No /arm_sdk received yet is treated the same as stale: the effective weight target is 0
    // either way, so arms simply hold at hold_q_ until the first real command shows up.
    bool stale = true;
    if (arm_sdk_received_)
    {
        const auto age_s = std::chrono::duration<double>(now - arm_sdk_arrival_).count();
        stale            = age_s > arm_sdk_timeout_s_;
    }
    effective_weight_ =
        stepEffectiveWeight(effective_weight_, arm_cmd_weight_, stale, timeout_ramp_down_s_, dt);

    // The leg-authority decision, in one place: the walking policy owns motors 0-14 only while
    // its targets are FRESH. A stalled inference falls back to the stiff hold rather than
    // re-publishing a stale target, which is the same "never keep commanding from a dead source"
    // rule the arm side's staleness ramp enforces.
    const bool policy_fresh = walk_policy_enabled_ && walk_target_valid_ &&
                              std::chrono::duration<double>(now - walk_target_stamp_).count() <=
                                  walk_policy_staleness_timeout_s_;

    std::array<double, kFirstArmMotor> lower_q{};
    std::array<double, kFirstArmMotor> lower_kp{};
    std::array<double, kFirstArmMotor> lower_kd{};
    for (std::size_t i = 0; i < kFirstArmMotor; ++i)
    {
        const bool is_waist = static_cast<int>(i) >= kNumLegMotors;
        lower_q[i]          = policy_fresh ? walk_target_q_[i] : hold_q_[i];
        lower_kp[i] =
            policy_fresh ? walk_policy_config_.lower_kp[i] : (is_waist ? waist_kp_ : leg_kp_);
        lower_kd[i] =
            policy_fresh ? walk_policy_config_.lower_kd[i] : (is_waist ? waist_kd_ : leg_kd_);
    }
    if (walk_policy_enabled_ && !policy_fresh)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "walking policy targets are stale -- legs and waist are stiff-holding the captured "
            "pose instead");
    }

    // assembleSimLowCmd() (blend_math) does the lower-body assignment, arm blend, and weight-slot
    // echo -- unit-tested directly (test/test_assemble_sim_low_cmd.cpp) without a live node or
    // DDS.
    unitree_hg::msg::LowCmd cmd = assembleSimLowCmd(
        hold_q_,
        lower_q,
        lower_kp,
        lower_kd,
        arm_cmd_q_,
        arm_cmd_kp_,
        arm_cmd_kd_,
        effective_weight_,
        arm_hold_kp_,
        arm_hold_kd_);

    // mode/mode_pr/mode_machine are deliberately left unset: unitree_mujoco computes actuator
    // torque as tau_ff + kp * (q_des - q_meas) + kd * (dq_des - dq_meas) per motor slot and never
    // reads the mode fields (simulate/src/unitree_sdk2_bridge.h, RobotBridge::run()).
    //
    // That's a sim-specific finding -- what the real motion service does with these fields is
    // unverified and stays a hardware re-validation item.
    g1_hardware_interface::vendored::computeLowCmdCrc(cmd);
    lowcmd_pub_->publish(cmd);
}

void MotionServiceSim::sportRequestCallback(const unitree_api::msg::Request::ConstSharedPtr& msg)
{
    unitree_api::msg::Response response;
    // Correlation contract: g1_locomotion's LocoRequestCorrelator matches purely on
    // header.identity.id, but the wire contract carries api_id alongside it in the same struct --
    // echo the whole identity back verbatim regardless of what this handler does with the
    // request.
    response.header.identity = msg->header.identity;
    response.header.status.code =
        dispatchSportRequest(msg->header.identity.api_id, msg->parameter, response.data);
    sport_response_pub_->publish(response);
}

std::int32_t MotionServiceSim::dispatchSportRequest(
    std::int64_t api_id, const std::string& parameter, std::string& response_data)
{
    if (api_id == kApiIdGetFsmId)
    {
        nlohmann::json js;
        js["data"]    = loco_fsm_state_;
        response_data = js.dump();
        return kLocoFsmCodeSuccess;
    }
    if (api_id == kApiIdSetFsmId)
    {
        const auto requested = parseIntDataField(parameter);
        if (!requested)
        {
            return kCodeTaskUnknownError;
        }
        const auto result = applySetFsmId(loco_fsm_state_, *requested);
        loco_fsm_state_   = result.new_state;
        // Leaving Start gives up locomotion authority, so the last velocity must not stay in
        // force: without this a Start -> Damp transition would leave the robot walking on a
        // command whose owner has already released. Releasing cleanly on the transition itself is
        // the responder-side half of the same rule the bridge's authority machine follows.
        if (loco_fsm_state_ != kFsmStart)
        {
            walk_velocity_.reset();
        }
        return result.error_code;
    }
    if (api_id == kApiIdSetVelocity)
    {
        const auto payload = parseSetVelocityPayload(parameter);
        if (!payload)
        {
            return kCodeTaskUnknownError;
        }
        // The EXISTING legality table decides, and it decides first: this is the authority gate,
        // not a second one alongside it. Outside Start the command is rejected with 7301 and
        // nothing is latched, so no velocity can reach the policy without the real
        // Damp -> StandUp -> Start sequence having been driven through the bridge.
        const std::int32_t code = checkVelocityAllowed(loco_fsm_state_);
        if (code != kLocoFsmCodeSuccess)
        {
            return code;
        }

        const auto command =
            clampVelocity(payload->vx, payload->vy, payload->vyaw, walk_policy_config_);
        if (isBelowGaitThreshold(command, walk_policy_config_))
        {
            // Passed through unmodified regardless -- scaling a small command UP to force motion
            // is exactly what CONTROL_MODES.md forbids. The operator gets told instead.
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "commanded velocity (%.2f, %.2f, %.2f) is below this policy's measured "
                "gait-initiation thresholds (%.2f, %.2f, %.2f) -- the robot will stand still "
                "rather than step (see README)",
                command[0],
                command[1],
                command[2],
                walk_policy_config_.gait_initiation_threshold[0],
                walk_policy_config_.gait_initiation_threshold[1],
                walk_policy_config_.gait_initiation_threshold[2]);
        }
        walk_velocity_ = latchVelocity(
            command,
            payload->duration_s,
            std::chrono::steady_clock::now(),
            walk_policy_config_);
        return kLocoFsmCodeSuccess;
    }
    if (api_id == kApiIdSetArmTask)
    {
        // Deliberately unsupported: SET_ARM_TASK (WaveHand/ShakeHand-style moves) hands arm
        // authority to the onboard controller, fighting this stack's rt/arm_sdk blend weight
        // (see g1_locomotion's README -- the api id isn't even defined there, same reason).
        // Rejecting it here, not a silent no-op, makes the omission a tested fact.
        return kCodeTaskUnknownError;
    }
    return kCodeTaskUnknownError;
}

}  // namespace g1_bringup
