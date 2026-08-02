/**
 * @file motion_service_sim_node.cpp
 * @brief Sim-only bridge that turns /arm_sdk weighted commands into /lowcmd for unitree_mujoco,
 * and a protocol-only responder for the LocoClient wire contract (/api/sport/request,
 * /api/sport/response).
 */

#include "g1_bringup/motion_service_sim_node.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>

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

/// Just enough shape-checking on a 7105 SET_VELOCITY parameter to reject something that plainly
/// isn't one. This responder is protocol-only and never reads vx/vy/vyaw/duration themselves
/// (see dispatchSportRequest()) -- there is nothing beyond shape worth validating.
bool looksLikeSetVelocityPayload(const std::string& json_text)
{
    try
    {
        const auto js = nlohmann::json::parse(json_text);
        return js.contains("velocity") && js["velocity"].is_array() && js.contains("duration");
    }
    catch (const nlohmann::json::parse_error&)
    {
        return false;
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

    RCLCPP_WARN(
        get_logger(),
        "motion_service_sim is SIM-ONLY: it owns /lowcmd in this process and answers "
        "/api/sport/* protocol-only (no leg motion); must never run against real hardware (see "
        "README.md).");
}

void MotionServiceSim::lowstateCallback(const unitree_hg::msg::LowState::ConstSharedPtr& msg)
{
    if (hold_pose_captured_)
    {
        return;
    }
    for (int i = 0; i < kNumBodyMotors; ++i)
    {
        hold_q_[static_cast<std::size_t>(i)] = msg->motor_state[static_cast<std::size_t>(i)].q;
    }
    hold_pose_captured_ = true;
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

    // assembleSimLowCmd() (blend_math) does the leg/waist stiff-hold, arm blend, and weight-slot
    // echo -- unit-tested directly (test/test_assemble_sim_low_cmd.cpp) without a live node or
    // DDS.
    unitree_hg::msg::LowCmd cmd = assembleSimLowCmd(
        hold_q_,
        arm_cmd_q_,
        arm_cmd_kp_,
        arm_cmd_kd_,
        effective_weight_,
        leg_kp_,
        leg_kd_,
        waist_kp_,
        waist_kd_,
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
        return result.error_code;
    }
    if (api_id == kApiIdSetVelocity)
    {
        if (!looksLikeSetVelocityPayload(parameter))
        {
            return kCodeTaskUnknownError;
        }
        return checkVelocityAllowed(loco_fsm_state_);
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
