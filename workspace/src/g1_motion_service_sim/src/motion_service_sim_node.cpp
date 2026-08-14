/**
 * @file motion_service_sim_node.cpp
 * @brief Sim-only bridge that turns /arm_sdk weighted commands into /lowcmd for unitree_mujoco,
 * and a protocol-only responder for the LocoClient wire contract (/api/sport/request,
 * /api/sport/response).
 */

#include "g1_motion_service_sim/motion_service_sim_node.hpp"

#include <array>
#include <cmath>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "g1_hardware_interface/motor_crc_hg.hpp"
#include "g1_motion_service_sim/blend_math.hpp"

namespace g1_motion_service_sim
{

namespace
{
// Duplicated from g1_locomotion's loco_api_ids.hpp (unexported internal header).
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
    mask_arm_observations_          = declare_parameter("mask_arm_observations", true);
    arm_hold_tracking_weight_       = declare_parameter("arm_hold_tracking_weight", 0.05);
    const double arm_sdk_timeout_ms = declare_parameter("arm_sdk_timeout_ms", 500.0);
    arm_sdk_timeout_s_              = arm_sdk_timeout_ms / 1000.0;
    timeout_ramp_down_s_            = declare_parameter("timeout_ramp_down_s", 1.0);

    // The waist belongs to the onboard controller, so nothing in this stack can command it and
    // the sim model always spawns it at zero. Overriding the captured hold target is the only
    // way to stand the torso anywhere else, which manipulation needs: a waist locked at zero
    // hides every error in the pelvis-to-torso transform that arm planning depends on.
    waist_hold_rad_ = declare_parameter("waist_hold_rad", std::vector<double>{});
    if (!waist_hold_rad_.empty() &&
        waist_hold_rad_.size() != static_cast<std::size_t>(kFirstArmMotor - kNumLegMotors))
    {
        RCLCPP_ERROR(
            get_logger(),
            "waist_hold_rad needs %d values (yaw, roll, pitch) but got %zu -- ignoring it and "
            "holding the captured waist pose",
            kFirstArmMotor - kNumLegMotors,
            waist_hold_rad_.size());
        waist_hold_rad_.clear();
    }

    arm_hold_rad_ = declare_parameter("arm_hold_rad", std::vector<double>{});
    if (!arm_hold_rad_.empty() && arm_hold_rad_.size() != kNumArmMotors)
    {
        RCLCPP_ERROR(
            get_logger(),
            "arm_hold_rad needs %d values (left arm then right, motors 15 to 28) but got %zu -- "
            "ignoring it and holding whatever the arms fell into at startup",
            kNumArmMotors,
            arm_hold_rad_.size());
        arm_hold_rad_.clear();
    }

    // Fail fast on non-positive rate/duration (same gate as G1ArmSdkSystem::on_init).
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

    // Best-effort, depth 1 — matches unitree_mujoco's /lowstate publisher.
    const auto lowstate_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    // Off unless asked for. This work lands on the ~1 kHz /lowstate callback, which is the
    // walking policy's own path, so only the sensor track pays for it. sim.launch.py turns
    // it on together with sensors, because that is what needs pelvis -> torso_link.
    publish_non_arm_joints_ = declare_parameter<bool>("publish_non_arm_joint_states", false);

    // Latest-only: robot_state_publisher merges by joint name, so this coexists with
    // joint_state_broadcaster's arm-only publication rather than competing with it.
    non_arm_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(rclcpp::KeepLast(1)));
    non_arm_joint_msg_.name.reserve(kNumLowerMotors);
    non_arm_joint_msg_.position.resize(kNumLowerMotors);
    non_arm_joint_msg_.velocity.resize(kNumLowerMotors);
    for (int i = 0; i < kNumLowerMotors; ++i)
    {
        non_arm_joint_msg_.name.emplace_back(kDdsMotorOrder[i]);
    }

    lowstate_sub_ = create_subscription<unitree_hg::msg::LowState>(
        "/lowstate",
        lowstate_qos,
        [this](const unitree_hg::msg::LowState::ConstSharedPtr& msg) { lowstateCallback(msg); });

    // Reliable, depth 1 — matches G1ArmSdkSystem's /arm_sdk publisher.
    const auto arm_sdk_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    arm_sdk_sub_           = create_subscription<unitree_hg::msg::LowCmd>(
        "/arm_sdk",
        arm_sdk_qos,
        [this](const unitree_hg::msg::LowCmd::ConstSharedPtr& msg) { armSdkCallback(msg); });

    // Best-effort, depth 1 — matches unitree_mujoco's /lowcmd subscription.
    const auto lowcmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    lowcmd_pub_           = create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", lowcmd_qos);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] {
            publishTick();
        });

    // Vendor-matched reliability/durability. Request reader depth 10 to avoid
    // overwrites when multiple requests land in one DDS batch.
    const auto sport_request_qos  = rclcpp::QoS(10).reliable().durability_volatile();
    const auto sport_response_qos = rclcpp::QoS(1).reliable().durability_volatile();
    sport_request_sub_            = create_subscription<unitree_api::msg::Request>(
        "/api/sport/request",
        sport_request_qos,
        [this](const unitree_api::msg::Request::ConstSharedPtr& msg) { sportRequestCallback(msg); });
    sport_response_pub_ =
        create_publisher<unitree_api::msg::Response>("/api/sport/response", sport_response_qos);

    // Best-effort — base linear velocity for the policy observation.
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
            ament_index_cpp::get_package_share_directory("g1_motion_service_sim") +
                "/policy/walker.onnx" :
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
    // Legs and waist, which nothing else publishes. Motor index equals kDdsMotorOrder
    // index, asserted by test_walk_policy's joint-order check. Decimated to ~100 Hz:
    // /lowstate is ~1 kHz and robot_state_publisher has no use for that.
    if (publish_non_arm_joints_ && ++non_arm_joint_decimate_ >= 10)
    {
        non_arm_joint_decimate_         = 0;
        non_arm_joint_msg_.header.stamp = now();
        for (int i = 0; i < kNumLowerMotors; ++i)
        {
            non_arm_joint_msg_.position[i] = msg->motor_state[i].q;
            non_arm_joint_msg_.velocity[i] = msg->motor_state[i].dq;
        }
        non_arm_joint_pub_->publish(non_arm_joint_msg_);
    }

    // Refreshed every sample, unlike hold_q_ below: the policy observes live joint state.
    for (std::size_t i = 0; i < kNumBodyMotors; ++i)
    {
        walk_inputs_.joint_pos[i] = msg->motor_state[i].q;
        walk_inputs_.joint_vel[i] = msg->motor_state[i].dq;
    }

    // The arms are hidden from the policy, and this is the single change that makes this
    // simulator behave like the real robot rather than worse than it.
    //
    // The policy owns all 29 joints on paper, but /arm_sdk owns 15-28 here, so its arm actions
    // are thrown away (walk_policy.yaml). It still SEES those joints: 42 of the 99 observation
    // dimensions are arm position, arm velocity and arm last-action. During training the arms
    // only ever sat within the policy's own action distribution around the default posture --
    // about one action-scale unit, and the arm scales are 0.438577, which is exactly the
    // measured plus/minus 0.4 rad envelope this robot holds stably. A MoveIt goal of "arms
    // straight ahead" is roughly four units out, so the policy is extrapolating on a third of
    // its input.
    //
    // The real G1 has no equivalent failure: its onboard controller is not this policy, and per
    // CMU's G1 notes it "is not aware of how the arms are being controlled" -- it rejects arm
    // motion reactively, through the IMU, as an unmodelled disturbance. Showing our policy the
    // default posture reproduces exactly that: the physical disturbance still reaches it through
    // the base state, but the out-of-distribution input that hardware never suffers does not.
    // So this makes sim MORE faithful, not less -- today sim fails in a way hardware will not.
    //
    // Same approach HuggingFace ship in LeRobot's production G1 integration
    // (robots/unitree_g1/holosoma_locomotion.py) against a near-identical 100-obs/29-action
    // policy, for the same reason: "prevents policy from reacting to teleop arm movements".
    if (mask_arm_observations_)
    {
        for (std::size_t i = 0; i < kNumArmMotors; ++i)
        {
            const std::size_t motor       = static_cast<std::size_t>(kFirstArmMotor) + i;
            walk_inputs_.joint_pos[motor] = walk_policy_config_.default_joint_pos[motor];
            walk_inputs_.joint_vel[motor] = 0.0;
        }
    }

    // While /arm_sdk owns the arms, the hold target follows them, so a release blends toward
    // where the arms actually are instead of dragging them back to the spawn pose. Below the
    // threshold it freezes: a target that keeps chasing the measurement has no position error
    // to hold against, and the arms would sag on damping alone.
    if (hold_pose_captured_ && effective_weight_ > arm_hold_tracking_weight_)
    {
        for (std::size_t i = 0; i < kNumArmMotors; ++i)
        {
            const std::size_t motor = static_cast<std::size_t>(kFirstArmMotor) + i;
            hold_q_[motor]          = msg->motor_state[motor].q;
        }
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
        // Applied to the captured pose rather than to every command, so the waist ramps to the
        // target through the same stiff-hold PD as any other hold -- no snap, and /lowstate
        // still reports whatever the joint actually reached.
        for (std::size_t i = 0; i < waist_hold_rad_.size(); ++i)
        {
            hold_q_[static_cast<std::size_t>(kNumLegMotors) + i] = waist_hold_rad_[i];
        }
        // Same treatment, and for a stronger reason: the arms have been falling freely since
        // the simulator started, so what the capture reads is a startup transient rather than a
        // posture. See the member's comment.
        for (std::size_t i = 0; i < arm_hold_rad_.size(); ++i)
        {
            hold_q_[static_cast<std::size_t>(kFirstArmMotor) + i] = arm_hold_rad_[i];
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
    // Wait for both /lowstate and /sportmodestate before first inference.
    // Running earlier with zeros is measurably worse.
    if (!walk_policy_session_ || !hold_pose_captured_ || !sportmodestate_received_)
    {
        return;
    }
    const auto now     = std::chrono::steady_clock::now();
    const auto command = activeCommand(walk_velocity_, now);
    const auto observation =
        assembleObservation(walk_inputs_, walk_policy_config_, walk_last_action_, command);

    // Catch exceptions — a throw here would kill the process and collapse the robot.
    const auto action =
        runPolicyGuarded([this, &observation] { return walk_policy_session_->run(observation); });
    if (!action)
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "walking policy inference threw -- skipping this tick; leaving the freshness stamp "
            "untouched so the stiff-hold fallback engages if it keeps failing");
        return;
    }
    walk_last_action_ = *action;
    if (mask_arm_observations_)
    {
        // The other half of hiding the arms, and the half that is easy to miss. last_action is
        // fed straight back into the next observation, but only the lower 15 actions are ever
        // executed -- the arm actions are discarded and /arm_sdk drives those joints instead.
        // Left alone, the policy reads back arm commands that never happened, next to arm
        // positions produced by something else entirely, so the action-to-state relationship it
        // learned is broken on 14 joints every tick. Zero is the action that means "default
        // posture", which is what the masked position and velocity above already claim.
        //
        // FALCON's deployment code slices its feedback to executed joints for the same reason
        // (sim2real/rl_policy/dec_loco/dec_loco.py), arrived at independently.
        std::fill(walk_last_action_.begin() + kFirstArmMotor, walk_last_action_.end(), 0.0F);
    }
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

    // No /arm_sdk received yet is treated as stale (weight target = 0).
    bool stale = true;
    if (arm_sdk_received_)
    {
        const auto age_s = std::chrono::duration<double>(now - arm_sdk_arrival_).count();
        stale            = age_s > arm_sdk_timeout_s_;
    }
    effective_weight_ =
        stepEffectiveWeight(effective_weight_, arm_cmd_weight_, stale, timeout_ramp_down_s_, dt);

    // Walking policy owns motors 0-14 only while its targets are fresh.
    const bool policy_fresh = walk_policy_enabled_ && walk_target_valid_ &&
                              std::chrono::duration<double>(now - walk_target_stamp_).count() <=
                                  walk_policy_staleness_timeout_s_;

    // Computed once, as one value: see LegAuthority's own comment on why this is not two bools.
    const LegAuthority authority = !walk_target_valid_ ? LegAuthority::kHoldPose :
                                   policy_fresh        ? LegAuthority::kLivePolicy :
                                                         LegAuthority::kFrozenPolicy;
    const auto         lower     = selectLowerBodyCommand(
        authority,
        walk_target_q_,
        hold_q_,
        walk_policy_config_.lower_kp,
        walk_policy_config_.lower_kd,
        leg_kp_,
        leg_kd_,
        waist_kp_,
        waist_kd_);
    if (walk_policy_enabled_ && authority == LegAuthority::kFrozenPolicy)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "walking policy targets are stale -- legs and waist are frozen at the last policy "
            "output and held at stiff-hold gains");
    }

    // assembleSimLowCmd() does the lower-body assignment, arm blend, and weight-slot echo.
    unitree_hg::msg::LowCmd cmd = assembleSimLowCmd(
        hold_q_,
        lower.q,
        lower.kp,
        lower.kd,
        arm_cmd_q_,
        arm_cmd_kp_,
        arm_cmd_kd_,
        effective_weight_,
        arm_hold_kp_,
        arm_hold_kd_);

    // mode/mode_pr/mode_machine left unset — unitree_mujoco never reads them
    // (computes torque from q/kp/kd/dq directly). Real hardware may differ.
    g1_hardware_interface::vendored::computeLowCmdCrc(cmd);
    lowcmd_pub_->publish(cmd);
}

void MotionServiceSim::sportRequestCallback(const unitree_api::msg::Request::ConstSharedPtr& msg)
{
    unitree_api::msg::Response response;
    // Echo the full identity back for correlation.
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
        // Leaving Start releases locomotion authority — clear any latched velocity.
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
        // Velocity only allowed in Start state.
        const std::int32_t code = checkVelocityAllowed(loco_fsm_state_);
        if (code != kLocoFsmCodeSuccess)
        {
            return code;
        }

        const auto command =
            clampVelocity(payload->vx, payload->vy, payload->vyaw, walk_policy_config_);
        if (isBelowGaitThreshold(command, walk_policy_config_))
        {
            // Passed through unmodified regardless. Operator gets told.
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
        // Deliberately unsupported — SET_ARM_TASK conflicts with this stack's
        // rt/arm_sdk blend weight authority.
        return kCodeTaskUnknownError;
    }
    return kCodeTaskUnknownError;
}

}  // namespace g1_motion_service_sim
