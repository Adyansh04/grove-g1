/**
 * @file g1_loco_bridge_node.cpp
 * @brief LifecycleNode bridging cmd_vel/SetLocoMode onto the LocoClient wire contract.
 */
#include "g1_locomotion/g1_loco_bridge_node.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "g1_locomotion/loco_api_ids.hpp"
#include "g1_locomotion/loco_payloads.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_action/action_server.h"

namespace g1_locomotion
{

namespace
{
const char* const kSportRequestTopic  = "/api/sport/request";
const char* const kSportResponseTopic = "/api/sport/response";

constexpr std::chrono::milliseconds kSweepPeriod{ 50 };
constexpr std::chrono::seconds      kHeartbeatPeriod{ 1 };

/// Names the two SET_FSM_ID rejection codes this bridge's action clients actually need to
/// distinguish; anything else (a sweep() timeout, an unrecognised wire error) is reported as its
/// raw code rather than invented text.
std::string describeFsmRejection(std::int32_t error_code)
{
    if (error_code == kCodeLocoStateNotAvailable)
    {
        return "loco state not available";
    }
    if (error_code == kCodeInvalidFsmId)
    {
        return "invalid fsm id";
    }
    return "code " + std::to_string(error_code);
}
}  // namespace

G1LocoBridge::G1LocoBridge(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("g1_loco_bridge", options)
  , correlator_(LocoRequestCorrelator::Config{})
  , velocity_gate_(VelocityGate::Config{})
{
    declare_parameter("request_timeout_s", request_timeout_s_);
    declare_parameter("max_pending", static_cast<int>(max_pending_));
    declare_parameter("velocity_reissue_hz", velocity_reissue_hz_);
    declare_parameter("cmd_vel_timeout_ms", cmd_vel_timeout_s_ * 1000.0);
    declare_parameter("failure_streak_limit", failure_streak_limit_);
    declare_parameter(
        "max_velocity",
        std::vector<double>(max_velocity_.begin(), max_velocity_.end()));
    declare_parameter("axis_sign", std::vector<double>(axis_sign_.begin(), axis_sign_.end()));

    // Single callback group for all this node's callbacks.
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

bool G1LocoBridge::readParameters()
{
    request_timeout_s_             = get_parameter("request_timeout_s").as_double();
    const std::int64_t max_pending = get_parameter("max_pending").as_int();
    velocity_reissue_hz_           = get_parameter("velocity_reissue_hz").as_double();
    cmd_vel_timeout_s_             = get_parameter("cmd_vel_timeout_ms").as_double() / 1000.0;
    failure_streak_limit_   = static_cast<int>(get_parameter("failure_streak_limit").as_int());
    const auto max_velocity = get_parameter("max_velocity").as_double_array();
    const auto axis_sign    = get_parameter("axis_sign").as_double_array();

    if (max_velocity.size() != 3 || axis_sign.size() != 3)
    {
        RCLCPP_ERROR(
            get_logger(),
            "max_velocity and axis_sign must each have exactly 3 entries [vx, vy, vyaw]");
        return false;
    }
    std::copy(max_velocity.begin(), max_velocity.end(), max_velocity_.begin());
    std::copy(axis_sign.begin(), axis_sign.end(), axis_sign_.begin());

    if (velocity_reissue_hz_ <= 1.0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "velocity_reissue_hz (%f) must be > 1.0 -- otherwise duration's 1 s dead-man can "
            "expire between re-issues",
            velocity_reissue_hz_);
        return false;
    }
    if (request_timeout_s_ <= 0.0 || max_pending <= 0 || cmd_vel_timeout_s_ <= 0.0 ||
        failure_streak_limit_ <= 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "request_timeout_s/max_pending/cmd_vel_timeout_ms/failure_streak_limit must all be "
            "strictly positive");
        return false;
    }
    max_pending_ = static_cast<std::size_t>(max_pending);
    return true;
}

G1LocoBridge::CallbackReturn
G1LocoBridge::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    resetEntities();

    if (!readParameters())
    {
        return CallbackReturn::FAILURE;
    }
    correlator_ =
        LocoRequestCorrelator(LocoRequestCorrelator::Config{ request_timeout_s_, max_pending_ });
    velocity_gate_ =
        VelocityGate(VelocityGate::Config{ cmd_vel_timeout_s_, failure_streak_limit_ });
    last_known_fsm_id_     = -1;
    last_error_code_       = 0;
    last_published_status_ = g1_msgs::msg::LocoStatus{};

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = callback_group_;

    // Vendor-matched reliability/durability. Response reader depth 10 to avoid
    // overwrites when multiple responses land in one DDS batch.
    const auto sport_request_qos  = rclcpp::QoS(1).reliable().durability_volatile();
    const auto sport_response_qos = rclcpp::QoS(10).reliable().durability_volatile();
    request_pub_                  = create_publisher<unitree_api::msg::Request>(
        kSportRequestTopic,
        sport_request_qos,
        pub_options);
    response_sub_ = create_subscription<unitree_api::msg::Response>(
        kSportResponseTopic,
        sport_response_qos,
        [this](const unitree_api::msg::Response::ConstSharedPtr& msg) { onSportResponse(*msg); },
        sub_options);

    // Node-relative ~/cmd_vel — future orchestration layer handles arbitration.
    const auto cmd_vel_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    cmd_vel_sub_           = create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel",
        cmd_vel_qos,
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) { cmdVelCallback(msg); },
        sub_options);

    // Transient-local so late-joining monitors see the current status immediately.
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    status_pub_ = create_publisher<g1_msgs::msg::LocoStatus>("~/status", status_qos, pub_options);

    action_server_ = rclcpp_action::create_server<SetLocoMode>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        "~/set_mode",
        [this](
            const rclcpp_action::GoalUUID&                  uuid,
            const std::shared_ptr<const SetLocoMode::Goal>& goal) { return handleGoal(uuid, goal); },
        [](const std::shared_ptr<GoalHandleSetLocoMode>& goal_handle) {
            return handleCancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleSetLocoMode>& goal_handle) {
            handleAccepted(goal_handle);
        },
        rcl_action_server_get_default_options(),
        callback_group_);

    sweep_timer_ = create_wall_timer(
        kSweepPeriod,
        [this] { correlator_.sweep(std::chrono::steady_clock::now()); },
        callback_group_);
    const auto reissue_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / velocity_reissue_hz_));
    reissue_timer_ = create_wall_timer(
        reissue_period,
        [this] { onReissueTick(); },
        callback_group_);

    // Phase-offset heartbeat timer from reissue timer to avoid back-to-back
    // SET_VELOCITY and GET_FSM_ID on a single-call-in-flight channel
    // (causes ~20% SET_VELOCITY loss when they fire together).
    const auto heartbeat_phase_offset =
        std::chrono::duration_cast<std::chrono::nanoseconds>(reissue_period) / 2;
    heartbeat_phase_timer_ = create_wall_timer(
        kHeartbeatPeriod + heartbeat_phase_offset,
        [this] {
            heartbeat_phase_timer_->cancel();
            heartbeat_timer_ = create_wall_timer(
                kHeartbeatPeriod,
                [this] { onHeartbeatTick(); },
                callback_group_);
            onHeartbeatTick();
        },
        callback_group_);

    return CallbackReturn::SUCCESS;
}

G1LocoBridge::CallbackReturn
G1LocoBridge::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
    resetEntities();
    return CallbackReturn::SUCCESS;
}

G1LocoBridge::CallbackReturn G1LocoBridge::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    // Activate status_pub_ via base class before publishStatus().
    const auto base_result = LifecycleNode::on_activate(previous_state);
    if (base_result != CallbackReturn::SUCCESS)
    {
        return base_result;
    }
    publishStatus(/*force=*/true);
    RCLCPP_INFO(get_logger(), "g1_loco_bridge active -- accepting SetLocoMode goals");
    return CallbackReturn::SUCCESS;
}

G1LocoBridge::CallbackReturn
G1LocoBridge::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    // Terminate any in-flight SetLocoMode goal before releasing authority.
    // Supersede the correlator entry so a stale response can't re-acquire
    // authority after deactivation.
    if (active_goal_handle_)
    {
        if (pending_set_loco_mode_request_id_)
        {
            correlator_.supersede(*pending_set_loco_mode_request_id_);
            pending_set_loco_mode_request_id_.reset();
        }
        auto result        = std::make_shared<SetLocoMode::Result>();
        result->success    = false;
        result->error_code = kCodeTaskUnknownError;
        result->message    = "bridge deactivated while this goal was in flight";
        active_goal_handle_->abort(result);
        active_goal_handle_.reset();
    }

    if (velocity_gate_.authority() != LocoAuthority::kReleased)
    {
        RCLCPP_WARN(
            get_logger(),
            "deactivating while locomotion authority was not released -- forcing release");
        velocity_gate_.forceRelease();
    }
    return LifecycleNode::on_deactivate(previous_state);
}

G1LocoBridge::CallbackReturn
G1LocoBridge::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
    // No ramp-down needed — we don't actuate directly. The onboard controller's
    // 1 s duration dead-man takes over when re-issuing stops.
    resetEntities();
    return CallbackReturn::SUCCESS;
}

G1LocoBridge::CallbackReturn
G1LocoBridge::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
    resetEntities();
    return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse G1LocoBridge::handleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/, const std::shared_ptr<const SetLocoMode::Goal>& goal)
{
    // Self-gated: the action server has no lifecycle awareness in Humble,
    // so we must check state ourselves.
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        RCLCPP_WARN(get_logger(), "rejecting SetLocoMode goal: bridge is not active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->fsm_id != SetLocoMode::Goal::DAMP && goal->fsm_id != SetLocoMode::Goal::STAND_UP &&
        goal->fsm_id != SetLocoMode::Goal::START)
    {
        RCLCPP_WARN(
            get_logger(),
            "rejecting SetLocoMode goal: fsm_id %d is not one of DAMP(1)/STAND_UP(4)/START(500) "
            "-- Squat/Sit/ZeroTorque have no caller and unverified transition legality",
            goal->fsm_id);
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_goal_handle_)
    {
        RCLCPP_WARN(get_logger(), "rejecting SetLocoMode goal: a previous goal is still in flight");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
G1LocoBridge::handleCancel(const std::shared_ptr<GoalHandleSetLocoMode>& /*goal_handle*/)
{
    // Reject cancellation — goals are short-lived (bounded by request_timeout_s)
    // and unwind cleanly on teardown.
    return rclcpp_action::CancelResponse::REJECT;
}

void G1LocoBridge::handleAccepted(const std::shared_ptr<GoalHandleSetLocoMode>& goal_handle)
{
    const int fsm_id  = goal_handle->get_goal()->fsm_id;
    auto      request = correlator_.send(
        kApiIdSetFsmId,
        buildSetFsmIdPayload(fsm_id),
        std::chrono::steady_clock::now(),
        [this, goal_handle, fsm_id](std::int32_t error_code, const std::string& /*data*/) {
            onSetLocoModeResult(goal_handle, fsm_id, error_code);
        });
    if (!request)
    {
        auto result        = std::make_shared<SetLocoMode::Result>();
        result->success    = false;
        result->error_code = kCodeTaskUnknownError;
        result->message    = "too many LocoClient requests already in flight; try again";
        goal_handle->abort(result);
        return;
    }

    active_goal_handle_               = goal_handle;
    pending_set_loco_mode_request_id_ = request->header.identity.id;
    if (fsm_id == SetLocoMode::Goal::START)
    {
        velocity_gate_.beginAcquire();
    }
    else if (velocity_gate_.authority() == LocoAuthority::kHeld)
    {
        // Any transition away from Start means the robot is leaving the state
        // velocity authority depends on — release regardless of target (DAMP
        // or STAND_UP).
        velocity_gate_.beginRelease();
    }
    publishStatus();
    request_pub_->publish(*request);
}

void G1LocoBridge::onSetLocoModeResult(
    const std::shared_ptr<GoalHandleSetLocoMode>& goal_handle, int fsm_id, std::int32_t error_code)
{
    pending_set_loco_mode_request_id_.reset();

    // Self-gate on ACTIVE, same as cmdVelCallback/onReissueTick.
    const bool node_active =
        get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    const bool success = node_active && (error_code == kCodeSuccess);
    if (success)
    {
        last_known_fsm_id_ = fsm_id;
    }
    last_error_code_ = error_code;

    if (fsm_id == SetLocoMode::Goal::START)
    {
        velocity_gate_.onAcquireResult(success);
    }
    else
    {
        // Unconditional onReleaseResult(): harmless no-op when authority was
        // never kReleasing, prevents RELEASING from getting stuck.
        velocity_gate_.onReleaseResult();
    }

    if (active_goal_handle_ == goal_handle)
    {
        active_goal_handle_.reset();
    }
    publishStatus();

    auto result        = std::make_shared<SetLocoMode::Result>();
    result->success    = success;
    result->error_code = error_code;
    result->message    = success ? "fsm transition accepted" :
                                   ("fsm transition rejected: " + describeFsmRejection(error_code));
    if (success)
    {
        goal_handle->succeed(result);
    }
    else
    {
        goal_handle->abort(result);
    }
}

void G1LocoBridge::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        return;
    }
    const double vx =
        std::clamp(msg->linear.x * axis_sign_[0], -max_velocity_[0], max_velocity_[0]);
    const double vy =
        std::clamp(msg->linear.y * axis_sign_[1], -max_velocity_[1], max_velocity_[1]);
    const double vyaw =
        std::clamp(msg->angular.z * axis_sign_[2], -max_velocity_[2], max_velocity_[2]);

    const auto ignored_before = velocity_gate_.ignoredCommandCount();
    velocity_gate_.setCommand(vx, vy, vyaw, std::chrono::steady_clock::now());
    if (velocity_gate_.ignoredCommandCount() != ignored_before)
    {
        // A planner publishing into a gate that holds no authority is otherwise completely
        // silent: nothing moves, nothing errors. Name the remedy, because the two-goal
        // sequence is not guessable from the symptom.
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Discarding cmd_vel: locomotion authority is %d, not HELD. Acquire it with "
            "'ros2 action send_goal /g1_loco_bridge/set_mode g1_msgs/action/SetLocoMode "
            "\"{fsm_id: 4}\"' then the same with '{fsm_id: 500}'. Ignored %u so far.",
            static_cast<int>(velocity_gate_.authority()),
            velocity_gate_.ignoredCommandCount());
    }
}

void G1LocoBridge::onSportResponse(const unitree_api::msg::Response& msg)
{
    const auto dropped_before = correlator_.droppedResponseCount();
    correlator_.onResponse(msg);
    if (correlator_.droppedResponseCount() != dropped_before)
    {
        RCLCPP_DEBUG(
            get_logger(),
            "dropped an unmatched /api/sport/response (id %ld)",
            static_cast<long>(msg.header.identity.id));
    }
}

void G1LocoBridge::onReissueTick()
{
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        return;
    }
    const auto now    = std::chrono::steady_clock::now();
    const auto intent = velocity_gate_.tick(now);
    if (!intent)
    {
        return;
    }

    // Supersede stale requests, but feed the failure-streak safety net
    // a synthetic timeout first — otherwise a slow channel would never
    // trip the streak (supersede alone drops outcomes silently).
    if (pending_velocity_request_id_)
    {
        velocity_gate_.onVelocityResult(kCodeTaskTimeout);
        last_error_code_ = kCodeTaskTimeout;
        correlator_.supersede(*pending_velocity_request_id_);
        pending_velocity_request_id_.reset();
        publishStatus();
    }

    auto request = correlator_.send(
        kApiIdSetVelocity,
        buildSetVelocityPayload(
            static_cast<float>(intent->vx),
            static_cast<float>(intent->vy),
            static_cast<float>(intent->vyaw)),
        now,
        [this](std::int32_t error_code, const std::string& /*data*/) {
            pending_velocity_request_id_.reset();
            velocity_gate_.onVelocityResult(error_code);
            last_error_code_ = error_code;
            publishStatus();
        });
    if (!request)
    {
        RCLCPP_DEBUG(get_logger(), "skipped a SetVelocity re-issue: too many requests in flight");
        return;
    }
    pending_velocity_request_id_ = request->header.identity.id;
    request_pub_->publish(*request);
}

void G1LocoBridge::onHeartbeatTick()
{
    // GET_FSM_ID poll — keeps fsm_id authoritative from the robot, not just inferred.
    if (pending_fsm_poll_id_)
    {
        correlator_.supersede(*pending_fsm_poll_id_);
        pending_fsm_poll_id_.reset();
    }
    auto poll_request = correlator_.send(
        kApiIdGetFsmId,
        "",
        std::chrono::steady_clock::now(),
        [this](std::int32_t error_code, const std::string& data) {
            pending_fsm_poll_id_.reset();
            if (error_code != kCodeSuccess)
            {
                return;
            }
            const auto fsm_id = parseFsmIdResponse(data);
            if (fsm_id)
            {
                last_known_fsm_id_ = *fsm_id;
                publishStatus();
            }
        });
    if (poll_request)
    {
        pending_fsm_poll_id_ = poll_request->header.identity.id;
        request_pub_->publish(*poll_request);
    }

    // Advisory guard: stop and release authority if a second publisher appears.
    if (count_publishers(kSportRequestTopic) > 1)
    {
        RCLCPP_ERROR(get_logger(), "second publisher detected on /api/sport/request");
        if (velocity_gate_.authority() == LocoAuthority::kHeld)
        {
            auto stop_request = correlator_.send(
                kApiIdSetVelocity,
                buildSetVelocityPayload(0.0F, 0.0F, 0.0F),
                std::chrono::steady_clock::now(),
                [](std::int32_t, const std::string&) {});
            if (stop_request)
            {
                request_pub_->publish(*stop_request);
            }
            velocity_gate_.forceRelease();
            publishStatus();
        }
    }

    publishStatus(/*force=*/true);
}

void G1LocoBridge::publishStatus(bool force)
{
    g1_msgs::msg::LocoStatus msg;
    msg.stamp           = get_clock()->now();
    msg.fsm_id          = last_known_fsm_id_;
    msg.authority       = static_cast<std::uint8_t>(velocity_gate_.authority());
    msg.last_error_code = last_error_code_;
    msg.ignored_cmd_vel = velocity_gate_.ignoredCommandCount();

    // ignored_cmd_vel is deliberately absent from this comparison. It changes on every dropped
    // sample, so including it would publish at the publisher's rate on a transient-local topic
    // for as long as the drop lasts. The 1 Hz forced heartbeat surfaces it within a second.
    const bool changed = (msg.fsm_id != last_published_status_.fsm_id) ||
                         (msg.authority != last_published_status_.authority) ||
                         (msg.last_error_code != last_published_status_.last_error_code);
    if (!force && !changed)
    {
        return;
    }
    last_published_status_ = msg;
    status_pub_->publish(msg);
}

void G1LocoBridge::resetEntities()
{
    // Abort any in-flight goal before tearing down — once entities are destroyed,
    // nothing can fire the callback to resolve it. Abort while action_server_
    // still exists so the result reaches the client.
    if (active_goal_handle_)
    {
        auto result        = std::make_shared<SetLocoMode::Result>();
        result->success    = false;
        result->error_code = kCodeTaskUnknownError;
        result->message =
            "bridge is tearing down (cleanup/shutdown/error) while this goal was in flight";
        active_goal_handle_->abort(result);
    }
    correlator_.clear();

    sweep_timer_.reset();
    reissue_timer_.reset();
    heartbeat_timer_.reset();
    heartbeat_phase_timer_.reset();
    action_server_.reset();
    cmd_vel_sub_.reset();
    response_sub_.reset();
    status_pub_.reset();
    request_pub_.reset();
    active_goal_handle_.reset();
    pending_velocity_request_id_.reset();
    pending_fsm_poll_id_.reset();
    pending_set_loco_mode_request_id_.reset();
}

}  // namespace g1_locomotion
