#ifndef G1_LOCOMOTION__G1_LOCO_BRIDGE_NODE_HPP_
#define G1_LOCOMOTION__G1_LOCO_BRIDGE_NODE_HPP_

/**
 * @file g1_loco_bridge_node.hpp
 * @brief LifecycleNode bridging geometry_msgs/Twist + g1_msgs/SetLocoMode onto Unitree's
 * LocoClient wire contract (/api/sport/request, /api/sport/response).
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "g1_locomotion/loco_request_correlator.hpp"
#include "g1_locomotion/velocity_gate.hpp"
#include "g1_msgs/action/set_loco_mode.hpp"
#include "g1_msgs/msg/loco_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace g1_locomotion
{

/**
 * @brief LocoClient bridge: translates `~/cmd_vel` and `~/set_mode` goals into the LocoClient
 * wire contract's async request/response exchange, without ever blocking an executor callback on
 * a DDS round trip (see the package README for why the vendored BaseClient can't be reused).
 *
 * @par Thread-ownership contract -- read before touching this class
 * Exactly one thread ever touches this node's state, but that guarantee is two-part, not one:
 * every callback source *this class itself creates* is placed without exception in ONE explicitly
 * created and named `MutuallyExclusive` callback group (`callback_group_`, built in the
 * constructor) -- the `/api/sport/response` subscription, the ~50 ms sweep timer, the velocity
 * re-issue timer, the ~1 Hz heartbeat/FSM-poll/rogue-guard timer, `~/cmd_vel`, and the
 * `~/set_mode` action server -- AND `main()` spins that group on a single
 * `rclcpp::executors::SingleThreadedExecutor` (see `g1_loco_bridge_main.cpp`). The group alone
 * does not cover everything: `rclcpp_lifecycle::LifecycleNode`'s own transition/parameter services
 * (`~/change_state`, `~/get_state`, `~/set_parameters`, ...) are created by the *base class*, land
 * in Humble's default callback group, and cannot be redirected -- so `on_configure`/`on_cleanup`/
 * `on_deactivate` (which reset `request_pub_`, `status_pub_`, and every timer) are only kept out
 * from under a concurrently running `onHeartbeatTick()`/etc. by `main()`'s executor being
 * single-threaded, not by `callback_group_` on its own. No locks, no atomics, anywhere in this
 * class, `LocoRequestCorrelator`, or `VelocityGate` -- moving one of *this class's own* callbacks
 * to a different group would need a deliberate, visible edit right here in `on_configure()`; a
 * genuine `MultiThreadedExecutor` migration would additionally need the base class's lifecycle
 * services serialised against `callback_group_` by some other means, since the group can't do that
 * part. Contrast this with g1_hardware_interface's G1ArmSdkSystem, which genuinely needs
 * `std::atomic`: its RT read()/write() thread (controller_manager's) and its own hidden executor
 * thread are two real, independently-scheduled threads by construction, so shared state there
 * truly is concurrent. Nothing here blocks: correlator sends are fire-and-forget publishes, and
 * every outcome -- success, failure, or timeout -- arrives later through this same executor as
 * another ordinary callback.
 */
class G1LocoBridge : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using SetLocoMode           = g1_msgs::action::SetLocoMode;
    using GoalHandleSetLocoMode = rclcpp_action::ServerGoalHandle<SetLocoMode>;

    explicit G1LocoBridge(const rclcpp::NodeOptions& options);

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

private:
    /// Reads+validates every parameter into the members below. velocity_reissue_hz <= 1.0 is
    /// fatal (returns false): duration's 1 s dead-man would expire between re-issues, silently
    /// defeating the whole point of re-issuing.
    bool readParameters();

    rclcpp_action::GoalResponse
    handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const SetLocoMode::Goal> goal);
    rclcpp_action::CancelResponse
         handleCancel(const std::shared_ptr<GoalHandleSetLocoMode>& goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandleSetLocoMode>& goal_handle);
    void onSetLocoModeResult(
        const std::shared_ptr<GoalHandleSetLocoMode>& goal_handle, int fsm_id,
        std::int32_t error_code);

    void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);
    void onSportResponse(const unitree_api::msg::Response& msg);

    void onReissueTick();
    void onHeartbeatTick();

    /// Publishes ~/status if `force` or if fsm_id/authority/last_error_code changed since the
    /// last publish -- the "on change + 1 Hz heartbeat" policy from the README's QoS table.
    void publishStatus(bool force = false);

    /// Idempotent teardown, safe to call from on_cleanup/on_shutdown/on_error and from the top of
    /// on_configure (which rebuilds everything from scratch every time it runs). "Safe" includes
    /// never wedging a caller: any SetLocoMode goal still in flight is aborted with a terminal
    /// result and the correlator's pending map is cleared before anything else is torn down (see
    /// the .cpp for why both are necessary), so no goal is ever left with nothing standing that
    /// could resolve it.
    void resetEntities();

    // Parameters (config/g1_loco_bridge.yaml).
    double                request_timeout_s_{ 5.0 };
    std::size_t           max_pending_{ 16 };
    double                velocity_reissue_hz_{ 5.0 };
    double                cmd_vel_timeout_s_{ 0.5 };
    int                   failure_streak_limit_{ 3 };
    std::array<double, 3> max_velocity_{ 0.8, 0.5, 1.57 };
    std::array<double, 3> axis_sign_{ 1.0, 1.0, 1.0 };

    // Pure engines -- see their own headers. Placeholder-constructed here; on_configure replaces
    // them with real, parameter-derived instances (mirrors G1ArmSdkSystem's ramp_engine_).
    LocoRequestCorrelator correlator_;
    VelocityGate          velocity_gate_;

    // Bridge-observed state surfaced via ~/status.
    int                      last_known_fsm_id_{ -1 };
    std::int32_t             last_error_code_{ 0 };
    g1_msgs::msg::LocoStatus last_published_status_;

    // At most one velocity request and one FSM-poll request in flight at a time -- a fresher
    // re-issue/poll supersedes whatever it replaces (see onReissueTick()/onHeartbeatTick()).
    std::optional<std::int64_t> pending_velocity_request_id_;
    std::optional<std::int64_t> pending_fsm_poll_id_;
    // At most one SetLocoMode goal in flight at a time -- handleGoal() rejects a new one while
    // this is set. pending_set_loco_mode_request_id_ tracks its correlator request id so
    // on_deactivate() can supersede that entry (and resetEntities() can clear it) before a late
    // reply can mutate authority state that no longer belongs to the goal it answers.
    std::shared_ptr<GoalHandleSetLocoMode> active_goal_handle_;
    std::optional<std::int64_t>            pending_set_loco_mode_request_id_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    // Both LifecyclePublisher, not rclcpp::Publisher: publish() is non-virtual in Humble, so a
    // base-typed handle would bypass the activation check entirely and let this node command the
    // wire while merely configured, never activated (see onHeartbeatTick()'s own guard, which
    // exists because count_publishers()/the FSM poll intentionally aren't gated the same way).
    rclcpp_lifecycle::LifecyclePublisher<g1_msgs::msg::LocoStatus>::SharedPtr  status_pub_;
    rclcpp_lifecycle::LifecyclePublisher<unitree_api::msg::Request>::SharedPtr request_pub_;
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr                response_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr                 cmd_vel_sub_;
    rclcpp_action::Server<SetLocoMode>::SharedPtr                              action_server_;
    rclcpp::TimerBase::SharedPtr                                               sweep_timer_;
    rclcpp::TimerBase::SharedPtr                                               reissue_timer_;
    rclcpp::TimerBase::SharedPtr                                               heartbeat_timer_;
    /// One-shot; fires once to phase-offset heartbeat_timer_'s first tick away from
    /// reissue_timer_'s, then cancels itself (see on_configure()). Tracked so resetEntities() can
    /// reset it too.
    rclcpp::TimerBase::SharedPtr heartbeat_phase_timer_;
};

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__G1_LOCO_BRIDGE_NODE_HPP_
