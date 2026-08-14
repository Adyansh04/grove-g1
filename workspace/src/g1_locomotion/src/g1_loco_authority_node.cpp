/**
 * @file g1_loco_authority_node.cpp
 * @brief Lifecycle bracket that acquires and releases LocoClient velocity authority.
 *
 * A planner publishes cmd_vel and nothing else -- it has no way to send the SetLocoMode goals
 * the bridge needs before it will act on anything. This node is that missing step, expressed
 * as a lifecycle transition so a lifecycle manager can bracket a whole navigation session:
 * active means the robot is walk-capable, inactive means authority has been handed back.
 *
 * Deliberately NOT auto-acquiring on the first cmd_vel. That is implicit acquisition, and a
 * stray publisher would stand the robot up and walk it (CONTROL_MODES.md rule 4).
 */
#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>

#include "g1_locomotion/loco_api_ids.hpp"
#include "g1_msgs/action/set_loco_mode.hpp"
#include "g1_msgs/msg/loco_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace g1_locomotion
{

class G1LocoAuthority : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using SetLocoMode = g1_msgs::action::SetLocoMode;

    explicit G1LocoAuthority(const rclcpp::NodeOptions& options)
      : rclcpp_lifecycle::LifecycleNode("g1_loco_authority", options)
    {
        declare_parameter("acquire_timeout_s", acquire_timeout_s_);
        declare_parameter("settle_after_start_s", settle_after_start_s_);
        // A parameter rather than a remap. Remapping an action name does not reach the client
        // created through rclcpp_action::create_client's node-interfaces overload -- verified
        // on Humble: the subscription below remaps correctly while the client kept resolving to
        // /set_mode either way, and the acquire then times out claiming the bridge is down.
        // A parameter cannot half-apply, and ros2 param get shows what it actually resolved to.
        declare_parameter("set_mode_action", action_name_);
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override
    {
        acquire_timeout_s_    = get_parameter("acquire_timeout_s").as_double();
        settle_after_start_s_ = get_parameter("settle_after_start_s").as_double();
        action_name_          = get_parameter("set_mode_action").as_string();
        if (action_name_.empty())
        {
            RCLCPP_ERROR(get_logger(), "set_mode_action must name the bridge's action");
            return CallbackReturn::FAILURE;
        }
        if (acquire_timeout_s_ <= 0.0 || settle_after_start_s_ < 0.0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "acquire_timeout_s must be positive and settle_after_start_s non-negative; got "
                "%.3f and %.3f",
                acquire_timeout_s_,
                settle_after_start_s_);
            return CallbackReturn::FAILURE;
        }

        // Reentrant, because on_activate blocks on an action result that this same node has to
        // service. See the executor note in the package README.
        callback_group_ =
            create_callback_group(rclcpp::CallbackGroupType::Reentrant, /*automatically_add=*/true);

        client_ = rclcpp_action::create_client<SetLocoMode>(
            get_node_base_interface(),
            get_node_graph_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            action_name_,
            callback_group_);

        // Transient-local and reliable, matched to the bridge's ~/status publisher. A mismatch
        // here is silent: no samples ever arrive and the acquire hangs to its timeout with
        // nothing in the log to say why.
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group_;
        status_sub_                = create_subscription<g1_msgs::msg::LocoStatus>(
            "status",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            [this](const g1_msgs::msg::LocoStatus::ConstSharedPtr& msg) {
                latest_authority_.store(msg->authority, std::memory_order_relaxed);
            },
            sub_options);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override
    {
        const auto base_result = LifecycleNode::on_activate(previous_state);
        if (base_result != CallbackReturn::SUCCESS)
        {
            return base_result;
        }
        RCLCPP_INFO(
            get_logger(),
            "acquiring locomotion authority (from %s)",
            previous_state.label().c_str());

        // Reactivation has to wait for a FRESH HELD. Left at whatever the previous cycle ended
        // on, waitForHeld() below can return on a stale value and skip the wait entirely.
        latest_authority_.store(g1_msgs::msg::LocoStatus::RELEASED, std::memory_order_relaxed);

        if (!client_->wait_for_action_server(timeout()))
        {
            RCLCPP_ERROR(
                get_logger(),
                "no SetLocoMode action server on '%s' within %.1f s; g1_loco_bridge is not up, "
                "or is not active",
                action_name_.c_str(),
                acquire_timeout_s_);
            return releaseAndFail();
        }
        if (!sendMode(SetLocoMode::Goal::STAND_UP, "STAND_UP"))
        {
            return releaseAndFail();
        }
        if (!sendMode(SetLocoMode::Goal::START, "START"))
        {
            return releaseAndFail();
        }
        // From here the bridge believes it holds authority, so every exit path owes a release.
        acquired_ = true;

        if (!waitForHeld())
        {
            RCLCPP_ERROR(
                get_logger(),
                "START succeeded but ~/status never reported HELD within %.1f s",
                acquire_timeout_s_);
            return releaseAndFail();
        }

        // The gait is not responsive the instant START returns. Measured across 8 fresh
        // launches: 1.83-1.92 s for six of seven that moved, deliberately not the p90.
        std::this_thread::sleep_for(std::chrono::duration<double>(settle_after_start_s_));

        RCLCPP_INFO(
            get_logger(),
            "locomotion authority held; the robot is walk-capable. Releasing on deactivate.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override
    {
        release();
        return LifecycleNode::on_deactivate(previous_state);
    }

    /// Humble permits shutdown() straight from ACTIVE, bypassing on_deactivate entirely -- the
    /// same trap the bridge's README documents. Without this, that path leaks authority.
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override
    {
        release();
        resetEntities();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const rclcpp_lifecycle::State&) override
    {
        release();
        resetEntities();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override
    {
        resetEntities();
        return CallbackReturn::SUCCESS;
    }

    /// Release from outside the lifecycle machinery, for the process-exit path. No-op unless
    /// authority is actually held, so calling it after a normal deactivate costs nothing.
    void releaseOnExit() { release(); }

private:
    std::chrono::nanoseconds timeout() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(acquire_timeout_s_));
    }

    /// Whether a failed attempt is worth repeating.
    enum class Attempt
    {
        kOk,
        kNotReadyYet,  ///< the stack is still coming up; the same call later should work
        kFatal,        ///< a real rejection; repeating it would just fail again
    };

    /**
     * @brief Sends one SetLocoMode goal and blocks on its result, retrying while the stack is
     * still coming up.
     *
     * Needed because everything launches at once: the bridge can be up and answering before the
     * simulator has stepped any physics, and the onboard controller then refuses the transition.
     * Waiting a fixed amount instead would be a guess about startup that this retry does not
     * have to make.
     *
     * Bounded by acquire_timeout_s overall, not per attempt. A rejection comes back in about a
     * millisecond, so the budget is spent almost entirely on the sleeps between tries.
     */
    bool sendMode(int fsm_id, const char* label)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout();
        Attempt    outcome  = Attempt::kNotReadyYet;
        int        attempts = 0;
        while (std::chrono::steady_clock::now() < deadline)
        {
            ++attempts;
            outcome = trySendMode(fsm_id, label);
            if (outcome == Attempt::kOk)
            {
                if (attempts > 1)
                {
                    RCLCPP_INFO(get_logger(), "%s accepted on attempt %d", label, attempts);
                }
                return true;
            }
            if (outcome == Attempt::kFatal)
            {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        RCLCPP_ERROR(
            get_logger(),
            "%s never became possible within %.1f s (%d attempts); the stack came up but the "
            "robot never reached a state that would accept it",
            label,
            acquire_timeout_s_,
            attempts);
        return false;
    }

    Attempt trySendMode(int fsm_id, const char* label)
    {
        auto goal    = SetLocoMode::Goal();
        goal.fsm_id  = fsm_id;
        auto pending = client_->async_send_goal(goal);
        if (pending.wait_for(timeout()) != std::future_status::ready)
        {
            RCLCPP_ERROR(get_logger(), "%s goal was never acknowledged", label);
            return Attempt::kFatal;
        }
        const auto& handle = pending.get();
        if (!handle)
        {
            // A goal rejection carries no code, so this cannot be classified from the wire. Of
            // the bridge's three reject reasons, two are transient (not yet active, a previous
            // goal still in flight) and the third -- an fsm_id outside DAMP/STAND_UP/START --
            // is unreachable from here, because those are the only two ids this node sends.
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "%s not accepted yet; the bridge is up but not ready. Retrying.",
                label);
            return Attempt::kNotReadyYet;
        }
        auto result = client_->async_get_result(handle);
        if (result.wait_for(timeout()) != std::future_status::ready)
        {
            RCLCPP_ERROR(get_logger(), "%s goal produced no result", label);
            return Attempt::kFatal;
        }
        const auto& wrapped = result.get();
        if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success)
        {
            return Attempt::kOk;
        }

        // Here the code IS meaningful, so retry only what is actually worth repeating.
        // 7301 says the onboard controller is not in a state that can service the call, which is
        // exactly "still coming up"; a sweep timeout is the same kind of transient. 7302 is the
        // controller's own transition table refusing the move, and no amount of waiting changes
        // that -- retrying it would just hide a real fault behind a timeout.
        const std::int32_t code = wrapped.result->error_code;
        const bool transient    = code == kCodeLocoStateNotAvailable || code == kCodeTaskTimeout;
        if (transient)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "%s refused with code %d (%s); retrying.",
                label,
                code,
                wrapped.result->message.c_str());
            return Attempt::kNotReadyYet;
        }
        RCLCPP_ERROR(
            get_logger(),
            "%s rejected with code %d (%s). Not retried: this is the controller's own transition "
            "table refusing, not a startup race.",
            label,
            code,
            wrapped.result->message.c_str());
        return Attempt::kFatal;
    }

    bool waitForHeld()
    {
        // Belt and braces: a successful START already implies the bridge moved to HELD. This
        // confirms it on the topic a test or an operator can actually observe, and closes the
        // window where the goal succeeded and the bridge then deactivated underneath us.
        const auto deadline = std::chrono::steady_clock::now() + timeout();
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (latest_authority_.load(std::memory_order_relaxed) == g1_msgs::msg::LocoStatus::HELD)
            {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    }

    /// StandUp, never Damp: StandUp leaves the robot balanced and upright, Damp drops it.
    void release()
    {
        // exchange, not a read-then-write: since the exit path releases on its own thread while
        // the executor still services a deactivate, two releases can arrive at once, and both
        // passing the guard would put two SetLocoMode goals in flight.
        if (!acquired_.exchange(false))
        {
            return;
        }
        if (!sendMode(SetLocoMode::Goal::STAND_UP, "release to STAND_UP"))
        {
            // Logged, not propagated. The bridge's own beginRelease/onReleaseResult lands in
            // kReleased either way, and refusing to deactivate would strand this node active
            // while still holding authority -- the exact opposite of what rule 4 asks for.
            RCLCPP_ERROR(
                get_logger(),
                "release did not confirm on the wire; the bridge releases authority regardless");
        }
    }

    /// on_activate returning FAILURE leaves the node inactive, which means on_deactivate never
    /// runs. Without releasing here, a failure after START would leave the bridge holding
    /// authority with nobody supervising it.
    CallbackReturn releaseAndFail()
    {
        release();
        return CallbackReturn::FAILURE;
    }

    void resetEntities()
    {
        status_sub_.reset();
        client_.reset();
    }

    std::string action_name_{ "/g1_loco_bridge/set_mode" };
    double      acquire_timeout_s_{ 5.0 };
    double      settle_after_start_s_{ 2.5 };
    /// Whether the bridge believes it holds authority because of us. Written from the transition
    /// thread and from the exit path's releaser thread, which can overlap.
    std::atomic<bool> acquired_{ false };
    /// The one piece of state shared across threads: written by the status callback on the
    /// reentrant group, read by the transition thread in waitForHeld().
    std::atomic<std::uint8_t> latest_authority_{ g1_msgs::msg::LocoStatus::RELEASED };

    rclcpp::CallbackGroup::SharedPtr                          callback_group_;
    rclcpp_action::Client<SetLocoMode>::SharedPtr             client_;
    rclcpp::Subscription<g1_msgs::msg::LocoStatus>::SharedPtr status_sub_;
};

}  // namespace g1_locomotion

namespace
{
// Mutable by necessity: a signal handler cannot capture, so a flag at namespace scope is the
// only way back into the process.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::atomic<bool> g_stopping{ false };
// The handler below only stores to this. That is async-signal-safe if and only if the store
// is lock-free; if it were not, the "just a flag" argument would smuggle a lock into the
// handler, which is the exact hazard the comment below is defending against.
static_assert(
    std::atomic<bool>::is_always_lock_free,
    "the signal handler stores to g_stopping and needs it lock-free");

/// Ctrl-C is the ordinary way a navigation session ends, and it must not leak authority.
///
/// rclcpp's own signal handler invalidates the context before spin() returns, which leaves
/// nothing able to send the release goal -- the node would exit from ACTIVE with the bridge
/// still in kHeld, holding authority with nobody supervising it.
///
/// Note that on_shutdown() does NOT cover this. It only runs for an explicit
/// TRANSITION_ACTIVE_SHUTDOWN, which neither launch_ros nor a signal ever issues.
///
/// The handler itself only stores a flag. executor->cancel() takes an rmw mutex and can throw,
/// neither of which is async-signal-safe: a signal delivered to a thread already inside the RMW
/// holding that mutex would deadlock in the handler, and the process would then survive SIGTERM
/// too and die to launch's SIGKILL still holding authority -- the exact leak this closes. A
/// watcher thread does the cancel instead, which is the same split rclcpp's own handler uses.
void onSignal(int) { g_stopping.store(true); }
}  // namespace

int main(int argc, char** argv)
{
    rclcpp::InitOptions init_options;
    init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_options);

    // Nothing below may escape uncaught: unwinding past main() does not guarantee the release
    // logic runs, which is this file's entire reason to exist past on_activate.
    try
    {
        // Two threads, not the hardware default: on_activate blocks on an action result that this
        // same node must service, so the result callback needs a thread the transition is not
        // holding. Two is exactly enough and bounds the deviation. See the package README.
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        auto node = std::make_shared<g1_locomotion::G1LocoAuthority>(rclcpp::NodeOptions());
        executor.add_node(node->get_node_base_interface());

        // Checked: a handler that failed to install means Ctrl-C kills the process with
        // authority still held, which is the leak the whole watcher/releaser dance below exists
        // to close.
        const auto previous_int  = std::signal(SIGINT, onSignal);
        const auto previous_term = std::signal(SIGTERM, onSignal);
        if (previous_int == SIG_ERR || previous_term == SIG_ERR)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "could not install the SIGINT/SIGTERM handlers -- an interrupt will now exit "
                "without releasing locomotion authority");
        }

        std::thread watcher([&executor] {
            while (!g_stopping.load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            executor.cancel();
        });

        try
        {
            executor.spin();
        }
        catch (const std::exception& e)
        {
            // A callback that throws must not skip the release below.
            RCLCPP_ERROR(node->get_logger(), "executor.spin() threw: %s", e.what());
        }
        g_stopping.store(true);  // covers spin() returning for any other reason, so watcher joins
        watcher.join();

        // The context is still valid here, so the release can actually reach the bridge. It runs on
        // its own thread because sendMode blocks on futures that only the executor can resolve.
        std::atomic<bool> released{ false };
        std::thread       releaser([&node, &released] {
            node->releaseOnExit();
            released.store(true);
        });
        // The loop stops pumping after 8 s; the join after it is deliberately unbounded. Detaching
        // instead would let the releaser touch the node while shutdown() tears the context down.
        // Without the executor pumping, the futures inside can only run out their own timeouts, so
        // this terminates -- at worst 2 x acquire_timeout_s.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
        while (!released.load() && std::chrono::steady_clock::now() < deadline)
        {
            executor.spin_some(std::chrono::milliseconds(50));
        }
        releaser.join();

        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("g1_loco_authority"), "fatal: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
