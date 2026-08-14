/**
 * @file test_loco_bridge_node.cpp
 * @brief In-process node tests for G1LocoBridge: lifecycle/authority races that only exist once
 * DDS, the action server, and the correlator's timers are wired together on a live node -- the
 * pure-class tests (test_loco_correlator.cpp, test_velocity_gate.cpp) can't see these.
 */
#include <gmock/gmock.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "g1_locomotion/g1_loco_bridge_node.hpp"
#include "g1_locomotion/loco_api_ids.hpp"
#include "g1_msgs/action/set_loco_mode.hpp"
#include "g1_msgs/msg/loco_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace g1_locomotion
{
namespace
{

using namespace std::chrono_literals;
using SetLocoMode      = g1_msgs::action::SetLocoMode;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<SetLocoMode>;

/// Deep reader QoS so bursts in one DDS batch don't look like drops.
rclcpp::QoS sportReaderQos() { return rclcpp::QoS(10).reliable().durability_volatile(); }
rclcpp::QoS sportWriterQos() { return rclcpp::QoS(1).reliable().durability_volatile(); }
rclcpp::QoS statusReaderQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
}
rclcpp::QoS cmdVelWriterQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

/**
 * @brief Drives a G1LocoBridge in-process against a fake /api/sport/... responder: no sim, no
 * launch_testing, just DDS loopback on an isolated domain.
 * The fake responder answers SET_FSM_ID immediately and never answers SET_VELOCITY.
 */
class LocoBridgeNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Isolated domain to avoid collision with a live sim or concurrent tests.
        // Before any node or thread exists, so the thread-safety this warns about does not apply.
        // NOLINTNEXTLINE(concurrency-mt-unsafe)
        setenv("ROS_DOMAIN_ID", "67", 1);
        rclcpp::init(0, nullptr);

        // Constructed here because rclcpp::init() must run first.
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

        bridge_ = std::make_shared<G1LocoBridge>(rclcpp::NodeOptions());
        driver_ = std::make_shared<rclcpp::Node>("test_loco_bridge_driver");
        executor_->add_node(bridge_->get_node_base_interface());
        executor_->add_node(driver_);

        request_sub_ = driver_->create_subscription<unitree_api::msg::Request>(
            "/api/sport/request",
            sportReaderQos(),
            [this](const unitree_api::msg::Request::ConstSharedPtr& msg) { onRequest(*msg); });
        response_pub_ = driver_->create_publisher<unitree_api::msg::Response>(
            "/api/sport/response",
            sportWriterQos());
        status_sub_ = driver_->create_subscription<g1_msgs::msg::LocoStatus>(
            "/g1_loco_bridge/status",
            statusReaderQos(),
            [this](const g1_msgs::msg::LocoStatus::ConstSharedPtr& msg) { latest_status_ = *msg; });
        cmd_vel_pub_ = driver_->create_publisher<geometry_msgs::msg::Twist>(
            "/g1_loco_bridge/cmd_vel",
            cmdVelWriterQos());
        action_client_ =
            rclcpp_action::create_client<SetLocoMode>(driver_, "/g1_loco_bridge/set_mode");
    }

    void TearDown() override
    {
        action_client_.reset();
        request_sub_.reset();
        response_pub_.reset();
        status_sub_.reset();
        cmd_vel_pub_.reset();
        driver_.reset();
        bridge_.reset();
        executor_.reset();
        rclcpp::shutdown();
    }

    // --- spinning helpers ----------------------------------------------------------------------

    void spinFor(std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline)
        {
            executor_->spin_some(20ms);
            std::this_thread::sleep_for(2ms);
        }
    }

    /// Spins until `predicate` is true or `timeout` elapses; returns the final predicate value
    /// (so a caller can ASSERT on it without also having to re-check the timed-out case).
    template <typename Predicate>
    bool spinUntil(Predicate predicate, std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        do
        {
            if (predicate())
            {
                return true;
            }
            executor_->spin_some(20ms);
            std::this_thread::sleep_for(2ms);
        } while (std::chrono::steady_clock::now() < deadline);
        return predicate();
    }

    // --- bridge lifecycle ------------------------------------------------------------------------

    void configureAndActivate()
    {
        ASSERT_EQ(bridge_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
        ASSERT_TRUE(action_client_->wait_for_action_server(5s));
        ASSERT_EQ(bridge_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    }

    // --- fake /api/sport/* responder --------------------------------------------------------------

    void onRequest(const unitree_api::msg::Request& msg)
    {
        requests_.push_back(msg);
        if (auto_accept_fsm_ && msg.header.identity.api_id == kApiIdSetFsmId)
        {
            publishSportResult(msg.header.identity.id, kApiIdSetFsmId, kCodeSuccess);
        }
        else if (auto_accept_velocity_ && msg.header.identity.api_id == kApiIdSetVelocity)
        {
            publishSportResult(msg.header.identity.id, kApiIdSetVelocity, kCodeSuccess);
        }
    }

    void publishSetFsmIdResult(std::int64_t id, std::int32_t code)
    {
        publishSportResult(id, kApiIdSetFsmId, code);
    }

    void publishSportResult(std::int64_t id, std::int64_t api_id, std::int32_t code)
    {
        unitree_api::msg::Response response;
        response.header.identity.id     = id;
        response.header.identity.api_id = api_id;
        response.header.status.code     = code;
        response_pub_->publish(response);
    }

    std::size_t countRequests(std::int64_t api_id) const
    {
        return static_cast<std::size_t>(std::count_if(
            requests_.begin(),
            requests_.end(),
            [api_id](const unitree_api::msg::Request& req) {
                return req.header.identity.api_id == api_id;
            }));
    }

    std::optional<unitree_api::msg::Request> lastRequest(std::int64_t api_id) const
    {
        // std::ranges::reverse_view breaks clang-tidy's Clang-14 parser against libstdc++ here.
        // NOLINTNEXTLINE(modernize-loop-convert)
        for (auto it = requests_.rbegin(); it != requests_.rend(); ++it)
        {
            if (it->header.identity.api_id == api_id)
            {
                return *it;
            }
        }
        return std::nullopt;
    }

    // --- driving the bridge ------------------------------------------------------------------------

    ClientGoalHandle::SharedPtr
    sendSetLocoModeGoal(std::int32_t fsm_id, std::chrono::milliseconds timeout = 3s)
    {
        SetLocoMode::Goal goal;
        goal.fsm_id       = fsm_id;
        auto       future = action_client_->async_send_goal(goal);
        const auto ok     = spinUntil(
            [&future] {
                return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
            },
            timeout);
        return ok ? future.get() : nullptr;
    }

    std::optional<ClientGoalHandle::WrappedResult>
    waitForResult(const ClientGoalHandle::SharedPtr& goal_handle, std::chrono::milliseconds timeout)
    {
        auto       future = action_client_->async_get_result(goal_handle);
        const auto ok     = spinUntil(
            [&future] {
                return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
            },
            timeout);
        return ok ? std::make_optional(future.get()) : std::nullopt;
    }

    /// Publishes the same non-zero-vx Twist repeatedly for `duration` -- a single publish can
    /// race subscription discovery (mirrors g1_bringup/test/test_loco.launch.py's own pattern).
    void publishCmdVelFor(double vx, std::chrono::milliseconds duration)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x      = vx;
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline)
        {
            cmd_vel_pub_->publish(twist);
            executor_->spin_some(20ms);
            std::this_thread::sleep_for(20ms);
        }
    }

    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<G1LocoBridge>                              bridge_;
    std::shared_ptr<rclcpp::Node>                              driver_;

    rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr request_sub_;
    rclcpp::Publisher<unitree_api::msg::Response>::SharedPtr   response_pub_;
    rclcpp::Subscription<g1_msgs::msg::LocoStatus>::SharedPtr  status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_pub_;
    rclcpp_action::Client<SetLocoMode>::SharedPtr              action_client_;

    std::vector<unitree_api::msg::Request>  requests_;
    std::optional<g1_msgs::msg::LocoStatus> latest_status_;
    bool                                    auto_accept_fsm_{ true };
    /// Off by default: most tests leave SET_VELOCITY unanswered. Tests isolating
    /// a specific release path enable this to prevent unrelated failure-streak releases.
    bool auto_accept_velocity_{ false };
};

// -------------------------------------------------------------------------
// Blocker: authority must not survive a late SetLocoMode reply after deactivate
// -------------------------------------------------------------------------

TEST_F(LocoBridgeNodeTest, LateSetLocoModeReplyAfterDeactivateDoesNotReviveAuthority)
{
    auto_accept_fsm_ = false;  // control exactly when the SET_FSM_ID reply arrives
    configureAndActivate();

    auto goal_handle = sendSetLocoModeGoal(SetLocoMode::Goal::START);
    ASSERT_TRUE(goal_handle) << "START goal was not accepted";
    ASSERT_TRUE(spinUntil([this] { return countRequests(kApiIdSetFsmId) > 0; }, 2s))
        << "bridge never published the SET_FSM_ID request";
    ASSERT_TRUE(spinUntil(
        [this] {
            return latest_status_ &&
                   latest_status_->authority == g1_msgs::msg::LocoStatus::ACQUIRING;
        },
        1s))
        << "authority never reached ACQUIRING";
    const auto request_id = lastRequest(kApiIdSetFsmId)->header.identity.id;

    bridge_->deactivate();
    ASSERT_EQ(bridge_->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    // Reply delivered after deactivate — must not revive authority.
    publishSetFsmIdResult(request_id, kCodeSuccess);
    spinFor(200ms);

    latest_status_.reset();
    bridge_->activate();
    ASSERT_TRUE(spinUntil([this] { return latest_status_.has_value(); }, 1s))
        << "no ~/status observed after re-activating";
    EXPECT_EQ(latest_status_->authority, g1_msgs::msg::LocoStatus::RELEASED)
        << "a SetLocoMode(START) reply delivered after deactivate revived locomotion authority "
           "with no fresh acquire in this session";

    requests_.clear();
    publishCmdVelFor(0.1, 500ms);
    EXPECT_EQ(countRequests(kApiIdSetVelocity), 0U)
        << "cmd_vel produced SET_VELOCITY traffic despite authority never being acquired this "
           "session";
}

// -------------------------------------------------------------------------
// Blocker: a round trip slower than the re-issue period must still advance the failure streak
// -------------------------------------------------------------------------

TEST_F(LocoBridgeNodeTest, UnansweredVelocityRequestsStillReleaseAuthority)
{
    configureAndActivate();

    auto goal_handle = sendSetLocoModeGoal(SetLocoMode::Goal::START);
    ASSERT_TRUE(goal_handle) << "START goal was not accepted";
    const auto start_result = waitForResult(goal_handle, 2s);
    ASSERT_TRUE(start_result.has_value()) << "START result never arrived";
    ASSERT_EQ(start_result->code, rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_TRUE(spinUntil(
        [this] {
            return latest_status_ && latest_status_->authority == g1_msgs::msg::LocoStatus::HELD;
        },
        1s))
        << "authority never reached HELD";

    // Fake responder never answers SET_VELOCITY, simulating a slow channel.
    // Live cmd_vel must keep flowing to exercise the failure-streak path.
    publishCmdVelFor(0.1, 2s);

    ASSERT_TRUE(latest_status_.has_value());
    EXPECT_EQ(latest_status_->authority, g1_msgs::msg::LocoStatus::RELEASED)
        << "an unanswered SET_VELOCITY channel never released locomotion authority -- the "
           "failure-streak safety net stayed disabled the whole window";
    EXPECT_EQ(latest_status_->last_error_code, kCodeTaskTimeout)
        << "the released status did not record the synthetic timeout that should have caused it";
}

// -------------------------------------------------------------------------
// Major: STAND_UP from kHeld must release velocity authority, not just DAMP
// -------------------------------------------------------------------------

TEST_F(LocoBridgeNodeTest, StandUpFromHeldReleasesAuthorityAndStopsVelocityIntents)
{
    // Answer SET_VELOCITY to isolate STAND_UP's release path from failure-streak release.
    auto_accept_velocity_ = true;
    configureAndActivate();

    auto start_handle = sendSetLocoModeGoal(SetLocoMode::Goal::START);
    ASSERT_TRUE(start_handle) << "START goal was not accepted";
    const auto start_result = waitForResult(start_handle, 2s);
    ASSERT_TRUE(start_result.has_value()) << "START result never arrived";
    ASSERT_EQ(start_result->code, rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_TRUE(spinUntil(
        [this] {
            return latest_status_ && latest_status_->authority == g1_msgs::msg::LocoStatus::HELD;
        },
        1s))
        << "authority never reached HELD";

    // Sanity check first: confirm velocity traffic is actually flowing while HELD, so the later
    // "traffic stopped" assertion means something rather than being vacuously true.
    requests_.clear();
    publishCmdVelFor(0.1, 300ms);
    ASSERT_GT(countRequests(kApiIdSetVelocity), 0U)
        << "no SET_VELOCITY traffic observed while HELD";

    auto stand_up_handle = sendSetLocoModeGoal(SetLocoMode::Goal::STAND_UP);
    ASSERT_TRUE(stand_up_handle) << "STAND_UP goal was not accepted";
    const auto stand_up_result = waitForResult(stand_up_handle, 2s);
    ASSERT_TRUE(stand_up_result.has_value()) << "STAND_UP result never arrived";
    EXPECT_EQ(stand_up_result->code, rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_TRUE(spinUntil(
        [this] {
            return latest_status_ &&
                   latest_status_->authority == g1_msgs::msg::LocoStatus::RELEASED;
        },
        1s))
        << "STAND_UP accepted from kHeld did not release locomotion authority";

    requests_.clear();
    publishCmdVelFor(0.1, 500ms);
    EXPECT_EQ(countRequests(kApiIdSetVelocity), 0U)
        << "cmd_vel kept producing SET_VELOCITY intents after STAND_UP left kHeld -- the gate "
           "still believed it held velocity authority after the robot left Start";
}

// -------------------------------------------------------------------------
// Major: a goal in flight when the node tears down must terminate, not wedge
// -------------------------------------------------------------------------

TEST_F(LocoBridgeNodeTest, ShutdownWhileGoalInFlightTerminatesItInsteadOfWedging)
{
    auto_accept_fsm_ = false;  // keep the goal genuinely in flight -- no reply ever arrives
    configureAndActivate();

    bool                            result_received = false;
    ClientGoalHandle::WrappedResult result;
    SetLocoMode::Goal               goal;
    goal.fsm_id = SetLocoMode::Goal::START;
    rclcpp_action::Client<SetLocoMode>::SendGoalOptions options;
    options.result_callback = [&result_received,
                               &result](const ClientGoalHandle::WrappedResult& wrapped) {
        result_received = true;
        result          = wrapped;
    };
    auto goal_future = action_client_->async_send_goal(goal, options);
    ASSERT_TRUE(spinUntil(
        [&goal_future] {
            return goal_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        },
        3s));
    ASSERT_TRUE(goal_future.get()) << "START goal was not accepted";
    ASSERT_TRUE(spinUntil([this] { return countRequests(kApiIdSetFsmId) > 0; }, 2s))
        << "bridge never published the SET_FSM_ID request -- goal isn't genuinely in flight yet";
    // Extra spinning so the action server registers the result request.
    spinFor(200ms);

    // shutdown() bypasses on_deactivate entirely — this is the one path that
    // can reach resetEntities() with a goal still in flight.
    bridge_->shutdown();
    ASSERT_EQ(
        bridge_->get_current_state().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

    ASSERT_TRUE(spinUntil([&result_received] { return result_received; }, 1s))
        << "SetLocoMode goal was left hanging instead of being terminated by shutdown";
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
    ASSERT_TRUE(result.result);
    EXPECT_FALSE(result.result->success);
}

}  // namespace
}  // namespace g1_locomotion
