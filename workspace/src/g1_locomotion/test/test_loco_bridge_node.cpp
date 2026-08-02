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

/// History depth is not an RxO-matched QoS policy (only reliability/durability are) -- this
/// driver's own readers go deeper than the depth-1 vendor-matched wire QoS deliberately, so a
/// burst landing in one DDS write batch can never look like a drop that's actually just this test
/// under-provisioning its own subscription (see the Commit-3 fix a couple of these tests exercise
/// indirectly by counting requests).
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
 * @brief Drives a G1LocoBridge in-process against a fake /api/sport/* responder: no sim, no
 * launch_testing, just DDS loopback on an isolated domain.
 *
 * The fake responder (this fixture's own driver_ node) answers SET_FSM_ID immediately with
 * success by default -- enough to reach kHeld without needing motion_service_sim's own FSM
 * legality table, which is exercised separately by g1_bringup's test_loco_fsm/test_loco -- and
 * never answers SET_VELOCITY at all, since every test here cares about request traffic/authority
 * state rather than a velocity round trip actually completing.
 */
class LocoBridgeNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        /*
         * Isolated domain, the same methodology the review's own repro used: this driver_ node
         * publishes/subscribes the exact topic names (/api/sport/request, ~/cmd_vel, ...) a live
         * sim or a concurrently-running colcon-test package could also be using, so this process
         * must not share a domain with either. CycloneDDS's own container config pins the `lo`
         * interface, not a specific domain id, so any id is fine here.
         */
        setenv("ROS_DOMAIN_ID", "67", 1);
        rclcpp::init(0, nullptr);

        // Constructed here, not as a default-initialized member: gtest builds the fixture object
        // (default-initializing every member, including this one) before SetUp() ever runs, and
        // SingleThreadedExecutor's constructor needs rclcpp::init() to have already happened.
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
            publishSetFsmIdResult(msg.header.identity.id, kCodeSuccess);
        }
    }

    void publishSetFsmIdResult(std::int64_t id, std::int32_t code)
    {
        unitree_api::msg::Response response;
        response.header.identity.id     = id;
        response.header.identity.api_id = kApiIdSetFsmId;
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

    /*
     * The reply that would have promoted authority to HELD is delivered only now -- after
     * deactivate already ran -- mirroring the review's own repro ("reply 3 s later"). Whether it
     * ever arrives at all must not matter: on_deactivate()'s fix makes this unreachable regardless
     * of timing, not because the reply happens to lose a race against this test's own spinning.
     */
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

}  // namespace
}  // namespace g1_locomotion
