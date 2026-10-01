/**
 * @file test_nervros_executor.cpp
 * @brief The executor node against stand-in skill servers: one mission at a time, halts, stops,
 *        authority and what the hands hold.
 *
 * Everything is in one process on a private domain. Dry runs need no servers at all; the runs
 * that move something use stand-ins for the skill servers, a detector and the costmap services.
 */

#include <gmock/gmock.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <atomic>
#include <behaviortree_cpp/contrib/json.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <future>
#include <g1_msgs/action/approach_object.hpp>
#include <g1_msgs/action/pick.hpp>
#include <g1_msgs/action/place.hpp>
#include <g1_msgs/action/retreat.hpp>
#include <g1_msgs/action/set_arm_posture.hpp>
#include <g1_msgs/action/step_clear.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nervros_interfaces/action/execute_mission.hpp>
#include <nervros_interfaces/msg/heartbeat.hpp>
#include <nervros_interfaces/msg/robot_state.hpp>
#include <nervros_interfaces/srv/get_catalog.hpp>
#include <nervros_interfaces/srv/preview_mission.hpp>
#include <nervros_interfaces/srv/stop_all.hpp>
#include <nervros_interfaces/srv/teleop.hpp>
#include <nervros_interfaces/srv/validate_mission.hpp>
#include <numbers>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <thread>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_orchestration/authority_port.hpp"
#include "g1_orchestration/catalog.hpp"
#include "g1_orchestration/nervros_executor.hpp"
#include "g1_orchestration/sha256.hpp"

namespace
{

using ExecuteMission = nervros_interfaces::action::ExecuteMission;
using RobotState     = nervros_interfaces::msg::RobotState;
using namespace std::chrono_literals;

/// Counts how often the arms were taken and given back.
class FakeAuthority : public g1_orchestration::AuthorityPort
{
public:
    g1_orchestration::Acquired acquire() override
    {
        ++acquired;
        std::this_thread::sleep_for(std::chrono::milliseconds(acquire_delay_ms.load()));
        if (!acquire_ok)
        {
            return g1_orchestration::Acquired::kFailed;
        }
        return already_active ? g1_orchestration::Acquired::kAlready :
                                g1_orchestration::Acquired::kTaken;
    }
    void release() override { ++released; }

    std::atomic<int>  acquired{ 0 };
    std::atomic<int>  released{ 0 };
    std::atomic<bool> acquire_ok{ true };
    /// The arms were taken by someone else before the executor came.
    std::atomic<bool> already_active{ false };
    std::atomic<int>  acquire_delay_ms{ 0 };
};

/// A skill server that takes a while, then succeeds or fails as told, and honours a cancel.
template <typename ActionT>
class FakeServer
{
public:
    using Handle = rclcpp_action::ServerGoalHandle<ActionT>;

    FakeServer(const rclcpp::Node::SharedPtr& node, const std::string& name)
    {
        server_ = rclcpp_action::create_server<ActionT>(
            node,
            name,
            [](const rclcpp_action::GoalUUID&,
               const std::shared_ptr<const typename ActionT::Goal>&) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<Handle>&) {
                // A server that is busy elsewhere can turn a cancel down, and take the next one.
                return reject_cancels-- > 0 ? rclcpp_action::CancelResponse::REJECT :
                                              rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<Handle>& handle) {
                const std::lock_guard<std::mutex> lock(mutex_);
                threads_.emplace_back([this, handle] { execute(handle); });
            });
    }

    ~FakeServer()
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        threads_.clear();
    }

    std::atomic<int>  goals{ 0 };
    std::atomic<int>  cancels{ 0 };
    std::atomic<bool> succeeds{ true };
    std::atomic<int>  duration_ms{ 100 };
    /// False for a server that keeps going with a goal it has been asked to cancel.
    std::atomic<bool> honor_cancel{ true };
    /// How many cancel requests the server turns down before it takes one.
    std::atomic<int> reject_cancels{ 0 };
    std::string      message = "done";

    /// The `object_id` of the newest goal, for the actions that have one.
    std::string lastObjectId()
    {
        const std::lock_guard<std::mutex> lock(id_mutex_);
        return last_object_id_;
    }

private:
    void execute(const std::shared_ptr<Handle>& handle)
    {
        ++goals;
        if constexpr (requires { handle->get_goal()->object_id; })
        {
            const std::lock_guard<std::mutex> lock(id_mutex_);
            last_object_id_ = handle->get_goal()->object_id;
        }
        auto       result = std::make_shared<typename ActionT::Result>();
        const auto end =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms.load());
        while (std::chrono::steady_clock::now() < end)
        {
            if (honor_cancel && handle->is_canceling())
            {
                ++cancels;
                if constexpr (requires { result->success; })
                {
                    result->success = false;
                    result->message = "cancelled";
                }
                handle->canceled(result);
                return;
            }
            std::this_thread::sleep_for(10ms);
        }
        // Nav2's and the controller's results carry an error instead of success and a message.
        if constexpr (requires { result->success; })
        {
            result->success = succeeds;
            result->message = succeeds ? message : "phase: " + message;
        }
        if (succeeds)
        {
            handle->succeed(result);
        }
        else
        {
            handle->abort(result);
        }
    }

    typename rclcpp_action::Server<ActionT>::SharedPtr server_;
    std::mutex                                         mutex_;
    std::vector<std::jthread>                          threads_;
    std::mutex                                         id_mutex_;
    std::string                                        last_object_id_;
};

/// The detector's parameter, and /objects with whatever ids the test says are in view.
class FakeDetector
{
public:
    explicit FakeDetector(std::vector<std::string> visible)
      : visible_(std::move(visible))
    {
        node = std::make_shared<rclcpp::Node>("g1_detector");
        node->declare_parameter<std::vector<std::string>>("phrases", std::vector<std::string>{});
        callback_ = node->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& changed) {
                for (const auto& parameter : changed)
                {
                    if (parameter.get_name() == "phrases")
                    {
                        const std::lock_guard<std::mutex> lock(mutex_);
                        written_.push_back(parameter.as_string_array());
                    }
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });
        pub_ = node->create_publisher<vision_msgs::msg::Detection3DArray>(
            "/objects",
            rclcpp::QoS(1).reliable());
        timer_ = node->create_wall_timer(100ms, [this] {
            vision_msgs::msg::Detection3DArray msg;
            for (const auto& id : visible_)
            {
                msg.detections.emplace_back().id = id;
            }
            pub_->publish(msg);
        });
    }

    /// Every value `phrases` was set to, in order.
    std::vector<std::vector<std::string>> written()
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        return written_;
    }

    rclcpp::Node::SharedPtr node;

private:
    std::vector<std::string>                                          visible_;
    std::mutex                                                        mutex_;
    std::vector<std::vector<std::string>>                             written_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr  pub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;
};

std::string mission(const std::string& body)
{
    return R"(<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">)" +
           body + R"(</BehaviorTree>
</root>)";
}

const std::string kTuck = R"(<SubTree ID="TuckForTravel" name="s1_TuckForTravel"/>)";
const std::string kPick =
    R"(<SubTree ID="PickObject" name="s1_PickObject" object_id="red_block" phrase="bright red plastic ball" arm="right"/>)";
const std::string kPlace =
    R"(<SubTree ID="PlaceInto" name="s1_PlaceInto" container_id="brown_box" phrase="brown box container" arm="right"/>)";
const std::string kWalk = R"(<SubTree ID="GoToPose" name="s2_GoToPose" station="1.0;2.0;0.5"/>)";

nlohmann::json diagnostics(const ExecuteMission::Result& result)
{
    return nlohmann::json::parse(result.diagnostics_json);
}

/// Whether @p text is JSON, without throwing.
bool isJson(const std::string& text)
{
    return !nlohmann::json::parse(text, nullptr, /*allow_exceptions=*/false).is_discarded();
}

ExecuteMission::Goal makeGoal(
    const std::string& xml, std::uint8_t mode = ExecuteMission::Goal::MODE_EXECUTE,
    float max_s = 0.0F)
{
    ExecuteMission::Goal goal;
    goal.mission_id     = "test";
    goal.tree_xml       = xml;
    goal.tree_sha256    = g1_orchestration::sha256Hex(xml);
    goal.mode           = mode;
    goal.max_duration_s = max_s;
    return goal;
}

/// What one goal produced, as the client saw it.
struct Run
{
    bool                                  accepted = false;
    rclcpp_action::ResultCode             code     = rclcpp_action::ResultCode::UNKNOWN;
    ExecuteMission::Result::SharedPtr     result;
    std::vector<ExecuteMission::Feedback> feedback;
};

/// The executor node, spinning, with a client node beside it.
class Harness
{
public:
    explicit Harness(const std::vector<rclcpp::Parameter>& overrides = {})
    {
        authority = std::make_shared<FakeAuthority>();
        rclcpp::NodeOptions            options;
        std::vector<rclcpp::Parameter> parameters = {
            { "catalog_file", std::string(G1_CONFIG_DIR) + "/catalog.yaml" },
            { "macros_dir", std::string(G1_TREES_DIR) + "/library" },
            { "groot2_port", 0 },
            // No IMU or controller_manager here; the gate's own tests turn it on with fakes.
            { "health_gate", false },
            { "dry_run_time_scale", 0.0005 },
            { "cancel_settle_s", 0.1 },
            { "stop_wait_s", 1.5 },
            { "stop_action_servers",
              std::vector<std::string>{
                  "/navigate_to_pose:nav2_msgs/action/NavigateToPose",
                  "/spin:nav2_msgs/action/Spin",
                  "/g1_base_approach/approach_object:g1_msgs/action/ApproachObject",
                  "/g1_base_approach/retreat:g1_msgs/action/Retreat",
                  "/g1_base_approach/step_clear:g1_msgs/action/StepClear",
                  "/g1_manipulation_server/pick:g1_msgs/action/Pick",
                  "/g1_manipulation_server/place:g1_msgs/action/Place",
                  "/g1_manipulation_server/set_arm_posture:g1_msgs/action/SetArmPosture" } },
            { "arm_controller_action",
              "/arm_trajectory_controller/follow_joint_trajectory:control_msgs/action/"
              "FollowJointTrajectory" },
        };
        for (const auto& replacement : overrides)
        {
            std::erase_if(parameters, [&](const rclcpp::Parameter& p) {
                return p.get_name() == replacement.get_name();
            });
            parameters.push_back(replacement);
        }
        options.parameter_overrides(parameters);
        executor_node = std::make_shared<g1_orchestration::MissionExecutor>(authority, options);
        client_node   = std::make_shared<rclcpp::Node>("test_mission_client");
        spinner       = std::jthread([this] { spin(); });

        client = rclcpp_action::create_client<ExecuteMission>(
            client_node,
            "/nervros_executor/execute_mission");
        state_sub = client_node->create_subscription<RobotState>(
            "/nervros_executor/robot_state",
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const RobotState::ConstSharedPtr& msg) {
                const std::lock_guard<std::mutex> lock(state_mutex);
                states.push_back(*msg);
            });
        stop = client_node->create_client<nervros_interfaces::srv::StopAll>(
            "/nervros_executor/stop_all");
        validate = client_node->create_client<nervros_interfaces::srv::ValidateMission>(
            "/nervros_executor/validate_mission");
        catalog = client_node->create_client<nervros_interfaces::srv::GetCatalog>(
            "/nervros_executor/get_catalog");
        EXPECT_TRUE(client->wait_for_action_server(20s));
        EXPECT_TRUE(stop->wait_for_service(20s));
        EXPECT_TRUE(validate->wait_for_service(20s));
        EXPECT_TRUE(catalog->wait_for_service(20s));
    }

    ~Harness()
    {
        executor_node->shutdown();
        // Joined here: the executor below is destroyed before the thread member would be.
        executor_.cancel();
        spinner.join();
    }

    /// Sends a goal and returns before it finishes; `finished()` collects it.
    void start(const ExecuteMission::Goal& goal)
    {
        run_  = std::make_shared<Run>();
        done_ = false;
        rclcpp_action::Client<ExecuteMission>::SendGoalOptions options;
        options.goal_response_callback = [this](const auto& handle) {
            const std::lock_guard<std::mutex> lock(run_mutex_);
            run_->accepted = handle != nullptr;
            handle_        = handle;
        };
        options.feedback_callback =
            [this](const auto&, const std::shared_ptr<const ExecuteMission::Feedback>& feedback) {
                const std::lock_guard<std::mutex> lock(run_mutex_);
                run_->feedback.push_back(*feedback);
            };
        options.result_callback = [this](const auto& wrapped) {
            {
                const std::lock_guard<std::mutex> lock(run_mutex_);
                run_->code   = wrapped.code;
                run_->result = wrapped.result;
                done_        = true;
            }
            done_cv_.notify_all();
        };
        (void)client->async_send_goal(goal, options);
    }

    /// Waits for the goal started last to end.
    std::shared_ptr<Run> finished(std::chrono::seconds timeout = 60s)
    {
        std::unique_lock<std::mutex> lock(run_mutex_);
        EXPECT_TRUE(done_cv_.wait_for(lock, timeout, [this] { return done_.load(); }))
            << "the mission never ended";
        return run_;
    }

    std::shared_ptr<Run>
    execute(const ExecuteMission::Goal& goal, std::chrono::seconds timeout = 60s)
    {
        start(goal);
        return finished(timeout);
    }

    /// Waits until the running goal has sent feedback that names a running node.
    void waitUntilRunning()
    {
        const auto end = std::chrono::steady_clock::now() + 20s;
        while (std::chrono::steady_clock::now() < end)
        {
            {
                const std::lock_guard<std::mutex> lock(run_mutex_);
                if (std::ranges::any_of(run_->feedback, [](const auto& f) {
                        return !f.running_nodes.empty();
                    }))
                {
                    return;
                }
            }
            std::this_thread::sleep_for(20ms);
        }
        FAIL() << "the mission never started running";
    }

    void cancelRunning()
    {
        std::shared_ptr<rclcpp_action::ClientGoalHandle<ExecuteMission>> handle;
        {
            const std::lock_guard<std::mutex> lock(run_mutex_);
            handle = handle_;
        }
        ASSERT_NE(handle, nullptr);
        client->async_cancel_goal(handle);
    }

    nervros_interfaces::srv::StopAll::Response::SharedPtr stopAll(const std::string& reason)
    {
        auto request    = std::make_shared<nervros_interfaces::srv::StopAll::Request>();
        request->reason = reason;
        auto future     = stop->async_send_request(request);
        EXPECT_EQ(future.wait_for(30s), std::future_status::ready);
        return future.get();
    }

    RobotState latestState()
    {
        const auto end = std::chrono::steady_clock::now() + 5s;
        while (std::chrono::steady_clock::now() < end)
        {
            {
                const std::lock_guard<std::mutex> lock(state_mutex);
                if (!states.empty())
                {
                    return states.back();
                }
            }
            std::this_thread::sleep_for(20ms);
        }
        return RobotState();
    }

    /// Waits for the newest state to satisfy @p condition, since state arrives asynchronously.
    template <typename Predicate>
    RobotState stateWhere(Predicate condition)
    {
        const auto end = std::chrono::steady_clock::now() + 10s;
        RobotState last;
        while (std::chrono::steady_clock::now() < end)
        {
            last = latestState();
            if (condition(last))
            {
                return last;
            }
            std::this_thread::sleep_for(50ms);
        }
        return last;
    }

    std::shared_ptr<FakeAuthority>                                      authority;
    std::shared_ptr<g1_orchestration::MissionExecutor>                  executor_node;
    rclcpp::Node::SharedPtr                                             client_node;
    rclcpp_action::Client<ExecuteMission>::SharedPtr                    client;
    rclcpp::Client<nervros_interfaces::srv::StopAll>::SharedPtr         stop;
    rclcpp::Client<nervros_interfaces::srv::ValidateMission>::SharedPtr validate;
    rclcpp::Client<nervros_interfaces::srv::GetCatalog>::SharedPtr      catalog;
    rclcpp::Subscription<RobotState>::SharedPtr                         state_sub;
    std::mutex                                                          state_mutex;
    std::vector<RobotState>                                             states;
    std::jthread                                                        spinner;

private:
    void spin()
    {
        executor_.add_node(executor_node);
        executor_.add_node(client_node);
        executor_.spin();
    }

    rclcpp::executors::MultiThreadedExecutor executor_{ rclcpp::ExecutorOptions(), 6 };
    std::mutex                               run_mutex_;
    std::condition_variable                  done_cv_;
    std::atomic<bool>                        done_{ false };
    std::shared_ptr<Run>                     run_;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<ExecuteMission>> handle_;
};

/// The skill servers, detector and costmap services a PickObject or PlaceInto macro talks to.
class FakeRobot
{
public:
    /// @param navigation False leaves Nav2's server out, so a walk has nothing to send its goal to.
    explicit FakeRobot(bool navigation = true)
      : detector({ "red_block", "brown_box", "o17", "o31" })
    {
        node     = std::make_shared<rclcpp::Node>("fake_robot");
        approach = std::make_unique<FakeServer<g1_msgs::action::ApproachObject>>(
            node,
            "/g1_base_approach/approach_object");
        retreat = std::make_unique<FakeServer<g1_msgs::action::Retreat>>(
            node,
            "/g1_base_approach/retreat");
        pick = std::make_unique<FakeServer<g1_msgs::action::Pick>>(
            node,
            "/g1_manipulation_server/pick");
        place = std::make_unique<FakeServer<g1_msgs::action::Place>>(
            node,
            "/g1_manipulation_server/place");
        posture = std::make_unique<FakeServer<g1_msgs::action::SetArmPosture>>(
            node,
            "/g1_manipulation_server/set_arm_posture");
        step_clear = std::make_unique<FakeServer<g1_msgs::action::StepClear>>(
            node,
            "/g1_base_approach/step_clear");
        if (navigation)
        {
            navigate = std::make_unique<FakeServer<nav2_msgs::action::NavigateToPose>>(
                node,
                "/navigate_to_pose");
        }
        spin           = std::make_unique<FakeServer<nav2_msgs::action::Spin>>(node, "/spin");
        arm_controller = std::make_unique<FakeServer<control_msgs::action::FollowJointTrajectory>>(
            node,
            "/arm_trajectory_controller/follow_joint_trajectory");
        arm_controller->duration_ms = 20000;
        for (const char* name : { "/global_costmap/clear_entirely_global_costmap",
                                  "/local_costmap/clear_entirely_local_costmap" })
        {
            clears.push_back(node->create_service<nav2_msgs::srv::ClearEntireCostmap>(
                name,
                [](const nav2_msgs::srv::ClearEntireCostmap::Request::SharedPtr&,
                   const nav2_msgs::srv::ClearEntireCostmap::Response::SharedPtr&) {}));
        }
        executor_.add_node(node);
        executor_.add_node(detector.node);
        spinner_ = std::jthread([this] { executor_.spin(); });
    }

    ~FakeRobot() { executor_.cancel(); }

    rclcpp::Node::SharedPtr                                                     node;
    FakeDetector                                                                detector;
    std::unique_ptr<FakeServer<g1_msgs::action::ApproachObject>>                approach;
    std::unique_ptr<FakeServer<g1_msgs::action::Retreat>>                       retreat;
    std::unique_ptr<FakeServer<g1_msgs::action::Pick>>                          pick;
    std::unique_ptr<FakeServer<g1_msgs::action::Place>>                         place;
    std::unique_ptr<FakeServer<g1_msgs::action::SetArmPosture>>                 posture;
    std::unique_ptr<FakeServer<g1_msgs::action::StepClear>>                     step_clear;
    std::unique_ptr<FakeServer<nav2_msgs::action::NavigateToPose>>              navigate;
    std::unique_ptr<FakeServer<nav2_msgs::action::Spin>>                        spin;
    std::unique_ptr<FakeServer<control_msgs::action::FollowJointTrajectory>>    arm_controller;
    std::vector<rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr> clears;

private:
    rclcpp::executors::MultiThreadedExecutor executor_{ rclcpp::ExecutorOptions(), 4 };
    std::jthread                             spinner_;
};

template <typename Predicate>
bool waitFor(Predicate condition, std::chrono::seconds timeout = 20s)
{
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end)
    {
        if (condition())
        {
            return true;
        }
        std::this_thread::sleep_for(20ms);
    }
    return condition();
}

/// Sends @p goal through a client of its own, beside whatever the harness is running, and returns
/// what came of it.
ExecuteMission::Result::SharedPtr
sendOnAnotherClient(Harness& harness, const ExecuteMission::Goal& goal)
{
    auto other = rclcpp_action::create_client<ExecuteMission>(
        harness.client_node,
        "/nervros_executor/execute_mission");
    EXPECT_TRUE(other->wait_for_action_server(10s));
    std::promise<ExecuteMission::Result::SharedPtr>        answered;
    rclcpp_action::Client<ExecuteMission>::SendGoalOptions options;
    options.result_callback = [&answered](const auto& wrapped) {
        answered.set_value(wrapped.result);
    };
    (void)other->async_send_goal(goal, options);
    auto future = answered.get_future();
    EXPECT_EQ(future.wait_for(20s), std::future_status::ready);
    return future.get();
}

}  // namespace

// --- interfaces that need no robot --------------------------------------------------------------

TEST(NervrosExecutor, ServesTheCatalogItsMacrosAndItsVersion)
{
    Harness harness;
    auto    future = harness.catalog->async_send_request(
        std::make_shared<nervros_interfaces::srv::GetCatalog::Request>());
    ASSERT_EQ(future.wait_for(20s), std::future_status::ready);
    const auto response = future.get();

    const auto json = nlohmann::json::parse(response->catalog_json);
    EXPECT_EQ(json.at("catalog_version").get<std::string>(), response->catalog_version);
    EXPECT_EQ(json.at("skills").size(), 8U);
    EXPECT_THAT(response->tree_nodes_model_xml, ::testing::HasSubstr("<SubTree ID=\"PickObject\""));
    EXPECT_THAT(
        response->tree_nodes_model_xml,
        ::testing::HasSubstr("<SubTree ID=\"ExploreBuilding\""));
    EXPECT_THAT(response->tree_nodes_model_xml, ::testing::HasSubstr("<Action ID=\"Pick\""));
    EXPECT_THAT(response->bt_cpp_version, ::testing::StartsWith("4."));
    EXPECT_NE(
        response->catalog_version,
        g1_orchestration::Catalog::load(std::string(G1_CONFIG_DIR) + "/catalog.yaml").version())
        << "the version covers the palette served beside the catalog, not the file alone";
}

TEST(NervrosExecutor, ValidateMissionSaysWhyATreeIsRefusedAndHowLongAGoodOneTakes)
{
    Harness harness;

    auto good      = std::make_shared<nervros_interfaces::srv::ValidateMission::Request>();
    good->tree_xml = mission("<Sequence>" + kTuck + kWalk + "</Sequence>");
    auto answer    = harness.validate->async_send_request(good);
    ASSERT_EQ(answer.wait_for(20s), std::future_status::ready);
    const auto verdict = answer.get();
    EXPECT_TRUE(verdict->ok);
    EXPECT_EQ(verdict->tree_sha256, g1_orchestration::sha256Hex(good->tree_xml));
    EXPECT_FLOAT_EQ(verdict->worst_case_duration_s, 120.0F + 240.0F);
    EXPECT_EQ(verdict->diagnostics_json, "[]");

    auto bad      = std::make_shared<nervros_interfaces::srv::ValidateMission::Request>();
    bad->tree_xml = mission("<Parallel>" + kTuck + "</Parallel>");
    auto refusal  = harness.validate->async_send_request(bad);
    ASSERT_EQ(refusal.wait_for(20s), std::future_status::ready);
    const auto refused = refusal.get();
    EXPECT_FALSE(refused->ok);
    EXPECT_EQ(nlohmann::json::parse(refused->diagnostics_json).at(0).at("code"), "FORBIDDEN_NODE");
}

TEST(NervrosExecutor, RejectsAForbiddenMissionWithDiagnosticsAndRunsNothing)
{
    Harness harness;

    const auto run =
        harness.execute(makeGoal(mission("<ReactiveSequence>" + kTuck + "</ReactiveSequence>")));

    EXPECT_TRUE(run->accepted);
    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_EQ(diagnostics(*run->result).at(0).at("code"), "FORBIDDEN_NODE");
    EXPECT_FALSE(run->result->failure_reason.empty());
    EXPECT_EQ(harness.authority->acquired, 0) << "nothing was taken for a mission that was refused";
}

TEST(NervrosExecutor, RejectsAMissionWhoseHashIsNotTheApprovedOne)
{
    Harness harness;
    auto    goal = makeGoal(mission("<Sequence>" + kTuck + "</Sequence>"));

    goal.tree_sha256 = g1_orchestration::sha256Hex("something else");
    EXPECT_EQ(harness.execute(goal)->result->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    goal.tree_sha256 = "";
    const auto run   = harness.execute(goal);
    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_EQ(diagnostics(*run->result).at(0).at("code"), "HASH_MISMATCH");
}

TEST(NervrosExecutor, ValidateModeChecksAndLoadsWithoutRunning)
{
    Harness harness;

    const auto run = harness.execute(makeGoal(
        mission("<Sequence>" + kTuck + "</Sequence>"),
        ExecuteMission::Goal::MODE_VALIDATE));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS);
    EXPECT_EQ(harness.authority->acquired, 0);
    EXPECT_TRUE(run->feedback.empty()) << "nothing ticked";
}

TEST(NervrosExecutor, RefusesAModeItDoesNotKnow)
{
    Harness harness;
    auto    goal = makeGoal(mission("<Sequence>" + kTuck + "</Sequence>"));
    goal.mode    = 9;

    const auto run = harness.execute(goal);

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_EQ(diagnostics(*run->result).at(0).at("code"), "BAD_MODE");
}

// --- dry runs -------------------------------------------------------------------------------

TEST(NervrosExecutor, ADryRunWalksTheWholeMissionAndReportsItsStepsWithoutTouchingTheArms)
{
    Harness harness;

    const auto run = harness.execute(makeGoal(
        mission("<Sequence>" + kTuck + kWalk + kPick + "</Sequence>"),
        ExecuteMission::Goal::MODE_DRY_RUN));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << run->result->failure_reason;
    EXPECT_EQ(harness.authority->acquired, 0);
    EXPECT_EQ(harness.authority->released, 0);
    std::vector<std::string> steps;
    for (const auto& feedback : run->feedback)
    {
        for (const auto& event : feedback.events)
        {
            if (event.registration_id == "SubTree" && event.status == 1)
            {
                steps.push_back(event.name);
            }
        }
    }
    // Three steps, though two are named alike: the third is named s1_PickObject in kPick.
    EXPECT_THAT(steps, ::testing::Contains("s2_GoToPose"));
    EXPECT_THAT(steps, ::testing::Contains("s1_PickObject"));
    EXPECT_GT(run->result->elapsed_s, 0.0F);
}

TEST(NervrosExecutor, TheEventsNameEachNodeTheWayThePlannerNamedTheStep)
{
    Harness harness;

    const auto run = harness.execute(
        makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"), ExecuteMission::Goal::MODE_DRY_RUN));

    bool found_leaf = false;
    for (const auto& feedback : run->feedback)
    {
        for (const auto& event : feedback.events)
        {
            if (event.registration_id == "NavigateToPose")
            {
                found_leaf = true;
                EXPECT_EQ(event.path, "s2_GoToPose/walk_there");
                EXPECT_EQ(event.name, "walk_there");
            }
        }
    }
    EXPECT_TRUE(found_leaf) << "a skill leaf is an event at the steps level";
}

TEST(NervrosExecutor, EventsAtLevelAllIncludeEveryNode)
{
    Harness harness({ rclcpp::Parameter("event_level", "all") });

    const auto run = harness.execute(makeGoal(
        mission("<Sequence name=\"whole\">" + kWalk + "</Sequence>"),
        ExecuteMission::Goal::MODE_DRY_RUN));

    bool sequence = false;
    for (const auto& feedback : run->feedback)
    {
        sequence = sequence || std::ranges::any_of(feedback.events, [](const auto& e) {
                       return e.name == "whole";
                   });
    }
    EXPECT_TRUE(sequence);
}

TEST(NervrosExecutor, ARunningMissionRefusesASecondWithAReason)
{
    Harness harness({ rclcpp::Parameter("dry_run_time_scale", 0.02) });
    harness.start(
        makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"), ExecuteMission::Goal::MODE_DRY_RUN));
    harness.waitUntilRunning();

    // The harness follows one goal at a time; the second goes through a client of its own.
    const auto second =
        sendOnAnotherClient(harness, makeGoal(mission("<Sequence>" + kTuck + "</Sequence>")));

    EXPECT_EQ(second->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_EQ(diagnostics(*second).at(0).at("code"), "BUSY");
    EXPECT_THAT(second->failure_reason, ::testing::HasSubstr("already running"));
    harness.cancelRunning();
    harness.finished();
}

TEST(NervrosExecutor, ACancelHaltsTheMissionAndReportsItCancelled)
{
    Harness harness({ rclcpp::Parameter("dry_run_time_scale", 0.05) });
    harness.start(
        makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"), ExecuteMission::Goal::MODE_DRY_RUN));
    harness.waitUntilRunning();

    harness.cancelRunning();
    const auto run = harness.finished();

    EXPECT_EQ(run->code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_CANCELED);
    EXPECT_EQ(run->result->failed_step_id, "s2");
    EXPECT_LT(run->result->elapsed_s, 20.0F) << "it did not run to the end";
}

TEST(NervrosExecutor, TheWatchdogHaltsAMissionThatRunsPastItsLimit)
{
    Harness harness({ rclcpp::Parameter("dry_run_time_scale", 0.05) });

    const auto run = harness.execute(makeGoal(
        mission("<Sequence>" + kWalk + "</Sequence>"),
        ExecuteMission::Goal::MODE_DRY_RUN,
        1.0F));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_TIMEOUT);
    EXPECT_EQ(run->result->failed_step_id, "s2");
    EXPECT_THAT(run->result->failure_reason, ::testing::HasSubstr("limit"));
    EXPECT_GE(run->result->elapsed_s, 1.0F);
    EXPECT_LT(run->result->elapsed_s, 5.0F);
}

TEST(NervrosExecutor, TheWatchdogIsWhatTheGoalAskedForElseTheWorstCaseWithAMarginNeverPastTheCap)
{
    using g1_orchestration::MissionExecutor;
    const double nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_DOUBLE_EQ(MissionExecutor::watchdogLimitS(90.0, 500.0, 1.2, 1800.0), 90.0);
    EXPECT_DOUBLE_EQ(MissionExecutor::watchdogLimitS(0.0, 500.0, 1.2, 1800.0), 600.0);
    EXPECT_DOUBLE_EQ(MissionExecutor::watchdogLimitS(nan, 500.0, 1.2, 1800.0), 600.0);
    EXPECT_DOUBLE_EQ(MissionExecutor::watchdogLimitS(-3.0, 500.0, 1.2, 1800.0), 600.0);
    EXPECT_DOUBLE_EQ(MissionExecutor::watchdogLimitS(900.0, 100.0, 1.2, 300.0), 300.0)
        << "a goal cannot ask for more than the cap";
    EXPECT_DOUBLE_EQ(MissionExecutor::watchdogLimitS(0.0, 1000.0, 1.2, 300.0), 300.0);
    EXPECT_DOUBLE_EQ(MissionExecutor::watchdogLimitS(0.0, 0.0, 1.2, 300.0), 300.0)
        << "a mission with no worst case still has a limit";
}

// --- stopping ---------------------------------------------------------------------------------

TEST(NervrosExecutor, StopAllHaltsARunningMissionAndHoldsUntilTheNextOne)
{
    Harness harness({ rclcpp::Parameter("dry_run_time_scale", 0.05) });
    harness.start(
        makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"), ExecuteMission::Goal::MODE_DRY_RUN));
    harness.waitUntilRunning();

    const auto stopped = harness.stopAll("button");
    const auto run     = harness.finished();

    EXPECT_TRUE(stopped->ok) << stopped->message;
    EXPECT_THAT(stopped->state_after, ::testing::StartsWith("holding posture"));
    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_CANCELED);
    EXPECT_THAT(run->result->failure_reason, ::testing::HasSubstr("stopped: button"));
    const RobotState state =
        harness.stateWhere([](const RobotState& s) { return s.stopped && s.mission_id.empty(); });
    EXPECT_TRUE(state.stopped);
    EXPECT_THAT(state.message, ::testing::StartsWith("holding posture"));

    // A dry run does not lift the stop; the next real mission does.
    harness.execute(makeGoal(
        mission("<Sequence>" + kTuck + "</Sequence>"),
        ExecuteMission::Goal::MODE_DRY_RUN,
        0.3F));
    EXPECT_TRUE(harness.latestState().stopped);
}

TEST(NervrosExecutor, StopAllIsSafeWhenNothingRunsAndWhenCalledAgain)
{
    Harness harness;

    const auto first  = harness.stopAll("first");
    const auto second = harness.stopAll("second");

    EXPECT_TRUE(first->ok) << first->message;
    EXPECT_TRUE(second->ok) << second->message;
    EXPECT_EQ(first->state_after, second->state_after);
    EXPECT_THAT(first->state_after, ::testing::HasSubstr("hands empty"));
    EXPECT_TRUE(harness.stateWhere([](const RobotState& s) { return s.stopped; }).stopped);
}

TEST(NervrosExecutor, PublishesItsStateLatchedAndAtOneHertz)
{
    Harness harness;

    const RobotState first = harness.latestState();
    EXPECT_TRUE(first.mission_id.empty());
    EXPECT_FALSE(first.stopped);
    EXPECT_THAT(first.message, ::testing::StartsWith("idle"));
    std::this_thread::sleep_for(2500ms);
    const std::lock_guard<std::mutex> lock(harness.state_mutex);
    EXPECT_GE(harness.states.size(), 3U)
        << "the initial state and at least two of the periodic ones";
}

// --- against stand-in skill servers -----------------------------------------------------------

TEST(NervrosExecutor, TakesTheArmsOnceAndGivesThemBackWhenTheHandsAreEmpty)
{
    FakeRobot robot;
    Harness   harness;

    const auto run = harness.execute(makeGoal(mission("<Sequence>" + kTuck + "</Sequence>")));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << run->result->failure_reason;
    EXPECT_EQ(harness.authority->acquired, 1);
    EXPECT_EQ(harness.authority->released, 1);
    EXPECT_EQ(robot.posture->goals, 2) << "one tuck per arm";
    const RobotState state =
        harness.stateWhere([](const RobotState& s) { return s.resources_held.empty(); });
    EXPECT_THAT(state.resources_held, ::testing::IsEmpty());
}

TEST(NervrosExecutor, ADriveOnlyMissionTakesNoArmsAndLeavesAnyItDoesNotOwnAlone)
{
    FakeRobot robot;
    Harness   harness;

    const auto run = harness.execute(makeGoal(mission("<Sequence>" + kWalk + "</Sequence>")));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << run->result->failure_reason;
    EXPECT_EQ(robot.navigate->goals, 1);
    EXPECT_EQ(harness.authority->acquired, 0);
    EXPECT_EQ(harness.authority->released, 0);
    const RobotState during =
        harness.stateWhere([](const RobotState& s) { return s.mission_id.empty(); });
    EXPECT_THAT(during.resources_held, ::testing::IsEmpty());
}

TEST(NervrosExecutor, AStoppedWalkLeavesNothingRunningOnTheBase)
{
    FakeRobot robot;
    robot.navigate->duration_ms = 20000;
    Harness harness;
    harness.start(makeGoal(mission("<Sequence>" + kWalk + "</Sequence>")));
    const auto waiting = std::chrono::steady_clock::now() + 20s;
    while (robot.navigate->goals == 0 && std::chrono::steady_clock::now() < waiting)
    {
        std::this_thread::sleep_for(20ms);
    }
    ASSERT_EQ(robot.navigate->goals, 1);
    const RobotState walking =
        harness.stateWhere([](const RobotState& s) { return !s.mission_id.empty(); });
    EXPECT_THAT(walking.resources_held, ::testing::ElementsAre("base"));

    const auto stopped = harness.stopAll("watchdog");
    const auto run     = harness.finished();

    EXPECT_TRUE(stopped->ok) << stopped->message;
    EXPECT_EQ(robot.navigate->cancels, 1) << "the goal was cancelled, once";
    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_CANCELED);
    EXPECT_EQ(run->result->failed_step_id, "s2");
}

TEST(NervrosExecutor, StopAllCancelsTheArmControllersGoalSoTheArmHoldsWhereItIs)
{
    FakeRobot robot;
    Harness   harness;
    auto trajectory = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        harness.client_node,
        "/arm_trajectory_controller/follow_joint_trajectory");
    ASSERT_TRUE(trajectory->wait_for_action_server(10s));
    std::promise<rclcpp_action::ResultCode>                                             ended;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions options;
    options.result_callback = [&ended](const auto& wrapped) { ended.set_value(wrapped.code); };
    (void)trajectory->async_send_goal(control_msgs::action::FollowJointTrajectory::Goal(), options);
    const auto waiting = std::chrono::steady_clock::now() + 10s;
    while (robot.arm_controller->goals == 0 && std::chrono::steady_clock::now() < waiting)
    {
        std::this_thread::sleep_for(20ms);
    }
    ASSERT_EQ(robot.arm_controller->goals, 1);

    (void)harness.stopAll("button");

    auto future = ended.get_future();
    ASSERT_EQ(future.wait_for(10s), std::future_status::ready);
    EXPECT_EQ(future.get(), rclcpp_action::ResultCode::CANCELED);
}

TEST(NervrosExecutor, ReportsAFailureWithTheStepTheLeafAndTheServersOwnWords)
{
    FakeRobot robot;
    robot.pick->succeeds = false;
    robot.pick->message  = "no usable grasp";
    Harness harness;

    const auto run = harness.execute(makeGoal(mission("<Sequence>" + kPick + "</Sequence>")));

    EXPECT_EQ(run->code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_FAILURE);
    EXPECT_EQ(run->result->failed_step_id, "s1");
    EXPECT_EQ(run->result->failed_node, "s1_PickObject/pick_the_object");
    EXPECT_THAT(run->result->failure_reason, ::testing::HasSubstr("no usable grasp"));
    EXPECT_EQ(robot.pick->goals, 2) << "the macro retries a failed pick once";
    // The pick failed, so the hand is empty and the arms go back.
    EXPECT_EQ(harness.authority->acquired, 1);
    EXPECT_EQ(harness.authority->released, 1);
    EXPECT_TRUE(harness.stateWhere([](const RobotState& s) { return s.holding_right.empty(); })
                    .holding_right.empty());
}

TEST(NervrosExecutor, KeepsTheArmsWhileAHandHoldsAnObjectAndPlacesItInASecondMission)
{
    FakeRobot robot;
    Harness   harness;

    const auto picked = harness.execute(makeGoal(mission("<Sequence>" + kPick + "</Sequence>")));

    ASSERT_EQ(picked->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << picked->result->failure_reason;
    EXPECT_EQ(harness.authority->acquired, 1);
    EXPECT_EQ(harness.authority->released, 0)
        << "a hand holds the ball: letting go of the arm would drop it";
    RobotState state = harness.stateWhere(
        [](const RobotState& s) { return s.holding_right == "red_block" && s.mission_id.empty(); });
    EXPECT_EQ(state.holding_right, "red_block");
    EXPECT_TRUE(state.holding_left.empty());
    EXPECT_THAT(state.resources_held, ::testing::UnorderedElementsAre("left_arm", "right_arm"));
    EXPECT_THAT(state.message, ::testing::HasSubstr("right hand holds red_block"));

    const auto placed = harness.execute(makeGoal(mission("<Sequence>" + kPlace + "</Sequence>")));

    ASSERT_EQ(placed->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << placed->result->failure_reason;
    EXPECT_EQ(harness.authority->acquired, 1) << "already held: nothing to take again";
    EXPECT_EQ(harness.authority->released, 1);
    state = harness.stateWhere(
        [](const RobotState& s) { return s.holding_right.empty() && s.resources_held.empty(); });
    EXPECT_TRUE(state.holding_right.empty());
    EXPECT_THAT(state.resources_held, ::testing::IsEmpty());

    // Place has to see the held object again to confirm where it landed, so the detector was
    // asked for it alongside the container: the executor passed it on from the pick.
    const auto written = robot.detector.written();
    const auto asked   = std::ranges::any_of(written, [](const std::vector<std::string>& phrases) {
        return std::ranges::find(phrases, "red_block=bright red plastic ball") != phrases.end() &&
               std::ranges::find(phrases, "brown_box=brown box container") != phrases.end();
    });
    EXPECT_TRUE(asked);
}

TEST(NervrosExecutor, ARobotHoldsAWorldModelObjectUnderTheIdTheMissionGaveAndTheDetectorsOwn)
{
    FakeRobot         robot;
    Harness           harness;
    const std::string pick_o17 =
        R"(<SubTree ID="PickObject" name="s1_PickObject" object_id="O17" phrase="red mug" arm="right"/>)";
    const std::string place_o31 =
        R"(<SubTree ID="PlaceInto" name="s1_PlaceInto" container_id="O31" phrase="basket" arm="right"/>)";

    const auto picked = harness.execute(makeGoal(mission("<Sequence>" + pick_o17 + "</Sequence>")));

    ASSERT_EQ(picked->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << picked->result->failure_reason;
    EXPECT_EQ(robot.pick->lastObjectId(), "o17")
        << "the detector reports what it finds in lowercase";
    EXPECT_EQ(robot.approach->lastObjectId(), "o17");
    const RobotState state = harness.stateWhere(
        [](const RobotState& s) { return s.holding_right == "O17" && s.mission_id.empty(); });
    EXPECT_EQ(state.holding_right, "O17") << "the robot state speaks the mission's own ids";
    const auto stopped = harness.stopAll("chat");
    EXPECT_EQ(stopped->state_after, "holding posture; right hand keeps O17");

    const auto placed =
        harness.execute(makeGoal(mission("<Sequence>" + place_o31 + "</Sequence>")));

    ASSERT_EQ(placed->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << placed->result->failure_reason;
    EXPECT_EQ(robot.place->lastObjectId(), "") << "Place is given a surface, not an object";
    const auto written = robot.detector.written();
    const bool asked   = std::ranges::any_of(written, [](const std::vector<std::string>& phrases) {
        return std::ranges::find(phrases, "o17=red mug") != phrases.end() &&
               std::ranges::find(phrases, "O31=basket") != phrases.end();
    });
    EXPECT_TRUE(asked) << "the held object is asked for under the id Place will look it up by";
}

TEST(NervrosExecutor, PickPlaceAndTuckInOneMissionCarryWhatTheHandHoldsFromStepToStep)
{
    FakeRobot         robot;
    Harness           harness;
    const std::string place_step =
        R"(<SubTree ID="PlaceInto" name="s2_PlaceInto" container_id="brown_box" phrase="brown box container" arm="right"/>)";
    const std::string tuck_step = R"(<SubTree ID="TuckForTravel" name="s3_TuckForTravel"/>)";

    const auto run = harness.execute(
        makeGoal(mission("<Sequence>" + kPick + place_step + tuck_step + "</Sequence>")));

    ASSERT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << run->result->failure_reason;
    EXPECT_EQ(harness.authority->acquired, 1);
    EXPECT_EQ(harness.authority->released, 1);
    // Place fails unless the held object is on /objects after the release, so the place step has
    // to ask for it, though the pick that told the executor about it happened in this mission.
    const auto written = robot.detector.written();
    const bool asked   = std::ranges::any_of(written, [](const std::vector<std::string>& phrases) {
        return std::ranges::find(phrases, "red_block=bright red plastic ball") != phrases.end() &&
               std::ranges::find(phrases, "brown_box=brown box container") != phrases.end();
    });
    EXPECT_TRUE(asked) << "the place step did not look for the object in the hand";
    // And once it is placed the hand is empty, so the last step tucks both arms.
    EXPECT_EQ(robot.posture->goals, 2);
}

TEST(NervrosExecutor, StopAllWhileHoldingKeepsTheGripAndSaysSo)
{
    FakeRobot robot;
    Harness   harness;
    ASSERT_EQ(
        harness.execute(makeGoal(mission("<Sequence>" + kPick + "</Sequence>")))->result->outcome,
        ExecuteMission::Result::OUTCOME_SUCCESS);

    const auto stopped = harness.stopAll("chat");

    EXPECT_TRUE(stopped->ok) << stopped->message;
    EXPECT_EQ(stopped->state_after, "holding posture; right hand keeps red_block");
    EXPECT_EQ(harness.authority->released, 0) << "never let go of the arm while it holds something";
    const RobotState state = harness.stateWhere([](const RobotState& s) { return s.stopped; });
    EXPECT_EQ(state.holding_right, "red_block");
    EXPECT_THAT(state.resources_held, ::testing::UnorderedElementsAre("left_arm", "right_arm"));
}

TEST(NervrosExecutor, StopAllCancelsTheGoalsOfOtherClientsToo)
{
    FakeRobot robot;
    robot.retreat->duration_ms = 20000;
    Harness harness;
    // Someone else's goal on a server the leaves use.
    auto other = rclcpp_action::create_client<g1_msgs::action::Retreat>(
        harness.client_node,
        "/g1_base_approach/retreat");
    ASSERT_TRUE(other->wait_for_action_server(10s));
    g1_msgs::action::Retreat::Goal goal;
    goal.distance_m = 0.5;
    std::promise<rclcpp_action::ResultCode>                          ended;
    rclcpp_action::Client<g1_msgs::action::Retreat>::SendGoalOptions options;
    options.result_callback = [&ended](const auto& wrapped) { ended.set_value(wrapped.code); };
    (void)other->async_send_goal(goal, options);
    const auto waiting = std::chrono::steady_clock::now() + 10s;
    while (robot.retreat->goals == 0 && std::chrono::steady_clock::now() < waiting)
    {
        std::this_thread::sleep_for(20ms);
    }
    ASSERT_EQ(robot.retreat->goals, 1);

    const auto stopped = harness.stopAll("someone else's goal");

    auto future = ended.get_future();
    ASSERT_EQ(future.wait_for(10s), std::future_status::ready);
    EXPECT_EQ(future.get(), rclcpp_action::ResultCode::CANCELED);
    EXPECT_EQ(robot.retreat->cancels, 1);
    EXPECT_TRUE(stopped->ok) << stopped->message;
}

TEST(NervrosExecutor, FailsAMissionWhoseArmsCannotBeTaken)
{
    FakeRobot robot;
    Harness   harness;
    harness.authority->acquire_ok = false;

    const auto run = harness.execute(makeGoal(mission("<Sequence>" + kPick + "</Sequence>")));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_ERROR);
    EXPECT_THAT(run->result->failure_reason, ::testing::HasSubstr("arms"));
    EXPECT_EQ(robot.pick->goals, 0);
    EXPECT_EQ(harness.authority->released, 0) << "nothing was taken";
}

// --- input that is not what a planner would write ---------------------------------------------

TEST(NervrosExecutor, ValidateMissionAnswersForBytesThatAreNotUtf8)
{
    Harness harness;
    auto    request   = std::make_shared<nervros_interfaces::srv::ValidateMission::Request>();
    request->tree_xml = mission("<Sequence><\xff/></Sequence>");

    auto answer = harness.validate->async_send_request(request);
    ASSERT_EQ(answer.wait_for(20s), std::future_status::ready) << "the callback never answered";
    const auto verdict = answer.get();

    EXPECT_FALSE(verdict->ok);
    EXPECT_TRUE(isJson(verdict->diagnostics_json)) << verdict->diagnostics_json;
}

TEST(NervrosExecutor, ARefusalThatQuotesBytesThatAreNotUtf8IsStillSent)
{
    Harness harness;

    const auto run = harness.execute(makeGoal(mission("<Sequence><\xff/></Sequence>")));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_TRUE(isJson(run->result->diagnostics_json)) << run->result->diagnostics_json;
}

TEST(NervrosExecutor, ABusyRefusalNamesTheRunningMissionEvenWhenItsIdIsNotUtf8)
{
    Harness harness({ rclcpp::Parameter("dry_run_time_scale", 0.02) });
    auto    first =
        makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"), ExecuteMission::Goal::MODE_DRY_RUN);
    first.mission_id = "\xff";
    harness.start(first);
    harness.waitUntilRunning();

    const auto second =
        sendOnAnotherClient(harness, makeGoal(mission("<Sequence>" + kTuck + "</Sequence>")));

    EXPECT_EQ(second->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_EQ(diagnostics(*second).at(0).at("code"), "BUSY");
    harness.cancelRunning();
    harness.finished();
}

// --- what a mission may cost ------------------------------------------------------------------

TEST(NervrosExecutor, TurnsAwayAMissionThatCouldRunPastTheCapAndRunsOneThatFits)
{
    Harness harness({ rclcpp::Parameter("max_duration_cap_s", 300.0) });

    // 240 s for the walk and 120 s for the tuck: 360 s in the worst case, past the 300 s cap.
    const auto over =
        harness.execute(makeGoal(mission("<Sequence>" + kTuck + kWalk + "</Sequence>")));
    EXPECT_EQ(over->result->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_EQ(diagnostics(*over->result).at(0).at("code"), "TOO_LONG");
    EXPECT_EQ(harness.authority->acquired, 0) << "nothing was taken for a mission that was refused";

    auto request      = std::make_shared<nervros_interfaces::srv::ValidateMission::Request>();
    request->tree_xml = mission("<Sequence>" + kTuck + kWalk + "</Sequence>");
    auto answer       = harness.validate->async_send_request(request);
    ASSERT_EQ(answer.wait_for(20s), std::future_status::ready);
    EXPECT_FALSE(answer.get()->ok) << "ValidateMission says the same";

    const auto fits = harness.execute(
        makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"), ExecuteMission::Goal::MODE_DRY_RUN));
    EXPECT_EQ(fits->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << fits->result->failure_reason;
}

TEST(NervrosExecutor, TurnsAwayAMissionWithMoreStepsThanItWillLoad)
{
    Harness           harness({ rclcpp::Parameter("max_tree_steps", 2) });
    const std::string three =
        R"(<Sequence><SubTree ID="TuckForTravel" name="s1_Tuck"/><SubTree ID="TuckForTravel" name="s2_Tuck"/><SubTree ID="TuckForTravel" name="s3_Tuck"/></Sequence>)";

    const auto run = harness.execute(makeGoal(mission(three), ExecuteMission::Goal::MODE_VALIDATE));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_EQ(diagnostics(*run->result).at(0).at("code"), "TOO_MANY_STEPS");
}

// --- a tick that would otherwise not come back ------------------------------------------------

TEST(NervrosExecutor, AStopEndsAMissionThatIsWaitingToStartALeaf)
{
    FakeRobot robot(/*navigation=*/false);
    Harness   harness;
    harness.start(makeGoal(mission("<Sequence>" + kWalk + "</Sequence>")));
    harness.waitUntilRunning();
    // The walk's last leaf is waiting for a Nav2 that is not there, for up to 10 s in one tick.
    std::this_thread::sleep_for(1s);

    const auto stopped = harness.stopAll("button");
    const auto run     = harness.finished();

    EXPECT_TRUE(stopped->ok) << stopped->message;
    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_CANCELED);
    EXPECT_THAT(run->result->failure_reason, ::testing::HasSubstr("stopped: button"));
    EXPECT_LT(run->result->elapsed_s, 5.0F) << "it did not sit out the wait, or the retries";
}

TEST(NervrosExecutor, ACancelEndsAMissionThatIsWaitingToStartALeaf)
{
    FakeRobot robot(/*navigation=*/false);
    Harness   harness;
    // Retried three times over, around a walk that has nowhere to go.
    harness.start(makeGoal(mission(
        R"(<RetryUntilSuccessful num_attempts="3"><SubTree ID="GoToPose" name="s1_GoToPose" station="90.0;90.0;0.0"/></RetryUntilSuccessful>)")));
    harness.waitUntilRunning();
    std::this_thread::sleep_for(1s);

    harness.cancelRunning();
    const auto run = harness.finished();

    EXPECT_EQ(run->code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_CANCELED);
    EXPECT_LT(run->result->elapsed_s, 5.0F);
}

TEST(NervrosExecutor, TheWatchdogEndsAMissionThatIsWaitingToStartALeafToo)
{
    FakeRobot robot(/*navigation=*/false);
    Harness   harness;

    const auto run = harness.execute(makeGoal(
        mission("<Sequence>" + kWalk + "</Sequence>"),
        ExecuteMission::Goal::MODE_EXECUTE,
        1.0F));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_TIMEOUT)
        << run->result->failure_reason;
    EXPECT_LT(run->result->elapsed_s, 5.0F) << "not the 10 s of the wait";
}

// --- StopAll says what it did -----------------------------------------------------------------

TEST(NervrosExecutor, StopAllSaysWhichServersItCouldAsk)
{
    Harness harness;  // no robot: none of the servers is up to be asked

    const auto stopped = harness.stopAll("nothing there");

    EXPECT_TRUE(stopped->ok) << stopped->message;
    EXPECT_THAT(stopped->message, ::testing::HasSubstr("no mission was running"));
    EXPECT_THAT(
        stopped->message,
        ::testing::HasSubstr("cancel sent to 0 of 8 skill action servers"));
    EXPECT_THAT(stopped->message, ::testing::HasSubstr("arm controller has no action server up"));
    EXPECT_THAT(stopped->message, ::testing::Not(::testing::HasSubstr("was halted")));
}

TEST(NervrosExecutor, StopAllWithTheServersUpSaysItAskedEachOfThem)
{
    FakeRobot robot;
    Harness   harness;

    const auto stopped = harness.stopAll("button");

    EXPECT_TRUE(stopped->ok) << stopped->message;
    EXPECT_THAT(
        stopped->message,
        ::testing::HasSubstr("cancel sent to 8 of 8 skill action servers"));
    EXPECT_THAT(stopped->message, ::testing::HasSubstr("arm controller was asked to cancel"));
}

TEST(NervrosExecutor, StopAllWithNothingFollowedDoesNotClaimToHaveCancelledAnything)
{
    Harness harness({ rclcpp::Parameter("stop_action_servers", std::vector<std::string>{}),
                      rclcpp::Parameter("arm_controller_action", "") });

    const auto stopped = harness.stopAll("button");

    EXPECT_TRUE(stopped->ok) << stopped->message;
    EXPECT_THAT(stopped->message, ::testing::HasSubstr("no skill action servers are followed"));
    EXPECT_THAT(stopped->message, ::testing::HasSubstr("arm controller is not followed"));
    EXPECT_THAT(stopped->message, ::testing::Not(::testing::HasSubstr("cancel sent")));
}

TEST(NervrosExecutor, StopAllSaysWhenTheMissionThreadHasNotHaltedYet)
{
    FakeRobot robot;
    Harness   harness({ rclcpp::Parameter("stop_wait_s", 0.5) });
    // The mission thread is inside a slow controller_manager call, which no stop can interrupt.
    harness.authority->acquire_delay_ms = 2500;
    harness.start(makeGoal(mission("<Sequence>" + kTuck + "</Sequence>")));
    ASSERT_TRUE(waitFor([&] { return harness.authority->acquired == 1; }));

    const auto stopped = harness.stopAll("button");

    EXPECT_FALSE(stopped->ok);
    EXPECT_THAT(stopped->message, ::testing::HasSubstr("the mission thread has not halted yet"));
    EXPECT_THAT(stopped->message, ::testing::Not(::testing::HasSubstr("was halted")));
    const auto run = harness.finished();
    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_CANCELED);
    EXPECT_THAT(run->result->failure_reason, ::testing::HasSubstr("stopped: button"));
    EXPECT_EQ(robot.posture->goals, 0) << "it never got to start a leaf";
}

TEST(NervrosExecutor, StopAllAlsoWaitsForTheArmControllerAndSaysWhenItWillNotStop)
{
    FakeRobot robot;
    robot.arm_controller->honor_cancel = false;
    robot.arm_controller->duration_ms  = 3000;
    Harness harness;
    auto    trajectory = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        harness.client_node,
        "/arm_trajectory_controller/follow_joint_trajectory");
    ASSERT_TRUE(trajectory->wait_for_action_server(10s));
    std::promise<rclcpp_action::ResultCode>                                             ended;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions options;
    options.result_callback = [&ended](const auto& wrapped) { ended.set_value(wrapped.code); };
    (void)trajectory->async_send_goal(control_msgs::action::FollowJointTrajectory::Goal(), options);
    ASSERT_TRUE(waitFor([&] { return robot.arm_controller->goals == 1; }));

    const auto stopped = harness.stopAll("button");

    EXPECT_FALSE(stopped->ok) << "the arm controller still has its goal";
    EXPECT_THAT(stopped->message, ::testing::HasSubstr("still winding down"));
    EXPECT_THAT(stopped->message, ::testing::HasSubstr("follow_joint_trajectory"));
    auto future = ended.get_future();
    ASSERT_EQ(future.wait_for(20s), std::future_status::ready);
}

TEST(NervrosExecutor, AHaltAsksAgainForAGoalThatAServerWouldNotCancelTheFirstTime)
{
    FakeRobot robot;
    robot.navigate->duration_ms    = 20000;
    robot.navigate->reject_cancels = 1;
    Harness harness;
    harness.start(makeGoal(mission("<Sequence>" + kWalk + "</Sequence>")));
    ASSERT_TRUE(waitFor([&] { return robot.navigate->goals == 1; }));

    harness.cancelRunning();
    const auto run = harness.finished();

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_CANCELED);
    EXPECT_THAT(run->result->failure_reason, ::testing::Not(::testing::HasSubstr("winding down")))
        << "the leaf's own cancel was turned down and the second one was not";
    EXPECT_EQ(robot.navigate->cancels, 1);
}

TEST(NervrosExecutor, StopAllAsksAgainForAGoalThatAServerWouldNotCancelTheFirstTime)
{
    FakeRobot robot;
    robot.retreat->duration_ms    = 20000;
    robot.retreat->reject_cancels = 1;
    Harness harness;
    auto    other = rclcpp_action::create_client<g1_msgs::action::Retreat>(
        harness.client_node,
        "/g1_base_approach/retreat");
    ASSERT_TRUE(other->wait_for_action_server(10s));
    g1_msgs::action::Retreat::Goal goal;
    goal.distance_m = 0.5;
    std::promise<rclcpp_action::ResultCode>                          ended;
    rclcpp_action::Client<g1_msgs::action::Retreat>::SendGoalOptions options;
    options.result_callback = [&ended](const auto& wrapped) { ended.set_value(wrapped.code); };
    (void)other->async_send_goal(goal, options);
    ASSERT_TRUE(waitFor([&] { return robot.retreat->goals == 1; }));

    const auto stopped = harness.stopAll("someone else's goal");

    auto future = ended.get_future();
    ASSERT_EQ(future.wait_for(10s), std::future_status::ready);
    EXPECT_EQ(future.get(), rclcpp_action::ResultCode::CANCELED);
    EXPECT_TRUE(stopped->ok) << stopped->message;
}

// --- what a hand may hold ---------------------------------------------------------------------

TEST(NervrosExecutor, ArmsThatWereAlreadyTakenAreKeptAndTheirHandsAreNotTrusted)
{
    FakeRobot robot;
    Harness   harness;
    harness.authority->already_active = true;

    const auto tucked = harness.execute(makeGoal(mission("<Sequence>" + kTuck + "</Sequence>")));

    ASSERT_EQ(tucked->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << tucked->result->failure_reason;
    EXPECT_EQ(robot.posture->goals, 0) << "an arm whose hand may be full is not folded in";
    EXPECT_EQ(harness.authority->released, 0) << "letting go of the arms could drop an object";
    const RobotState state = harness.stateWhere(
        [](const RobotState& s) { return s.mission_id.empty() && !s.resources_held.empty(); });
    EXPECT_THAT(state.resources_held, ::testing::UnorderedElementsAre("left_arm", "right_arm"));
    EXPECT_THAT(state.message, ::testing::HasSubstr("left hand unknown"));
    EXPECT_THAT(state.message, ::testing::HasSubstr("right hand unknown"));
    EXPECT_TRUE(state.holding_left.empty());
    EXPECT_TRUE(state.holding_right.empty()) << "no id is made up for what is not known";
    EXPECT_THAT(harness.stopAll("chat")->state_after, ::testing::HasSubstr("right hand unknown"));

    // And it does not pick with a hand that may be full.
    const auto pick = harness.execute(makeGoal(mission("<Sequence>" + kPick + "</Sequence>")));
    EXPECT_EQ(pick->result->outcome, ExecuteMission::Result::OUTCOME_FAILURE);
    EXPECT_EQ(pick->result->failed_node, "s1_PickObject/hand_must_be_empty_to_pick");
    EXPECT_EQ(robot.pick->goals, 0);
    EXPECT_EQ(harness.authority->acquired, 2)
        << "the arms are looked at again while a hand may be full, in case they were let go of";
}

TEST(NervrosExecutor, HandsEmptyOnAttachTrustsTheHandsOfArmsThatWereAlreadyTaken)
{
    FakeRobot robot;
    Harness   harness({ rclcpp::Parameter("hands_empty_on_attach", true) });
    harness.authority->already_active = true;

    const auto tucked = harness.execute(makeGoal(mission("<Sequence>" + kTuck + "</Sequence>")));

    ASSERT_EQ(tucked->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << tucked->result->failure_reason;
    EXPECT_EQ(robot.posture->goals, 2);
    EXPECT_EQ(harness.authority->released, 1);
}

TEST(NervrosExecutor, APlaceClearsTheHandThatDidItAndTheArmsGoOnceBothHandsAreKnown)
{
    FakeRobot robot;
    Harness   harness;
    harness.authority->already_active = true;
    ASSERT_EQ(
        harness.execute(makeGoal(mission("<Sequence>" + kTuck + "</Sequence>")))->result->outcome,
        ExecuteMission::Result::OUTCOME_SUCCESS);
    const std::string place_left =
        R"(<SubTree ID="PlaceInto" name="s1_PlaceInto" container_id="brown_box" phrase="brown box container" arm="left"/>)";

    const auto placed = harness.execute(makeGoal(mission("<Sequence>" + kPlace + "</Sequence>")));

    ASSERT_EQ(placed->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << placed->result->failure_reason;
    EXPECT_EQ(robot.place->goals, 1);
    EXPECT_EQ(harness.authority->released, 0) << "the left hand is still not known to be empty";
    const RobotState state = harness.stateWhere([](const RobotState& s) {
        return s.mission_id.empty() && s.message.find("right hand unknown") == std::string::npos;
    });
    EXPECT_THAT(state.message, ::testing::HasSubstr("left hand unknown"));
    EXPECT_THAT(state.message, ::testing::Not(::testing::HasSubstr("right hand unknown")));

    const auto other =
        harness.execute(makeGoal(mission("<Sequence>" + place_left + "</Sequence>")));

    ASSERT_EQ(other->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << other->result->failure_reason;
    EXPECT_EQ(harness.authority->released, 1) << "both hands are known now, and empty";
    // Nothing is known about what was in a hand, so the detector is not asked to find it.
    for (const auto& phrases : robot.detector.written())
    {
        for (const std::string& phrase : phrases)
        {
            EXPECT_EQ(phrase.find('?'), std::string::npos) << phrase;
        }
    }
}

TEST(NervrosExecutor, LettingGoOfTheArmsSettlesWhatTheHandsHold)
{
    FakeRobot robot;
    Harness   harness;
    harness.authority->already_active = true;
    ASSERT_EQ(
        harness.execute(makeGoal(mission("<Sequence>" + kTuck + "</Sequence>")))->result->outcome,
        ExecuteMission::Result::OUTCOME_SUCCESS);
    ASSERT_EQ(harness.authority->released, 0);

    // Someone deactivated the arms, so anything in a hand fell. The next mission finds them free.
    harness.authority->already_active = false;
    const auto tucked = harness.execute(makeGoal(mission("<Sequence>" + kTuck + "</Sequence>")));

    ASSERT_EQ(tucked->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << tucked->result->failure_reason;
    EXPECT_EQ(harness.authority->acquired, 2) << "the arms were looked at again";
    EXPECT_EQ(robot.posture->goals, 2) << "and both hands are known to be empty";
    EXPECT_EQ(harness.authority->released, 1);
}

TEST(NervrosExecutor, APickThatWasCutOffMayHaveGraspedSoItsHandIsNotTrusted)
{
    FakeRobot robot;
    robot.pick->duration_ms = 20000;
    Harness harness;
    harness.start(makeGoal(mission("<Sequence>" + kPick + "</Sequence>")));
    ASSERT_TRUE(waitFor([&] { return robot.pick->goals == 1; }));

    (void)harness.stopAll("button");
    const auto run = harness.finished();

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_CANCELED);
    EXPECT_EQ(harness.authority->released, 0)
        << "the hand may have closed on the object before the cancel arrived";
    const RobotState state = harness.stateWhere([](const RobotState& s) {
        return s.mission_id.empty() && s.message.find("unknown") != std::string::npos;
    });
    EXPECT_THAT(state.message, ::testing::HasSubstr("right hand unknown"));
    EXPECT_THAT(state.message, ::testing::Not(::testing::HasSubstr("left hand unknown")));
}

TEST(NervrosExecutor, APickThatAStepsTimeoutCutOffIsNotTrustedEither)
{
    FakeRobot robot;
    robot.pick->duration_ms = 20000;
    Harness harness;

    // The Timeout halts the pick from its own thread, where no halt() of the executor's is called.
    const auto run = harness.execute(
        makeGoal(mission("<Sequence><Timeout msec=\"1500\">" + kPick + "</Timeout></Sequence>")));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_TIMEOUT)
        << run->result->failure_reason;
    EXPECT_EQ(harness.authority->released, 0);
    const RobotState state = harness.stateWhere([](const RobotState& s) {
        return s.mission_id.empty() && s.message.find("unknown") != std::string::npos;
    });
    EXPECT_THAT(state.message, ::testing::HasSubstr("right hand unknown"));
}

TEST(NervrosExecutor, APlaceWithNothingInTheHandFailsBeforeAnythingMoves)
{
    FakeRobot robot;
    Harness   harness;

    const auto run = harness.execute(makeGoal(mission("<Sequence>" + kPlace + "</Sequence>")));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_FAILURE);
    EXPECT_EQ(run->result->failed_node, "s1_PlaceInto/hand_must_hold_something_to_place");
    EXPECT_EQ(run->result->failed_step_id, "s1");
    EXPECT_THAT(
        run->result->failure_reason,
        ::testing::HasSubstr("hand_must_hold_something_to_place"));
    EXPECT_EQ(robot.approach->goals, 0);
    EXPECT_EQ(robot.place->goals, 0);
    EXPECT_EQ(harness.authority->released, 1) << "both hands are empty, so the arms go back";
}

TEST(NervrosExecutor, APickWithAFullHandFailsBeforeAnythingMovesAndTheOtherHandStillPicks)
{
    FakeRobot         robot;
    Harness           harness;
    const std::string pick_left =
        R"(<SubTree ID="PickObject" name="s1_PickObject" object_id="O17" phrase="red mug" arm="left"/>)";
    ASSERT_EQ(
        harness.execute(makeGoal(mission("<Sequence>" + kPick + "</Sequence>")))->result->outcome,
        ExecuteMission::Result::OUTCOME_SUCCESS);

    const auto again = harness.execute(makeGoal(mission("<Sequence>" + kPick + "</Sequence>")));
    EXPECT_EQ(again->result->outcome, ExecuteMission::Result::OUTCOME_FAILURE);
    EXPECT_EQ(again->result->failed_node, "s1_PickObject/hand_must_be_empty_to_pick");
    EXPECT_EQ(robot.pick->goals, 1);

    const auto other = harness.execute(makeGoal(mission("<Sequence>" + pick_left + "</Sequence>")));
    EXPECT_EQ(other->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << other->result->failure_reason;
    EXPECT_EQ(robot.pick->goals, 2);
    const RobotState state = harness.stateWhere(
        [](const RobotState& s) { return s.holding_left == "O17" && s.mission_id.empty(); });
    EXPECT_EQ(state.holding_left, "O17");
    EXPECT_EQ(state.holding_right, "red_block");
}

TEST(NervrosExecutor, ADryRunCarriesWhatItsOwnPickHoldsToItsPlaceAndLeavesTheRealHandsAlone)
{
    Harness           harness;
    const std::string place_step =
        R"(<SubTree ID="PlaceInto" name="s2_PlaceInto" container_id="brown_box" phrase="brown box container" arm="right"/>)";

    const auto run = harness.execute(makeGoal(
        mission("<Sequence>" + kPick + place_step + "</Sequence>"),
        ExecuteMission::Goal::MODE_DRY_RUN));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << run->result->failure_reason;
    const RobotState state = harness.stateWhere([](const RobotState& s) {
        return s.mission_id.empty() && s.message.rfind("idle", 0) == 0;
    });
    EXPECT_TRUE(state.holding_right.empty());
    EXPECT_THAT(state.message, ::testing::Not(::testing::HasSubstr("holds")));
}

TEST(NervrosExecutor, ADryRunRefusesAPlaceWithNothingInTheHandToo)
{
    Harness harness;

    const auto run = harness.execute(makeGoal(
        mission("<Sequence>" + kPlace + "</Sequence>"),
        ExecuteMission::Goal::MODE_DRY_RUN));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_FAILURE);
    EXPECT_EQ(run->result->failed_node, "s1_PlaceInto/hand_must_hold_something_to_place");
}

// --- shutting down ----------------------------------------------------------------------------

TEST(NervrosExecutor, AGoalThatComesAfterShutdownIsTurnedAway)
{
    Harness harness;
    harness.executor_node->shutdown();

    const auto run = harness.execute(makeGoal(mission("<Sequence>" + kTuck + "</Sequence>")));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_EQ(diagnostics(*run->result).at(0).at("code"), "SHUTTING_DOWN");
    EXPECT_EQ(harness.authority->acquired, 0);
}

// --- the deadman ------------------------------------------------------------------------------

/// Publishes heartbeats as an agent does, until it is destroyed.
class Heartbeats
{
public:
    explicit Heartbeats(const rclcpp::Node::SharedPtr& node, std::string client = "test")
      : client_(std::move(client))
      , publisher_(node->create_publisher<nervros_interfaces::msg::Heartbeat>(
            "/nervros_executor/heartbeat", rclcpp::QoS(1).best_effort()))
      , thread_([this](const std::stop_token& stop) {
          while (!stop.stop_requested())
          {
              nervros_interfaces::msg::Heartbeat beat;
              beat.client = client_;
              publisher_->publish(beat);
              std::this_thread::sleep_for(100ms);
          }
      })
    {}

private:
    std::string                                                      client_;
    rclcpp::Publisher<nervros_interfaces::msg::Heartbeat>::SharedPtr publisher_;
    std::jthread                                                     thread_;
};

TEST(NervrosExecutor, TheDeadmanStopsAMissionWhoseAgentFallsSilentAndHolds)
{
    FakeRobot robot;
    robot.navigate->duration_ms = 20000;
    Harness harness;
    auto    goal             = makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"));
    goal.heartbeat_timeout_s = 0.5F;

    const auto run = harness.execute(goal);

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_CANCELED);
    EXPECT_THAT(run->result->failure_reason, ::testing::HasSubstr("deadman"));
    EXPECT_LT(run->result->elapsed_s, 6.0F);
    EXPECT_GE(robot.navigate->cancels, 1) << "the walk was not cancelled";
    EXPECT_TRUE(harness.stateWhere([](const RobotState& s) { return s.stopped; }).stopped);
}

TEST(NervrosExecutor, HeartbeatsKeepAMissionRunningToItsEnd)
{
    FakeRobot robot;
    robot.navigate->duration_ms = 1500;
    Harness    harness;
    Heartbeats beats(harness.client_node);
    auto       goal          = makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"));
    goal.heartbeat_timeout_s = 0.5F;

    const auto run = harness.execute(goal);

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << run->result->failure_reason;
}

TEST(NervrosExecutor, AnotherClientsHeartbeatsDoNotKeepAMissionAlive)
{
    FakeRobot robot;
    robot.navigate->duration_ms = 20000;
    Harness    harness;
    Heartbeats stranger(harness.client_node, "someone else");
    auto       goal          = makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"));
    goal.heartbeat_timeout_s = 0.5F;
    goal.heartbeat_client    = "test";

    const auto run = harness.execute(goal);

    EXPECT_THAT(run->result->failure_reason, ::testing::HasSubstr("deadman"));
}

TEST(NervrosExecutor, TheExecutorCapsTheTimeoutAnAgentAsksFor)
{
    FakeRobot robot;
    robot.navigate->duration_ms = 20000;
    Harness harness({ rclcpp::Parameter("max_heartbeat_timeout_s", 0.5) });
    auto    goal             = makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"));
    goal.heartbeat_timeout_s = 1000.0F;

    const auto run = harness.execute(goal);

    EXPECT_THAT(run->result->failure_reason, ::testing::HasSubstr("deadman"));
    EXPECT_LT(run->result->elapsed_s, 6.0F);
}

TEST(NervrosExecutor, AMissionWithoutAHeartbeatTimeoutHasNoDeadman)
{
    FakeRobot robot;
    robot.navigate->duration_ms = 1500;
    Harness harness;

    const auto run = harness.execute(makeGoal(mission("<Sequence>" + kWalk + "</Sequence>")));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << run->result->failure_reason;
}

// --- the health gate --------------------------------------------------------------------------

/// Publishes the body's orientation, tilted about x by a settable angle.
class FakeImu
{
public:
    explicit FakeImu(const rclcpp::Node::SharedPtr& node)
      : publisher_(
            node->create_publisher<sensor_msgs::msg::Imu>("/test_imu", rclcpp::SensorDataQoS()))
      , thread_([this](const std::stop_token& stop) {
          while (!stop.stop_requested())
          {
              const double          half = tilt_deg.load() * std::numbers::pi / 360.0;
              sensor_msgs::msg::Imu msg;
              msg.orientation.x = std::sin(half);
              msg.orientation.w = std::cos(half);
              publisher_->publish(msg);
              std::this_thread::sleep_for(50ms);
          }
      })
    {}

    std::atomic<double> tilt_deg{ 0.0 };

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    std::jthread                                        thread_;
};

TEST(NervrosExecutor, TheHealthGateRefusesToWalkAFallenRobotAndSaysWhy)
{
    FakeRobot robot;
    Harness   harness({ rclcpp::Parameter("health_gate", true),
                        rclcpp::Parameter("imu_topic", "/test_imu"),
                        rclcpp::Parameter("controller_manager", "") });
    FakeImu   imu(harness.client_node);
    imu.tilt_deg = 90.0;
    const RobotState down =
        harness.stateWhere([](const RobotState& s) { return !s.can_move && s.tilt_deg > 80.0F; });
    EXPECT_THAT(down.cannot_move_reason, ::testing::HasSubstr("fallen: tilted 90 degrees"));

    const auto refused = harness.execute(makeGoal(mission("<Sequence>" + kWalk + "</Sequence>")));
    EXPECT_EQ(refused->result->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_EQ(diagnostics(*refused->result)[0]["code"], "ROBOT_CANNOT_MOVE");
    EXPECT_THAT(refused->result->failure_reason, ::testing::HasSubstr("cannot walk"));
    EXPECT_EQ(robot.navigate->goals, 0);

    imu.tilt_deg = 3.0;
    harness.stateWhere([](const RobotState& s) { return s.can_move; });
    const auto run = harness.execute(makeGoal(mission("<Sequence>" + kWalk + "</Sequence>")));
    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << run->result->failure_reason;
}

TEST(NervrosExecutor, AFallMidWalkStopsTheMission)
{
    FakeRobot robot;
    robot.navigate->duration_ms = 20000;
    Harness harness({ rclcpp::Parameter("health_gate", true),
                      rclcpp::Parameter("imu_topic", "/test_imu"),
                      rclcpp::Parameter("controller_manager", ""),
                      rclcpp::Parameter("health_trip_s", 0.3) });
    FakeImu imu(harness.client_node);
    harness.stateWhere([](const RobotState& s) { return s.can_move; });
    harness.start(makeGoal(mission("<Sequence>" + kWalk + "</Sequence>")));
    harness.waitUntilRunning();

    imu.tilt_deg   = 95.0;
    const auto run = harness.finished();

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_CANCELED);
    EXPECT_THAT(run->result->failure_reason, ::testing::HasSubstr("can no longer walk: fallen"));
    EXPECT_GE(robot.navigate->cancels, 1);
}

TEST(NervrosExecutor, TheHealthGateLetsADryRunThrough)
{
    Harness harness({ rclcpp::Parameter("health_gate", true),
                      rclcpp::Parameter("imu_topic", "/test_imu"),
                      rclcpp::Parameter("controller_manager", "") });

    const auto run = harness.execute(
        makeGoal(mission("<Sequence>" + kWalk + "</Sequence>"), ExecuteMission::Goal::MODE_DRY_RUN));

    EXPECT_EQ(run->result->outcome, ExecuteMission::Result::OUTCOME_SUCCESS)
        << run->result->failure_reason;
}

// --- teleop -----------------------------------------------------------------------------------

TEST(NervrosExecutor, TeleopDrivesWithinItsLimitsStopsWhenCommandsPauseAndBlocksMissions)
{
    Harness    harness({ rclcpp::Parameter("teleop_command_topic", "/test_cmd_vel"),
                         rclcpp::Parameter("teleop_deadman_s", 0.3) });
    std::mutex mutex;
    std::vector<geometry_msgs::msg::Twist> out;
    auto sub = harness.client_node->create_subscription<geometry_msgs::msg::Twist>(
        "/test_cmd_vel",
        rclcpp::QoS(1).reliable(),
        [&](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
            const std::lock_guard<std::mutex> lock(mutex);
            out.push_back(*msg);
        });
    auto teleop = harness.client_node->create_client<nervros_interfaces::srv::Teleop>(
        "/nervros_executor/teleop");
    ASSERT_TRUE(teleop->wait_for_service(20s));
    auto on    = std::make_shared<nervros_interfaces::srv::Teleop::Request>();
    on->enable = true;
    auto reply = teleop->async_send_request(on);
    ASSERT_EQ(reply.wait_for(10s), std::future_status::ready);
    ASSERT_TRUE(reply.get()->ok) << reply.get()->message;
    EXPECT_TRUE(harness.stateWhere([](const RobotState& s) { return s.teleop; }).teleop);

    auto cmd = harness.client_node->create_publisher<geometry_msgs::msg::Twist>(
        "/nervros_executor/teleop_cmd",
        rclcpp::QoS(1).reliable());
    geometry_msgs::msg::Twist fast;
    fast.linear.x = 2.0;  // past the 0.5 m/s limit
    for (int i = 0; i < 30; ++i)
    {
        cmd->publish(fast);
        std::this_thread::sleep_for(50ms);
    }
    {
        const std::lock_guard<std::mutex> lock(mutex);
        ASSERT_FALSE(out.empty());
        const auto top = std::ranges::max(out, {}, [](const auto& t) { return t.linear.x; });
        EXPECT_NEAR(top.linear.x, 0.5, 1e-9) << "not held to the speed limit";
        EXPECT_LT(out.front().linear.x, 0.5) << "it snapped instead of ramping";
    }

    const auto refused = harness.execute(makeGoal(mission("<Sequence>" + kWalk + "</Sequence>")));
    EXPECT_EQ(refused->result->outcome, ExecuteMission::Result::OUTCOME_REJECTED);
    EXPECT_EQ(diagnostics(*refused->result)[0]["code"], "TELEOP_ACTIVE");

    // The commands stop: within the deadman and the ramp-down, the base is told to stand.
    EXPECT_TRUE(waitFor(
        [&] {
            const std::lock_guard<std::mutex> lock(mutex);
            return !out.empty() && out.back().linear.x == 0.0;
        },
        5s));

    harness.stopAll("button");
    EXPECT_FALSE(harness.stateWhere([](const RobotState& s) { return !s.teleop; }).teleop);
}

// --- preview ----------------------------------------------------------------------------------

TEST(NervrosExecutor, APreviewChainsTurnsAndWalksFromTheRobotsPoseWithoutMovingIt)
{
    Harness                              harness({ rclcpp::Parameter("preview_timeout_s", 0.3) });
    tf2_ros::StaticTransformBroadcaster  tf(harness.client_node);
    geometry_msgs::msg::TransformStamped pose;
    pose.header.frame_id         = "map";
    pose.child_frame_id          = "base_footprint";
    pose.transform.translation.x = 1.0;
    pose.transform.translation.y = 2.0;
    pose.transform.rotation.w    = 1.0;
    tf.sendTransform(pose);

    auto preview = harness.client_node->create_client<nervros_interfaces::srv::PreviewMission>(
        "/nervros_executor/preview_mission");
    ASSERT_TRUE(preview->wait_for_service(20s));
    auto request      = std::make_shared<nervros_interfaces::srv::PreviewMission::Request>();
    request->tree_xml = mission(
        R"(<Sequence><SubTree ID="TurnInPlace" name="s1_TurnInPlace" degrees="90"/>)"
        R"(<SubTree ID="WalkStraight" name="s2_WalkStraight" direction="forward" distance_m="1.0"/>)"
        R"(<SubTree ID="GoToPose" name="s3_GoToPose" station="4.0;5.0;0.0"/></Sequence>)");
    auto reply = preview->async_send_request(request);
    ASSERT_EQ(reply.wait_for(30s), std::future_status::ready);
    const auto response = reply.get();

    ASSERT_TRUE(response->ok) << response->message;
    ASSERT_EQ(response->steps.size(), 3U);
    EXPECT_EQ(response->steps[0].step_id, "s1");
    EXPECT_THAT(response->steps[0].note, ::testing::HasSubstr("90 degrees left"));
    EXPECT_NEAR(response->steps[1].goal.pose.position.x, 1.0, 1e-6);
    EXPECT_NEAR(response->steps[1].goal.pose.position.y, 3.0, 1e-6) << "the walk ignored the turn";
    EXPECT_EQ(response->steps[2].goal.header.frame_id, "map");
    EXPECT_THAT(response->steps[2].note, ::testing::HasSubstr("planner is not up"));
    EXPECT_TRUE(response->steps[2].path.poses.empty());
}

int main(int argc, char** argv)
{
    // No other thread exists yet.
    // NOLINTNEXTLINE(concurrency-mt-unsafe)
    setenv("ROS_DOMAIN_ID", "79", 1);
    ::testing::InitGoogleMock(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
