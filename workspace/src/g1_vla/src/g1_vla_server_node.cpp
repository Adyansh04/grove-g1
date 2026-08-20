/**
 * @file g1_vla_server_node.cpp
 * @brief The grasp action server: query a policy, validate the chunk, then execute it.
 */

#include "g1_vla/g1_vla_server_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <g1_manipulation/hand_contact.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <thread>
#include <utility>

namespace g1_vla
{

namespace
{

/// The one thing a grasping hand is always allowed to touch: space the sensor saw as occupied.
const std::vector<std::string> kTouchables = { "<octomap>" };

rclcpp::QoS objectsQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

/// Waits on a future without spinning: the executor owns this node and re-entering it deadlocks.
template <typename FutureT>
bool settled(FutureT& future, double timeout_s)
{
    return future.wait_for(std::chrono::duration<double>(timeout_s)) == std::future_status::ready;
}

}  // namespace

G1VlaServer::G1VlaServer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_vla_server", options)
{
    engine_service_ =
        declare_parameter<std::string>("engine_service", "/g1_vla_engine/get_action_chunk");
    declare_parameter<double>("engine_timeout_s", 10.0);
    // A chunk that opens away from where the arm actually is means the policy misread the state.
    declare_parameter<double>("max_start_jump_rad", 0.15);
    // Waypoints further apart than this sweep space no validity check ever looked at.
    declare_parameter<double>("max_segment_step_rad", 0.20);
    declare_parameter<double>("velocity_scaling", 0.5);
    declare_parameter<int64_t>("max_rejected_chunks", 5);
    declare_parameter<double>("timeout_s", 90.0);
    declare_parameter<double>("chunk_exec_timeout_s", 10.0);
    declare_parameter<double>("success_lift_m", 0.05);
    declare_parameter<double>("object_timeout_ms", 1000.0);
    // "servo" needs a servo_node running; move_group.launch.py starts one with servo:=true.
    declare_parameter<std::string>("execution_mode", "trajectory");
    declare_parameter<std::string>("servo_topic", "/servo_node/delta_joint_cmds");
    declare_parameter<double>("servo_publish_rate", 50.0);
    refreshTunables();

    objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        "/objects",
        objectsQos(),
        [this](const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg) { onObjects(msg); });
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) { onJointStates(msg); });

    engine_      = create_client<GetActionChunk>(engine_service_);
    validity_    = create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
    get_scene_   = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_scene_ = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    servo_pub_   = create_publisher<control_msgs::msg::JointJog>(
        get_parameter("servo_topic").as_string(),
        rclcpp::QoS(rclcpp::KeepLast(1)));
    servo_command_type_ =
        create_client<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type");
}

G1VlaServer::~G1VlaServer()
{
    while (goals_running_.load() > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void G1VlaServer::initialize()
{
    model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this());
    model_        = model_loader_->getModel();
    if (model_ == nullptr)
    {
        throw std::runtime_error("no robot model; is robot_description on this node?");
    }

    const auto joints_of = [this](const std::vector<std::string>& groups) {
        std::vector<std::string> names;
        for (const std::string& group : groups)
        {
            const moveit::core::JointModelGroup* jmg = model_->getJointModelGroup(group);
            if (jmg == nullptr)
            {
                throw std::runtime_error("the SRDF has no group '" + group + "'");
            }
            const std::vector<std::string>& active = jmg->getActiveJointModelNames();
            names.insert(names.end(), active.begin(), active.end());
        }
        return names;
    };

    // Which joints each controller owns comes from the model, not from a list here, so the two
    // cannot drift. The names match g1_moveit_config's controller list.
    const std::vector<std::pair<std::string, std::vector<std::string>>> layout = {
        { "arm_trajectory_controller", { "left_arm", "right_arm" } },
        { "left_hand_controller", { "left_hand" } },
        { "right_hand_controller", { "right_hand" } },
    };
    for (const auto& [name, groups] : layout)
    {
        ControllerTarget target;
        target.name   = name;
        target.joints = joints_of(groups);
        target.client = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/" + name + "/follow_joint_trajectory");
        controllers_.push_back(std::move(target));
    }

    // The URDF's velocity limits are what the motor can do, not what the arm tracks: 22 to 37
    // rad/s against the 0.8 MoveIt actually times trajectories against. Checking a chunk against
    // the motor spec would pass everything, so the planning limits win wherever they exist.
    std::vector<std::string> fell_back;
    for (const ControllerTarget& controller : controllers_)
    {
        for (const std::string& joint : controller.joints)
        {
            // Declared already if the model loader read the same limits; declared here so the
            // lookup still works when nothing did.
            const std::string name =
                "robot_description_planning.joint_limits." + joint + ".max_velocity";
            if (!has_parameter(name))
            {
                declare_parameter<double>(name, 0.0);
            }
            const double planning = get_parameter(name).as_double();
            if (planning > 0.0)
            {
                limits_[joint] = planning;
                continue;
            }
            const moveit::core::VariableBounds& bounds = model_->getVariableBounds(joint);
            limits_[joint] = bounds.velocity_bounded_ ? bounds.max_velocity_ : 0.0;
            fell_back.push_back(joint);
        }
    }
    if (!fell_back.empty())
    {
        RCLCPP_WARN(
            get_logger(),
            "%zu joint(s) have no planning velocity limit and are checked against the URDF's "
            "motor spec instead, starting with '%s'",
            fell_back.size(),
            fell_back.front().c_str());
    }

    grasp_server_ = rclcpp_action::create_server<Grasp>(
        this,
        "~/grasp",
        [this](const rclcpp_action::GoalUUID&, const std::shared_ptr<const Grasp::Goal>&) {
            return acquire() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
                               rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandle>&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const std::shared_ptr<GoalHandle>& handle) {
            std::thread([this, handle] {
                goals_running_.fetch_add(1);
                try
                {
                    executeGrasp(handle);
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(get_logger(), "grasp goal threw: %s", e.what());
                    auto result     = std::make_shared<Grasp::Result>();
                    result->success = false;
                    result->message = std::string("aborted on an internal error: ") + e.what();
                    if (handle->is_executing() || handle->is_canceling())
                    {
                        handle->abort(result);
                    }
                }
                busy_.store(false);
                goals_running_.fetch_sub(1);
            }).detach();
        });

    // The arm limit is logged because it is the one number that decides whether the velocity
    // check means anything, and it has two very different plausible values.
    RCLCPP_INFO(
        get_logger(),
        "grasp server ready in %s mode, engine at '%s', arm velocity limit %.2f rad/s",
        execution_mode_.c_str(),
        engine_service_.c_str(),
        limits_.at("right_elbow_joint"));
}

void G1VlaServer::refreshTunables()
{
    engine_timeout_s_     = get_parameter("engine_timeout_s").as_double();
    max_start_jump_rad_   = get_parameter("max_start_jump_rad").as_double();
    max_segment_step_rad_ = get_parameter("max_segment_step_rad").as_double();
    velocity_scaling_     = get_parameter("velocity_scaling").as_double();
    max_rejected_chunks_  = static_cast<int>(get_parameter("max_rejected_chunks").as_int());
    timeout_s_            = get_parameter("timeout_s").as_double();
    chunk_exec_timeout_s_ = get_parameter("chunk_exec_timeout_s").as_double();
    success_lift_m_       = get_parameter("success_lift_m").as_double();
    object_timeout_s_     = get_parameter("object_timeout_ms").as_double() / 1000.0;
    execution_mode_       = get_parameter("execution_mode").as_string();
    servo_publish_rate_   = get_parameter("servo_publish_rate").as_double();
}

bool G1VlaServer::acquire()
{
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true))
    {
        RCLCPP_WARN(get_logger(), "rejecting a goal: the arm is already running one");
        return false;
    }
    return true;
}

void G1VlaServer::onObjects(const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg)
{
    const std::lock_guard<std::mutex> lock(objects_mutex_);
    objects_ = *msg;
}

void G1VlaServer::onJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
{
    const std::lock_guard<std::mutex> lock(joints_mutex_);
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
        measured_[msg->name[i]] = msg->position[i];
    }
}

JointMap G1VlaServer::measuredJoints()
{
    const std::lock_guard<std::mutex> lock(joints_mutex_);
    return measured_;
}

std::optional<double> G1VlaServer::objectHeight(const std::string& object_id)
{
    vision_msgs::msg::Detection3DArray snapshot;
    {
        const std::lock_guard<std::mutex> lock(objects_mutex_);
        snapshot = objects_;
    }
    if (snapshot.detections.empty() ||
        (now() - rclcpp::Time(snapshot.header.stamp)).seconds() > object_timeout_s_)
    {
        return std::nullopt;
    }
    for (const vision_msgs::msg::Detection3D& detection : snapshot.detections)
    {
        if (!detection.results.empty() && detection.results.front().hypothesis.class_id == object_id)
        {
            return detection.bbox.center.position.z;
        }
    }
    return std::nullopt;
}

bool G1VlaServer::setHandContact(const std::string& side, bool allowed)
{
    const std::vector<std::string> links = g1_manipulation::handContactLinks(*model_, side);
    if (links.empty())
    {
        RCLCPP_ERROR(get_logger(), "the model has no '%s_hand' group", side.c_str());
        return false;
    }
    return g1_manipulation::applyHandContact(
        get_scene_,
        apply_scene_,
        get_logger(),
        links,
        kTouchables,
        allowed,
        true);
}

std::optional<G1VlaServer::JointTrajectory>
G1VlaServer::requestChunk(const std::string& instruction, std::string& why)
{
    auto request         = std::make_shared<GetActionChunk::Request>();
    request->instruction = instruction;
    auto future          = engine_->async_send_request(request);
    if (!settled(future, engine_timeout_s_))
    {
        why = "the policy engine did not answer within " + std::to_string(engine_timeout_s_) + " s";
        return std::nullopt;
    }
    GetActionChunk::Response::SharedPtr response = future.get();
    if (!response->ok)
    {
        why = "the policy engine refused: " + response->message;
        return std::nullopt;
    }
    return response->chunk;
}

std::string G1VlaServer::rejectionReason(const JointTrajectory& chunk, const std::string& side)
{
    if (!wellFormed(chunk))
    {
        return "malformed chunk";
    }

    const JointMap measured = measuredJoints();

    const std::optional<double> jump = startJump(chunk, measured);
    if (!jump.has_value())
    {
        return "the chunk names a joint that is not being measured";
    }
    if (*jump > max_start_jump_rad_)
    {
        return "starts " + std::to_string(*jump) + " rad from the measured pose";
    }

    const double step = maxSegmentStep(chunk);
    if (step > max_segment_step_rad_)
    {
        return "steps " + std::to_string(step) + " rad between waypoints";
    }

    const std::optional<double> ratio = maxVelocityRatio(chunk, measured, limits_);
    if (!ratio.has_value())
    {
        return "the chunk names a joint with no velocity limit in the model";
    }
    if (*ratio > velocity_scaling_)
    {
        return "asks for " + std::to_string(*ratio * 100.0) + "% of a joint's velocity limit";
    }

    return checkWaypoints(chunk, side + "_arm");
}

std::string G1VlaServer::checkWaypoints(const JointTrajectory& chunk, const std::string& group)
{
    const JointMap measured = measuredJoints();

    for (std::size_t p = 0; p < chunk.points.size(); ++p)
    {
        JointMap state = measured;
        for (std::size_t i = 0; i < chunk.joint_names.size(); ++i)
        {
            state[chunk.joint_names[i]] = chunk.points[p].positions[i];
        }

        auto request        = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        request->group_name = group;
        request->robot_state.is_diff = false;
        for (const auto& [name, position] : state)
        {
            request->robot_state.joint_state.name.push_back(name);
            request->robot_state.joint_state.position.push_back(position);
        }

        auto future = validity_->async_send_request(request);
        if (!settled(future, 5.0))
        {
            return "/check_state_validity did not answer";
        }
        const auto response = future.get();
        if (!response->valid)
        {
            // Naming the pair matters: "waypoint 3 is in collision" gives an operator nothing to
            // act on, and the two likely causes, the scene and the robot itself, want opposite
            // responses.
            const std::string what = response->contacts.empty() ?
                                         "" :
                                         " (" + response->contacts.front().contact_body_1 +
                                             " against " +
                                             response->contacts.front().contact_body_2 + ")";
            return "waypoint " + std::to_string(p) + " of " + std::to_string(chunk.points.size()) +
                   " is in collision" + what;
        }
    }
    return {};
}

bool G1VlaServer::executeChunk(const JointTrajectory& chunk, std::string& why)
{
    using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    std::vector<std::shared_future<GoalHandleFJT::WrappedResult>> results;
    std::vector<std::string>                                      sent_to;

    const bool      servoing = execution_mode_ == "servo";
    JointTrajectory arm_slice;

    for (const ControllerTarget& controller : controllers_)
    {
        const JointTrajectory slice = splitByController(chunk, controller.joints);
        if (slice.joint_names.empty())
        {
            continue;
        }
        // Servo owns the arm group when it is running, so the arm's share is streamed below
        // rather than sent as a trajectory. The hands are outside that group either way.
        if (servoing && controller.name == "arm_trajectory_controller")
        {
            arm_slice = slice;
            continue;
        }
        if (!controller.client->action_server_is_ready())
        {
            why = controller.name + " is not accepting trajectories; is the arm acquired?";
            return false;
        }

        FollowJointTrajectory::Goal goal;
        goal.trajectory = slice;
        auto handle     = controller.client->async_send_goal(goal);
        if (!settled(handle, chunk_exec_timeout_s_) || handle.get() == nullptr)
        {
            why = controller.name + " rejected the trajectory";
            return false;
        }
        results.push_back(controller.client->async_get_result(handle.get()));
        sent_to.push_back(controller.name);
    }

    if (results.empty() && arm_slice.joint_names.empty())
    {
        why = "the chunk names no joint any controller owns";
        return false;
    }

    // Streamed before the hand goals are awaited, so a chunk driving both moves them together.
    if (!arm_slice.joint_names.empty() && !streamArmServo(arm_slice, why))
    {
        return false;
    }

    for (std::size_t i = 0; i < results.size(); ++i)
    {
        if (!settled(results[i], chunk_exec_timeout_s_))
        {
            why = sent_to[i] + " did not finish the trajectory in time";
            return false;
        }
        const GoalHandleFJT::WrappedResult result = results[i].get();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            why = sent_to[i] + " failed the trajectory: " + result.result->error_string;
            return false;
        }
    }
    return true;
}

std::vector<double> G1VlaServer::clampToLimits(
    const std::vector<std::string>& joints, const std::vector<double>& velocities) const
{
    std::vector<double> clamped;
    clamped.reserve(velocities.size());
    for (std::size_t i = 0; i < velocities.size(); ++i)
    {
        // Correcting a large error would otherwise ask for a speed the chunk was never checked
        // at, which is the one thing the gate exists to prevent.
        const auto   limit_it = limits_.find(joints[i]);
        const double cap = limit_it == limits_.end() ? 0.0 : limit_it->second * velocity_scaling_;
        clamped.push_back(std::clamp(velocities[i], -cap, cap));
    }
    return clamped;
}

bool G1VlaServer::selectServoJointJog(std::string& why)
{
    if (!servo_command_type_->service_is_ready())
    {
        why = "servo is not running; execution_mode is 'servo' but nothing serves "
              "/servo_node/switch_command_type";
        return false;
    }
    auto request          = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
    auto future           = servo_command_type_->async_send_request(request);
    if (!settled(future, 5.0) || !future.get()->success)
    {
        why = "servo refused to switch to joint-jog mode";
        return false;
    }
    return true;
}

bool G1VlaServer::streamArmServo(const JointTrajectory& arm_slice, std::string& why)
{
    const double tick = 1.0 / servo_publish_rate_;
    if (trackingVelocity(arm_slice, measuredJoints(), 0.0, tick).empty())
    {
        why = "the chunk names an arm joint that is not being measured";
        return false;
    }

    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(tick));
    const auto start = std::chrono::steady_clock::now();

    // Paced off the wall clock rather than a sleep per iteration: servo integrates however long
    // each command was actually in force, so a loop that drifts late travels further.
    for (auto next = start;; next += period)
    {
        // Re-read every tick. The velocity is a correction toward the next validated waypoint,
        // so tracking error is cancelled rather than accumulated.
        const std::vector<double> velocities = trackingVelocity(
            arm_slice,
            measuredJoints(),
            std::chrono::duration<double>(next - start).count(),
            tick);
        if (velocities.empty())
        {
            break;
        }
        control_msgs::msg::JointJog jog;
        jog.header.stamp = now();
        jog.joint_names  = arm_slice.joint_names;
        jog.velocities   = clampToLimits(arm_slice.joint_names, velocities);
        jog.duration     = tick;
        servo_pub_->publish(jog);
        std::this_thread::sleep_until(next + period);
    }

    // Nothing is published to stop with. Servo halts on its own once incoming_command_timeout
    // passes without a command, which is the same dead-man that covers this process dying.
    return true;
}

void G1VlaServer::cancelAll()
{
    for (const ControllerTarget& controller : controllers_)
    {
        if (controller.client->action_server_is_ready())
        {
            controller.client->async_cancel_all_goals();
        }
    }
}

G1VlaServer::Outcome G1VlaServer::runGrasp(
    const std::shared_ptr<GoalHandle>& goal_handle, const std::string& side, double start_z)
{
    const std::shared_ptr<const Grasp::Goal> goal = goal_handle->get_goal();
    Outcome                                  outcome;
    auto                                     feedback = std::make_shared<Grasp::Feedback>();
    feedback->phase                                   = Grasp::Feedback::PHASE_ACTING;

    if (execution_mode_ == "servo" && !selectServoJointJog(outcome.message))
    {
        return outcome;
    }

    const rclcpp::Time deadline            = now() + rclcpp::Duration::from_seconds(timeout_s_);
    int                consecutive_rejects = 0;

    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling())
        {
            cancelAll();
            outcome.message = "cancelled";
            return outcome;
        }
        if (now() > deadline)
        {
            outcome.message = "timed out after " + std::to_string(timeout_s_) + " s";
            return outcome;
        }

        std::string                          why;
        const std::optional<JointTrajectory> chunk = requestChunk(goal->instruction, why);
        if (!chunk.has_value())
        {
            outcome.message = why;
            return outcome;
        }

        const std::string rejected = rejectionReason(*chunk, side);
        if (!rejected.empty())
        {
            ++outcome.rejected;
            ++consecutive_rejects;
            RCLCPP_WARN(
                get_logger(),
                "rejected a chunk (%d of %d before aborting): %s",
                consecutive_rejects,
                max_rejected_chunks_,
                rejected.c_str());
            if (consecutive_rejects >= max_rejected_chunks_)
            {
                // Named "blocked" so the tree can tell a refused skill from a broken one.
                outcome.message = "blocked: " + rejected;
                return outcome;
            }
            feedback->chunks_rejected = outcome.rejected;
            goal_handle->publish_feedback(feedback);
            continue;
        }
        consecutive_rejects = 0;

        if (!executeChunk(*chunk, why))
        {
            cancelAll();
            outcome.message = why;
            return outcome;
        }
        ++outcome.executed;
        feedback->chunks_executed = outcome.executed;
        goal_handle->publish_feedback(feedback);

        const std::optional<double> height = objectHeight(goal->object_id);
        if (height.has_value() && (*height - start_z) >= success_lift_m_)
        {
            outcome.success = true;
            outcome.message =
                "lifted " + goal->object_id + " by " + std::to_string(*height - start_z) + " m";
            return outcome;
        }
    }

    outcome.message = "shutting down";
    return outcome;
}

void G1VlaServer::executeGrasp(const std::shared_ptr<GoalHandle>& goal_handle)
{
    const std::shared_ptr<const Grasp::Goal> goal   = goal_handle->get_goal();
    auto                                     result = std::make_shared<Grasp::Result>();

    refreshTunables();

    auto feedback   = std::make_shared<Grasp::Feedback>();
    feedback->phase = Grasp::Feedback::PHASE_PREPARING;
    goal_handle->publish_feedback(feedback);

    if (goal->arm != Grasp::Goal::ARM_LEFT && goal->arm != Grasp::Goal::ARM_RIGHT)
    {
        result->message =
            "arm must be '" + Grasp::Goal::ARM_LEFT + "' or '" + Grasp::Goal::ARM_RIGHT + "'";
        goal_handle->abort(result);
        return;
    }

    const std::optional<double> start_z = objectHeight(goal->object_id);
    if (!start_z.has_value())
    {
        result->message = "no recent pose for '" + goal->object_id +
                          "' on /objects, so nothing could measure the lift";
        goal_handle->abort(result);
        return;
    }

    if (!setHandContact(goal->arm, true))
    {
        result->message = "could not exempt the hand from collision checking";
        goal_handle->abort(result);
        return;
    }

    const Outcome outcome = runGrasp(goal_handle, goal->arm, *start_z);

    if (!setHandContact(goal->arm, false))
    {
        RCLCPP_ERROR(
            get_logger(),
            "the hand exemption was not restored; the scene is blinded to the "
            "octomap until move_group restarts");
    }

    result->success = outcome.success;
    // Counters on every path, not just the happy one: how much of a policy's output survived the
    // gate is the measurement this skill exists to produce.
    result->message = outcome.message + " [" + std::to_string(outcome.executed) + " executed, " +
                      std::to_string(outcome.rejected) + " rejected]";
    if (outcome.success)
    {
        RCLCPP_INFO(get_logger(), "%s", outcome.message.c_str());
        goal_handle->succeed(result);
    }
    else if (goal_handle->is_canceling())
    {
        goal_handle->canceled(result);
    }
    else
    {
        RCLCPP_ERROR(
            get_logger(),
            "grasp failed after %u chunk(s), %u rejected: %s",
            outcome.executed,
            outcome.rejected,
            outcome.message.c_str());
        goal_handle->abort(result);
    }
}

}  // namespace g1_vla
