/**
 * @file chunks.cpp
 * @brief A chunk's life: asked of the engine, checked against the planning scene, executed.
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <future>
#include <moveit_msgs/msg/robot_state.hpp>
#include <thread>

#include "g1_vla/g1_vla_server_node.hpp"

namespace g1_vla
{

namespace
{

/// Scene and servo-mode services answer or they do not; this is not a tunable.
constexpr double kServiceTimeoutS = 5.0;

/// Waits on a future without spinning: the executor owns this node and re-entering it deadlocks.
template <typename FutureT>
bool settled(FutureT& future, double timeout_s)
{
    return future.wait_for(std::chrono::duration<double>(timeout_s)) == std::future_status::ready;
}

}  // namespace

ChunkFuture G1VlaServer::sendChunkRequest(const std::string& instruction, bool new_episode)
{
    auto request         = std::make_shared<GetActionChunk::Request>();
    request->instruction = instruction;
    request->new_episode = new_episode;
    return engine_->async_send_request(request).share();
}

std::optional<G1VlaServer::JointTrajectory>
G1VlaServer::awaitChunk(ChunkFuture& pending, std::string& why) const
{
    if (!settled(pending, engine_timeout_s_))
    {
        why = "the policy engine did not answer within " + std::to_string(engine_timeout_s_) + " s";
        return std::nullopt;
    }
    GetActionChunk::Response::SharedPtr response = pending.get();
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

    return checkWaypoints(chunk, side + "_arm", measured);
}

std::string G1VlaServer::checkWaypoints(
    const JointTrajectory& chunk, const std::string& group, const JointMap& measured)
{
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
        if (!settled(future, kServiceTimeoutS))
        {
            return "/check_state_validity did not answer";
        }
        const auto response = future.get();
        if (!response->valid)
        {
            // Name the pair: a scene contact and a self-contact want opposite responses.
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

bool G1VlaServer::dispatchFailed(std::string& why)
{
    for (auto& [name, result] : dispatched_)
    {
        if (result.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        {
            continue;
        }
        // A chunk superseded by the next one comes back cancelled and carries no error code, so
        // only a tolerance or validity complaint means the controller actually refused the plan.
        const int32_t code = result.get().result->error_code;
        if (code != control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
        {
            why = name + " aborted a chunk: " + std::to_string(code);
            return true;
        }
    }
    return false;
}

bool G1VlaServer::executeChunk(const JointTrajectory& chunk, std::string& why, bool await_completion)
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

    if (!await_completion)
    {
        // Kept so the next iteration can see a controller that aborted this one; the trajectory
        // itself keeps running until the following chunk supersedes it.
        dispatched_.clear();
        for (std::size_t i = 0; i < results.size(); ++i)
        {
            dispatched_.emplace_back(sent_to[i], results[i]);
        }
        return true;
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
    if (!settled(future, kServiceTimeoutS) || !future.get()->success)
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
        // cancelAll() cannot reach servo: it drives the controller's topic interface, not a goal.
        const std::shared_ptr<GoalHandle> goal = active_goal_;
        if (!rclcpp::ok() || (goal != nullptr && goal->is_canceling()))
        {
            return true;
        }
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

}  // namespace g1_vla
