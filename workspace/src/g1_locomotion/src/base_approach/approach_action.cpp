/**
 * @file approach_action.cpp
 * @brief The approach: walking the object into the arm's reach window.
 */

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "g1_locomotion/g1_base_approach_node.hpp"

namespace g1_locomotion
{

ApproachLimits BaseApproachNode::limitsFor(const ApproachObject::Goal& goal) const
{
    ApproachLimits limits = limits_;
    // The window mirrors with the arm, exactly as the grasp offset does: the right hand
    // works to the pelvis's -y, the left to its +y.
    if (goal.arm == ApproachObject::Goal::ARM_LEFT)
    {
        limits.target_y_m = -limits.target_y_m;
    }
    for (std::size_t i = 0; i < standoff_ids_.size(); ++i)
    {
        if (standoff_ids_[i] == goal.object_id)
        {
            limits.target_x_m = standoff_target_x_[i];
            RCLCPP_INFO(
                get_logger(),
                "'%s' is approached to %.3f rather than the default %.3f",
                goal.object_id.c_str(),
                limits.target_x_m,
                limits_.target_x_m);
            break;
        }
    }
    return limits;
}

void BaseApproachNode::runApproach(const std::shared_ptr<GoalHandleApproach>& handle)
{
    const auto goal   = handle->get_goal();
    auto       result = std::make_shared<ApproachObject::Result>();

    if (goal->arm != ApproachObject::Goal::ARM_LEFT && goal->arm != ApproachObject::Goal::ARM_RIGHT)
    {
        result->message = "locating: arm must be 'left' or 'right', got '" + goal->arm + "'";
        handle->abort(result);
        return;
    }
    const ApproachLimits limits = limitsFor(*goal);

    const double timeout_s = goal->timeout_s > 0.0 ? goal->timeout_s : default_timeout_s_;
    const auto   deadline  = deadlineIn(timeout_s);
    auto         feedback  = std::make_shared<ApproachObject::Feedback>();

    // Fixed up front: chasing the object's bearing would never arrive square to anything.
    const auto start_pose = basePose();
    if (!start_pose)
    {
        result->message = "locating: no base pose";
        handle->abort(result);
        return;
    }
    const double working_yaw =
        goal->use_current_heading ? tf2::getYaw(start_pose->pose.orientation) : goal->working_yaw;

    // Feedback at about 2 Hz, not every tick.
    const int  feedback_every = std::max(1, static_cast<int>(cmd_rate_hz_ / 2.0));
    int        tick           = 0;
    auto       last_measured  = std::chrono::steady_clock::now();
    const auto blind_for      = [&last_measured] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_measured)
            .count();
    };

    while (rclcpp::ok())
    {
        if (handle->is_canceling())
        {
            publish(0.0, 0.0, 0.0);
            result->message = "closing: cancelled";
            handle->canceled(result);
            return;
        }
        if (std::chrono::steady_clock::now() > deadline)
        {
            publish(0.0, 0.0, 0.0);
            result->message = "closing: gave up after " + std::to_string(timeout_s) + " s";
            handle->abort(result);
            return;
        }

        const auto object = objectInBase(goal->object_id);
        const auto here   = basePose();
        if (!object || !here)
        {
            // Stand still without a measurement, for up to lookup_grace_s: TF and detections
            // both drop out briefly.
            publish(0.0, 0.0, 0.0);
            if (blind_for() > lookup_grace_s_)
            {
                result->message = "locating: no fresh pose for '" + goal->object_id +
                                  "' on /objects after " + std::to_string(lookup_grace_s_) + " s";
                handle->abort(result);
                return;
            }
            std::this_thread::sleep_for(tickPeriod());
            continue;
        }
        last_measured = std::chrono::steady_clock::now();

        // Judged in the base frame, which is what the arm's reach is defined in.
        const double heading_error = wrap(working_yaw - tf2::getYaw(here->pose.orientation));
        const auto   command =
            planApproach(object->point.x, object->point.y, heading_error, limits, gait_);

        result->final_x_m         = object->point.x;
        result->final_y_m         = object->point.y;
        feedback->forward_error_m = command.forward_error_m;
        feedback->lateral_error_m = command.lateral_error_m;

        if (command.state == ApproachState::kOvershot)
        {
            publish(0.0, 0.0, 0.0);
            result->message = "closing: the object is " + std::to_string(object->point.x) +
                              " m ahead, under the robot's own footprint; re-stage "
                              "through Nav2";
            handle->abort(result);
            return;
        }
        if (command.state == ApproachState::kInvalid)
        {
            publish(0.0, 0.0, 0.0);
            result->message = "locating: the configured reach window is unusable";
            handle->abort(result);
            return;
        }

        if (command.state == ApproachState::kArrived)
        {
            // Stop, let the stride finish, then re-judge: the robot coasts, and a coast out of
            // the window has to be closed again.
            feedback->phase = ApproachObject::Feedback::PHASE_VERIFYING;
            handle->publish_feedback(feedback);
            settle();

            const auto settled = objectInBase(goal->object_id);
            if (!settled)
            {
                continue;
            }
            const auto verdict =
                planApproach(settled->point.x, settled->point.y, 0.0, limits, gait_);
            result->final_x_m = settled->point.x;
            result->final_y_m = settled->point.y;
            if (verdict.state != ApproachState::kArrived)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "coasted back out of the window to (%.3f, %.3f); closing again",
                    settled->point.x,
                    settled->point.y);
                continue;
            }

            result->success = true;
            result->message = "the object is at (" + std::to_string(settled->point.x) + ", " +
                              std::to_string(settled->point.y) + ")";
            RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
            handle->succeed(result);
            return;
        }

        if (tick % feedback_every == 0)
        {
            feedback->phase = ApproachObject::Feedback::PHASE_CLOSING;
            handle->publish_feedback(feedback);
            RCLCPP_INFO(
                get_logger(),
                "closing: forward %+.3f, lateral %+.3f, heading %+.1f deg -> "
                "(%.2f, %.2f, %.2f)",
                command.forward_error_m,
                command.lateral_error_m,
                heading_error * 180.0 / M_PI,
                command.vx_mps,
                command.vy_mps,
                command.yaw_rate_rps);
        }
        ++tick;

        publish(command.vx_mps, command.vy_mps, command.yaw_rate_rps);
        std::this_thread::sleep_for(tickPeriod());
    }

    publish(0.0, 0.0, 0.0);
    result->message = "closing: shutting down";
    handle->abort(result);
}

}  // namespace g1_locomotion
