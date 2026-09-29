/**
 * @file retreat_action.cpp
 * @brief The retreat: backing straight out from a surface.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "g1_locomotion/g1_base_approach_node.hpp"

namespace g1_locomotion
{

void BaseApproachNode::runRetreat(const std::shared_ptr<GoalHandleRetreat>& handle)
{
    const auto goal     = handle->get_goal();
    auto       result   = std::make_shared<Retreat::Result>();
    auto       feedback = std::make_shared<Retreat::Feedback>();

    const auto start = basePose();
    if (!start)
    {
        result->message = "backing_off: no base pose to retreat from";
        handle->abort(result);
        return;
    }

    const double timeout_s = goal->timeout_s > 0.0 ? goal->timeout_s : default_timeout_s_;
    const auto   deadline  = deadlineIn(timeout_s);

    // Straight back only: turning beside a surface swings the robot and what it holds across
    // it. The navigation goal that follows does the turning.
    feedback->phase = Retreat::Feedback::PHASE_BACKING_OFF;
    handle->publish_feedback(feedback);

    // nullopt on a TF outage, not 0.0, or the loop would reverse blind until the deadline.
    const auto travelled = [&]() -> std::optional<double> {
        const auto here = basePose();
        if (!here)
        {
            return std::nullopt;
        }
        return std::hypot(
            here->pose.position.x - start->pose.position.x,
            here->pose.position.y - start->pose.position.y);
    };
    double     last_travelled = 0.0;
    const auto blind_until    = [&,
                              grace =
                                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(lookup_grace_s_))] {
        return std::chrono::steady_clock::now() + grace;
    };
    auto blind_deadline = blind_until();

    const int feedback_every = std::max(1, static_cast<int>(cmd_rate_hz_ / 2.0));
    int       tick           = 0;
    while (rclcpp::ok() && !stopping_.load() && last_travelled < goal->distance_m &&
           std::chrono::steady_clock::now() < deadline)
    {
        if (handle->is_canceling())
        {
            publish(0.0, 0.0, 0.0);
            result->travelled_m = last_travelled;
            result->message     = "backing_off: cancelled";
            handle->canceled(result);
            return;
        }

        if (const auto measured = travelled())
        {
            last_travelled = *measured;
            blind_deadline = blind_until();
        }
        else
        {
            // As in the approach: stop, and give TF lookup_grace_s to come back.
            publish(0.0, 0.0, 0.0);
            if (std::chrono::steady_clock::now() > blind_deadline)
            {
                result->success     = false;
                result->travelled_m = last_travelled;
                result->message     = "backing_off: lost the base pose";
                handle->abort(result);
                return;
            }
            std::this_thread::sleep_for(tickPeriod());
            continue;
        }

        publish(-retreat_speed_mps_, 0.0, 0.0);
        if (tick++ % feedback_every == 0)
        {
            feedback->travelled_m = last_travelled;
            handle->publish_feedback(feedback);
        }
        std::this_thread::sleep_for(tickPeriod());
    }
    settle();

    // The last good reading if TF is still down, or a completed retreat reads as 0 m.
    const double backed = travelled().value_or(last_travelled);
    result->travelled_m = backed;
    result->success     = backed >= goal->distance_m;
    result->message     = result->success ? "reversed " + std::to_string(backed) + " m clear" :
                                            "backing_off: only made " + std::to_string(backed) +
                                            " m of " + std::to_string(goal->distance_m);
    RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
    if (result->success)
    {
        handle->succeed(result);
    }
    else
    {
        handle->abort(result);
    }
}

}  // namespace g1_locomotion
