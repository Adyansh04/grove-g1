/**
 * @file retreat_action.cpp
 * @brief The retreat: backing straight out from a surface.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
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
    bool       blocked        = false;
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

        // Never into what the costmap holds behind: a home's aisle is narrower than the back-off
        // a table wants, and the walk that follows steps clear of the rest.
        if (const auto around = surroundings();
            around && clearanceBehind(
                          around->obstacles,
                          around->x,
                          around->y,
                          around->yaw,
                          step_limits_.body_radius_m) < step_limits_.body_radius_m + retreat_gap_m_)
        {
            blocked = true;
            break;
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
    // The stride's last sway after the stop can read a few cm short of a distance reached.
    const bool far_enough = backed + retreat_settle_slop_m_ >= goal->distance_m;
    result->success       = far_enough || (blocked && backed >= retreat_min_m_);
    if (far_enough)
    {
        result->message = std::format("reversed {:.2f} m clear", backed);
    }
    else if (result->success)
    {
        result->message = std::format(
            "reversed {:.2f} m of {:.2f}: something is behind",
            backed,
            goal->distance_m);
    }
    else
    {
        result->message = std::format(
            "backing_off: only made {:.2f} m of {:.2f}{}",
            backed,
            goal->distance_m,
            blocked ? ", with something close behind" : "");
    }
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
