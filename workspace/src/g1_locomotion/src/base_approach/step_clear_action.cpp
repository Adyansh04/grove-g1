/**
 * @file step_clear_action.cpp
 * @brief The step clear: out of Nav2's collision band before a walk or a turn.
 */

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "g1_locomotion/g1_base_approach_node.hpp"

namespace g1_locomotion
{

namespace
{

/// Nav2's lethal cost in an OccupancyGrid; 99 only marks the band inflated around it.
constexpr std::int8_t kLethal = 100;

}  // namespace

std::optional<BaseApproachNode::Surroundings> BaseApproachNode::surroundings()
{
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap;
    {
        const std::lock_guard<std::mutex> lock(costmap_mutex_);
        costmap = costmap_;
    }
    if (!costmap)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "nothing on %s yet",
            costmap_topic_.c_str());
        return std::nullopt;
    }
    const auto base = basePose(costmap->header.frame_id);
    if (!base)
    {
        return std::nullopt;
    }
    Surroundings around;
    around.x          = base->pose.position.x;
    around.y          = base->pose.position.y;
    around.yaw        = tf2::getYaw(base->pose.orientation);
    const auto& info  = costmap->info;
    around.resolution = info.resolution;
    for (std::uint32_t row = 0; row < info.height; ++row)
    {
        for (std::uint32_t column = 0; column < info.width; ++column)
        {
            if (costmap->data[(static_cast<std::size_t>(row) * info.width) + column] == kLethal)
            {
                around.obstacles.push_back(
                    { info.origin.position.x + ((column + 0.5) * info.resolution),
                      info.origin.position.y + ((row + 0.5) * info.resolution) });
            }
        }
    }
    return around;
}

void BaseApproachNode::runStepClear(const std::shared_ptr<GoalHandleStepClear>& handle)
{
    const auto goal     = handle->get_goal();
    auto       result   = std::make_shared<StepClear::Result>();
    auto       feedback = std::make_shared<StepClear::Feedback>();
    const auto deadline = deadlineIn(goal->timeout_s > 0.0 ? goal->timeout_s : step_timeout_s_);
    const auto fail     = [&](const std::string& why) {
        publish(0.0, 0.0, 0.0);
        result->message = feedback->phase + ": " + why;
        RCLCPP_WARN(get_logger(), "step clear failed, %s", result->message.c_str());
        handle->abort(result);
    };

    std::optional<StepClearPlan> plan;
    std::optional<ObstaclePoint> step_start;
    double                       heading        = 0.0;
    double                       floor          = 0.0;
    double                       resolution     = 0.0;
    auto                         blind_deadline = deadlineIn(lookup_grace_s_);
    const int                    feedback_every = std::max(1, static_cast<int>(cmd_rate_hz_ / 2.0));
    int                          tick           = 0;
    feedback->phase                             = StepClear::Feedback::PHASE_TURNING;
    while (rclcpp::ok() && !stopping_.load())
    {
        if (handle->is_canceling())
        {
            publish(0.0, 0.0, 0.0);
            result->message = feedback->phase + ": cancelled";
            handle->canceled(result);
            return;
        }
        if (std::chrono::steady_clock::now() > deadline)
        {
            fail("timed out");
            return;
        }
        const auto around = surroundings();
        if (!around)
        {
            // As in the approach: stop, and give the costmap and TF lookup_grace_s to return.
            publish(0.0, 0.0, 0.0);
            if (std::chrono::steady_clock::now() > blind_deadline)
            {
                fail("no costmap or base pose");
                return;
            }
            std::this_thread::sleep_for(tickPeriod());
            continue;
        }
        blind_deadline      = deadlineIn(lookup_grace_s_);
        resolution          = around->resolution;
        const double clear  = clearanceAt(around->obstacles, around->x, around->y);
        result->clearance_m = clear;

        if (!plan)
        {
            plan = planStepClear(
                around->obstacles,
                around->x,
                around->y,
                around->yaw,
                goal->clearance_m,
                step_limits_);
            if (!plan->found)
            {
                fail(
                    "boxed in: no straight step of up to " +
                    std::to_string(step_limits_.max_step_m) + " m gets " +
                    std::to_string(goal->clearance_m) + " m clear");
                return;
            }
            if (plan->distance_m == 0.0)
            {
                result->success = true;
                result->message = "already clear";
                handle->succeed(result);
                return;
            }
            heading = wrap(plan->direction + (plan->reverse ? std::numbers::pi : 0.0));
            floor   = std::min(step_limits_.body_radius_m, clear);
            feedback->clearance_m = clear;
            handle->publish_feedback(feedback);
            RCLCPP_INFO(
                get_logger(),
                "%.2f m from an obstacle: stepping %s %.2f m toward %.0f deg",
                clear,
                plan->reverse ? "back" : "forward",
                plan->distance_m,
                plan->direction * 180.0 / std::numbers::pi);
        }

        const double error    = wrap(heading - around->yaw);
        const double yaw_rate = std::clamp(
            gait_.yaw_rate_per_rad * error,
            -gait_.max_yaw_rate_rps,
            gait_.max_yaw_rate_rps);
        if (!step_start)
        {
            if (std::abs(error) > step_heading_tolerance_rad_)
            {
                publish(0.0, 0.0, yaw_rate);
                std::this_thread::sleep_for(tickPeriod());
                continue;
            }
            step_start      = ObstaclePoint{ around->x, around->y };
            feedback->phase = StepClear::Feedback::PHASE_STEPPING;
        }
        const double stepped = std::hypot(around->x - step_start->x, around->y - step_start->y);
        if (clear >= goal->clearance_m || stepped >= plan->distance_m)
        {
            break;
        }
        // Something the plan did not know about: the LiDAR marks cells as they come in view.
        if (clear < floor - around->resolution)
        {
            fail("an obstacle came nearer than planned");
            return;
        }
        publish(plan->reverse ? -step_speed_mps_ : step_speed_mps_, 0.0, yaw_rate);
        if (tick++ % feedback_every == 0)
        {
            feedback->clearance_m = clear;
            handle->publish_feedback(feedback);
        }
        std::this_thread::sleep_for(tickPeriod());
    }
    settle();

    if (const auto after = surroundings())
    {
        result->clearance_m = clearanceAt(after->obstacles, after->x, after->y);
    }
    // Within a cell of the wanted clearance is as close as the costmap can say.
    result->success = result->clearance_m + resolution >= goal->clearance_m;
    result->message = result->success ?
                          "stepped " + std::to_string(result->clearance_m) + " m clear" :
                          feedback->phase + ": only " + std::to_string(result->clearance_m) +
                              " m clear of " + std::to_string(goal->clearance_m);
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
