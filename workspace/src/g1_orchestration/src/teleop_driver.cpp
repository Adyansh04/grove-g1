/**
 * @file teleop_driver.cpp
 * @brief The executor's teleop mode: operator velocity commands to the base, limited and ramped.
 */

#include "g1_orchestration/teleop_driver.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace g1_orchestration
{
namespace
{
using Clock = std::chrono::steady_clock;

/// @p value within ±@p limit; a value that is not a number is no command at all.
double bounded(double value, double limit)
{
    return std::isfinite(value) ? std::clamp(value, -limit, limit) : 0.0;
}

/// From @p from toward @p to by at most @p max_delta.
double approach(double from, double to, double max_delta)
{
    return from + std::clamp(to - from, -max_delta, max_delta);
}
}  // namespace

TeleopDriver::TeleopDriver(
    rclcpp::Node& node, Params params, const rclcpp::CallbackGroup::SharedPtr& group,
    std::function<void()> on_idle)
  : params_(std::move(params))
  , logger_(node.get_logger())
  , on_idle_(std::move(on_idle))
{
    // Commands are reliable, and only the newest matters.
    const auto qos = rclcpp::QoS(1).reliable();
    publisher_     = node.create_publisher<geometry_msgs::msg::Twist>(params_.command_topic, qos);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    subscription_          = node.create_subscription<geometry_msgs::msg::Twist>(
        "~/teleop_cmd",
        qos,
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
            const std::lock_guard<std::mutex> lock(mutex_);
            if (active_)
            {
                commanded_    = { msg->linear.x, msg->linear.y, msg->angular.z };
                commanded_at_ = Clock::now();
            }
        },
        options);
    timer_ = node.create_wall_timer(
        std::chrono::duration<double>(1.0 / params_.rate_hz),
        [this] { tick(); },
        group);
}

void TeleopDriver::enable()
{
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        active_       = true;
        stopping_     = false;
        commanded_    = {};
        commanded_at_ = Clock::now();
        ticked_at_    = commanded_at_;
    }
    RCLCPP_INFO(logger_, "teleop on: commands on ~/teleop_cmd drive the base");
}

void TeleopDriver::disable(const std::string& why)
{
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (!active_)
        {
            return;
        }
        active_   = false;
        stopping_ = true;
    }
    RCLCPP_INFO(logger_, "teleop off: %s", why.c_str());
}

bool TeleopDriver::active() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return active_;
}

Velocity
TeleopDriver::step(const Velocity& current, const Velocity& target, double dt, const Params& params)
{
    const double xy  = params.accel_xy * dt;
    const double yaw = params.accel_yaw * dt;
    return { approach(current.x, bounded(target.x, params.max_x), xy),
             approach(current.y, bounded(target.y, params.max_y), xy),
             approach(current.yaw, bounded(target.yaw, params.max_yaw), yaw) };
}

void TeleopDriver::tick()
{
    bool     ended = false;
    Velocity out;
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ && !stopping_)
        {
            return;
        }
        const auto   at    = Clock::now();
        const double dt    = std::chrono::duration<double>(at - ticked_at_).count();
        ticked_at_         = at;
        const double quiet = std::chrono::duration<double>(at - commanded_at_).count();
        if (active_ && quiet > params_.idle_s)
        {
            active_   = false;
            stopping_ = true;
            ended     = true;
        }
        const Velocity target = active_ && quiet <= params_.deadman_s ? commanded_ : Velocity{};
        output_               = step(output_, target, dt, params_);
        // Once stopped, the topic is left to Nav2: two writers there would fight.
        if (stopping_ && output_.isZero())
        {
            stopping_ = false;
        }
        out = output_;
    }
    publish(out);
    if (ended)
    {
        RCLCPP_INFO(logger_, "teleop off: no command for %.0f s", params_.idle_s);
        on_idle_();
    }
}

void TeleopDriver::publish(const Velocity& velocity)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = velocity.x;
    twist.linear.y  = velocity.y;
    twist.angular.z = velocity.yaw;
    publisher_->publish(twist);
}

}  // namespace g1_orchestration
