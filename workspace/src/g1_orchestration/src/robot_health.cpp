/**
 * @file robot_health.cpp
 * @brief Whether the robot can move now, from its IMU and its active controllers.
 */

#include "g1_orchestration/robot_health.hpp"

#include <algorithm>
#include <format>
#include <numbers>
#include <utility>

namespace g1_orchestration
{
namespace
{
using Clock = std::chrono::steady_clock;

/// Unanswered polls in a row after which the controllers count as unknown.
constexpr int kPollsStale = 3;

const std::string*
firstActive(const std::vector<std::string>& names, const std::set<std::string>& active)
{
    const auto it = std::ranges::find_if(names, [&](const auto& n) { return active.contains(n); });
    return it == names.end() ? nullptr : &*it;
}
}  // namespace

RobotHealth::RobotHealth(
    rclcpp::Node& node, Params params, const rclcpp::CallbackGroup::SharedPtr& group)
  : params_(std::move(params))
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    if (!params_.imu_topic.empty())
    {
        imu_sub_ = node.create_subscription<sensor_msgs::msg::Imu>(
            params_.imu_topic,
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
                const auto&                       q    = msg->orientation;
                const double                      tilt = tiltDeg(q.x, q.y, q.z, q.w);
                const std::lock_guard<std::mutex> lock(mutex_);
                tilt_deg_ = tilt;
                tilt_at_  = Clock::now();
            },
            options);
    }
    if (!params_.controller_manager.empty())
    {
        list_client_ = node.create_client<controller_manager_msgs::srv::ListControllers>(
            params_.controller_manager + "/list_controllers",
            rclcpp::ServicesQoS(),
            group);
        poll_timer_ = node.create_wall_timer(
            std::chrono::duration<double>(params_.poll_s),
            [this] { poll(); },
            group);
    }
}

void RobotHealth::poll()
{
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        const auto                        at = Clock::now();
        // A reply lost to a restarting controller_manager would otherwise stop the polling.
        if (asking_ && at - asked_at_ > 2 * std::chrono::duration<double>(params_.poll_s))
        {
            list_client_->prune_pending_requests();
            asking_ = false;
        }
        if (asking_ || !list_client_->service_is_ready())
        {
            return;
        }
        asking_   = true;
        asked_at_ = at;
    }
    list_client_->async_send_request(
        std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),
        // rclcpp matches the callback signature exactly, so the future is taken by value.
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
            std::set<std::string> active;
            for (const auto& controller : future.get()->controller)
            {
                if (controller.state == "active")
                {
                    active.insert(controller.name);
                }
            }
            const std::lock_guard<std::mutex> lock(mutex_);
            active_    = std::move(active);
            active_at_ = Clock::now();
            asking_    = false;
        });
}

Health RobotHealth::now() const
{
    std::optional<double>                tilt;
    std::optional<std::set<std::string>> active;
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        const auto                        at = Clock::now();
        if (tilt_deg_ && at - tilt_at_ <= std::chrono::duration<double>(params_.imu_timeout_s))
        {
            tilt = tilt_deg_;
        }
        if (active_ &&
            at - active_at_ <= kPollsStale * std::chrono::duration<double>(params_.poll_s))
        {
            active = active_;
        }
    }
    return assess(tilt, active, params_);
}

double RobotHealth::tiltDeg(double x, double y, double z, double w)
{
    // The body's z axis in the world is the third column of the rotation; its z component is
    // the cosine of the tilt. Normalised, since a filter's quaternion drifts off unit length.
    const double norm2 = x * x + y * y + z * z + w * w;
    if (norm2 <= 0.0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double up = std::clamp(1.0 - 2.0 * (x * x + y * y) / norm2, -1.0, 1.0);
    return std::acos(up) * 180.0 / std::numbers::pi;
}

Health RobotHealth::assess(
    const std::optional<double>& tilt_deg, const std::optional<std::set<std::string>>& active,
    const Params& params)
{
    Health health;
    if (params.imu_topic.empty())
    {
        health.upright = true;
    }
    else if (tilt_deg && std::isfinite(*tilt_deg))
    {
        health.tilt_deg = *tilt_deg;
        health.upright  = *tilt_deg <= params.max_tilt_deg;
    }

    bool walks  = params.controller_manager.empty();
    bool stands = walks;
    if (!params.controller_manager.empty() && active)
    {
        const std::string* walker = firstActive(params.walk_controllers, *active);
        const std::string* holder =
            walker != nullptr ? walker : firstActive(params.stand_controllers, *active);
        health.legs = holder != nullptr ? *holder : "";
        walks       = walker != nullptr;
        stands      = holder != nullptr;
    }
    health.can_walk  = health.upright && walks;
    health.can_stand = health.upright && stands;

    if (!health.upright)
    {
        health.reason = std::isfinite(health.tilt_deg) ?
                            std::format("fallen: tilted {:.0f} degrees", health.tilt_deg) :
                            "no tilt reading from " + params.imu_topic;
    }
    else if (!params.controller_manager.empty() && !active)
    {
        health.reason = params.controller_manager + " does not answer";
    }
    else if (!stands)
    {
        health.reason = "no controller holds the legs";
    }
    else if (!walks)
    {
        health.reason = "the balance policy is not running (" + health.legs + " holds the legs)";
    }
    return health;
}

}  // namespace g1_orchestration
