#ifndef G1_ORCHESTRATION__ROBOT_HEALTH_HPP_
#define G1_ORCHESTRATION__ROBOT_HEALTH_HPP_

/**
 * @file robot_health.hpp
 * @brief Whether the robot can move now: upright, with its legs held by the controller that
 *        balances it.
 *
 * A humanoid that has fallen, or whose balance policy has handed the legs to a freeze, must not
 * be sent walking: the walk would fail as something else (Nav2 waiting for an odometry that never
 * comes) or push a robot that cannot catch itself. The executor refuses such missions and says
 * why, and RobotState carries the reason to the agent.
 */

#include <chrono>
#include <cmath>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <limits>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <set>
#include <string>
#include <vector>

namespace g1_orchestration
{

/// What the readings say about moving.
struct Health
{
    /// Within the tilt limit of upright; false when the tilt is unknown.
    bool upright = false;
    /// Degrees from upright; NaN when there is no reading.
    double tilt_deg = std::numeric_limits<double>::quiet_NaN();
    /// The active controller that holds the legs, empty when none or unknown.
    std::string legs;
    /// Upright, and the legs held by a controller that balances: the base may move.
    bool can_walk = false;
    /// Upright, and the legs held by any controller: the arms may move in place.
    bool can_stand = false;
    /// Why not, for people and models; empty when it can walk.
    std::string reason;
};

class RobotHealth
{
public:
    struct Params
    {
        /// sensor_msgs/Imu of the body; empty skips the tilt check.
        std::string imu_topic;
        /// An IMU reading older than this counts as none.
        double imu_timeout_s = 1.0;
        double max_tilt_deg  = 45.0;
        /// Controllers that keep the robot balanced while it walks, such as the policy.
        std::vector<std::string> walk_controllers;
        /// Controllers that hold the legs without walking, such as a freeze.
        std::vector<std::string> stand_controllers;
        /// Namespace of controller_manager; empty skips the controller check.
        std::string controller_manager;
        double      poll_s = 1.0;
    };

    RobotHealth(rclcpp::Node& node, Params params, const rclcpp::CallbackGroup::SharedPtr& group);

    /// The robot as the newest readings show it; a reading older than its limit counts as none.
    [[nodiscard]] Health now() const;

    /// Degrees between a body's up axis and the world's, from its orientation quaternion.
    [[nodiscard]] static double tiltDeg(double x, double y, double z, double w);

    /// The decision, from the readings. @p tilt_deg or @p active absent: no reading came, or it
    /// is stale; a check whose source is not configured is passed.
    [[nodiscard]] static Health assess(
        const std::optional<double>& tilt_deg, const std::optional<std::set<std::string>>& active,
        const Params& params);

private:
    void poll();

    Params                                                                   params_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                   imu_sub_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;
    rclcpp::TimerBase::SharedPtr                                             poll_timer_;

    mutable std::mutex                    mutex_;
    std::optional<double>                 tilt_deg_;
    std::chrono::steady_clock::time_point tilt_at_;
    std::optional<std::set<std::string>>  active_;
    std::chrono::steady_clock::time_point active_at_;
    bool                                  asking_ = false;
    std::chrono::steady_clock::time_point asked_at_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__ROBOT_HEALTH_HPP_
