#ifndef G1_ORCHESTRATION__TELEOP_DRIVER_HPP_
#define G1_ORCHESTRATION__TELEOP_DRIVER_HPP_

/**
 * @file teleop_driver.hpp
 * @brief Drives the base from an operator's velocity commands: the executor's teleop mode.
 *
 * The executor is the only thing that moves the robot, so driving by hand goes through it too: it
 * holds the base while teleop is on and refuses missions meanwhile. The commands go where Nav2's
 * go, the collision monitor's input, so the monitor still stops the robot short of what it sees.
 * They are clamped to limits and ramped from the last output (ramp, don't snap), and a pause in
 * them longer than the deadman brings the base to a stop.
 */

#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace g1_orchestration
{

/// A planar base velocity: forward and left in m/s, yaw rate in rad/s.
struct Velocity
{
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;

    [[nodiscard]] bool isZero() const { return x == 0.0 && y == 0.0 && yaw == 0.0; }
};

class TeleopDriver
{
public:
    struct Params
    {
        /// Where the base's velocity goes: the collision monitor's input.
        std::string command_topic;
        double      max_x     = 0.5;
        double      max_y     = 0.3;
        double      max_yaw   = 0.8;
        double      accel_xy  = 1.0;
        double      accel_yaw = 2.0;
        /// A pause in commands longer than this brings the base to a stop.
        double deadman_s = 0.4;
        /// Teleop ends after this long without a command.
        double idle_s  = 20.0;
        double rate_hz = 20.0;
    };

    /// @param on_idle Called, from the driver's timer, when teleop ends for want of commands.
    TeleopDriver(
        rclcpp::Node& node, Params params, const rclcpp::CallbackGroup::SharedPtr& group,
        std::function<void()> on_idle);

    void enable();
    /// Brings the base to a stop at the deceleration limit and ends teleop.
    void               disable(const std::string& why);
    [[nodiscard]] bool active() const;

    /// One control period: the output moved toward @p target (zero when it is stale), within
    /// the speed limits and no faster than the acceleration limits allow over @p dt.
    [[nodiscard]] static Velocity
    step(const Velocity& current, const Velocity& target, double dt, const Params& params);

private:
    void tick();
    void publish(const Velocity& velocity);

    Params                                                     params_;
    rclcpp::Logger                                             logger_;
    std::function<void()>                                      on_idle_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr                               timer_;

    mutable std::mutex                    mutex_;
    bool                                  active_   = false;
    bool                                  stopping_ = false;
    Velocity                              commanded_;
    Velocity                              output_;
    std::chrono::steady_clock::time_point commanded_at_;
    std::chrono::steady_clock::time_point ticked_at_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__TELEOP_DRIVER_HPP_
