/**
 * @file g1_gait_shaper_node.cpp
 * @brief Node wrapper around GaitShaper: cmd_vel_in -> cmd_vel_out.
 *
 * A stateless transform on a topic, so a plain node rather than a lifecycle one -- there is
 * nothing to activate. All the logic worth testing is in GaitShaper, which needs no ROS.
 */
#include <memory>
#include <stdexcept>

#include "g1_locomotion/gait_shaper.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace g1_locomotion
{

class G1GaitShaperNode : public rclcpp::Node
{
public:
    explicit G1GaitShaperNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("g1_gait_shaper", options)
      , shaper_config_(declareConfig())
      , shaper_(shaper_config_)
    {
        // Command QoS, matched to both ends: the planner's cmd_vel publisher and the bridge's
        // ~/cmd_vel subscription. Never sensor-data here -- a best-effort control topic silently
        // drops the command that was meant to stop the robot.
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        publisher_     = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", qos);
        subscription_  = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_in",
            qos,
            [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) { onCommand(*msg); });

        RCLCPP_INFO(
            get_logger(),
            "Shaping %s -> %s. Below %.2f m/s forward or %.2f rad/s yaw the gait does not step, "
            "so anything smaller is passed through as a stop.",
            subscription_->get_topic_name(),
            publisher_->get_topic_name(),
            shaper_config_.fwd_engage,
            shaper_config_.yaw_engage);
    }

private:
    /// Declares the parameters and returns the config. Touches no member, so the two
    /// initialisers above cannot depend on declaration order.
    GaitShaper::Config declareConfig()
    {
        const GaitShaper::Config defaults;
        const GaitShaper::Config config{
            declare_parameter("fwd_engage", defaults.fwd_engage),
            declare_parameter("yaw_engage", defaults.yaw_engage),
            declare_parameter("yaw_clamp", defaults.yaw_clamp),
        };
        // The never-amplifies invariant is a property of the class AND its configuration, not
        // of the class alone. A negative yaw_clamp turns the clamp into a multiplier, and a
        // negative fwd_engage lets reverse through -- removing the backstop the navigation
        // trees rely on. Neither is reachable from the shipped YAML, and neither should be
        // reachable from any YAML.
        if (config.fwd_engage < 0.0 || config.yaw_engage <= 0.0 || config.yaw_clamp < 0.0)
        {
            throw std::invalid_argument(
                "g1_gait_shaper: fwd_engage and yaw_clamp must be >= 0 and yaw_engage > 0; "
                "negative thresholds would let this node amplify a command instead of "
                "reducing it");
        }
        return config;
    }

    void onCommand(const geometry_msgs::msg::Twist& msg)
    {
        const GaitShaper::Command in{ msg.linear.x, msg.linear.y, msg.angular.z };
        const GaitShaper::Command out = shaper_.shape(in);

        geometry_msgs::msg::Twist shaped;
        shaped.linear.x  = out.vx;
        shaped.linear.y  = out.vy;
        shaped.angular.z = out.vyaw;
        publisher_->publish(shaped);

        // DEBUG, not WARN: dropping below the threshold is what every successful goal does on
        // its final approach, and a warning there would cry wolf on the normal case. The useful
        // diagnostic is echoing both topics side by side, which is half of why this is its own
        // node.
        const bool asked   = in.vx != 0.0 || in.vy != 0.0 || in.vyaw != 0.0;
        const bool stopped = out.vx == 0.0 && out.vy == 0.0 && out.vyaw == 0.0;
        if (asked && stopped)
        {
            RCLCPP_DEBUG_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Stopping on (%.3f, %.3f, %.3f): below the gait's engage thresholds.",
                in.vx,
                in.vy,
                in.vyaw);
        }
    }

    const GaitShaper::Config                                   shaper_config_;
    const GaitShaper                                           shaper_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

}  // namespace g1_locomotion

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_locomotion::G1GaitShaperNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
