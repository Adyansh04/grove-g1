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
        // Validation belongs to GaitShaper's constructor, not here: shape() is what depends on
        // the bounds, and a check in this one caller left every other construction path free to
        // hand the class a config its body cannot handle. The throw still surfaces the same
        // way, from shaper_'s initialiser, and main() catches it.
        const GaitShaper::Config defaults;
        return GaitShaper::Config{
            declare_parameter("fwd_engage", defaults.fwd_engage),
            declare_parameter("yaw_engage", defaults.yaw_engage),
            declare_parameter("yaw_clamp", defaults.yaw_clamp),
        };
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
    try
    {
        rclcpp::spin(std::make_shared<g1_locomotion::G1GaitShaperNode>(rclcpp::NodeOptions()));
    }
    catch (const std::invalid_argument& e)
    {
        // Failing closed is right -- no shaper means no cmd_vel reaches the bridge -- but an
        // uncaught throw exits -6 through std::terminate, which reads as a crash rather than a
        // rejected parameter.
        RCLCPP_FATAL(rclcpp::get_logger("g1_gait_shaper"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
