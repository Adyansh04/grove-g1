/**
 * @file g1_livox_bridge_node.cpp
 * @brief Restates the simulator's sweep as the Livox CustomMsg FAST-LIO consumes.
 *
 * FAST-LIO does not consume what the rest of the stack consumes. It wants the CustomMsg, because
 * that is the only format carrying a per-point timestamp. On the robot livox_ros_driver2 produces
 * it; in simulation the sweep arrives as a PointCloud2 from g1_sensor_relay and this node converts
 * it, so the odometry pipeline below is identical in either place.
 *
 * The cloud only. The IMU FAST-LIO fuses sits inside the Mid360, the simulator models it there
 * too, and g1_sensor_relay publishes /livox/imu straight off the sensor socket. See
 * g1_state_estimation's README for why it is not the pelvis IMU.
 *
 * SIMULATION ONLY. On hardware the real driver publishes this topic and this node must not run --
 * two publishers on /livox/custom_msg would interleave scans from different sources.
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <utility>

#include "g1_sensor_relay/livox_custom_msg.hpp"

namespace g1_sensor_relay
{

class LivoxBridge : public rclcpp::Node
{
public:
    LivoxBridge()
      : rclcpp::Node("g1_livox_bridge")
    {
        // RELIABLE, with the depth livox_ros_driver2 uses (lddc.cpp CreatePublisher passes a
        // bare queue size, which is reliable by default). Not a style choice: FAST-LIO
        // subscribes reliably, and a best-effort publisher is silently incompatible with that
        // -- DDS drops the match and logs one warning about RELIABILITY_QOS_POLICY that is easy
        // to read past.
        custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
            declare_parameter<std::string>("custom_msg_topic", "/livox/custom_msg"),
            rclcpp::QoS(20));

        // Sensor QoS on the input: this is the simulator's own stream, published best-effort.
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            declare_parameter<std::string>("cloud_topic", "/livox/lidar"),
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) { onCloud(*msg); });
    }

private:
    void onCloud(const sensor_msgs::msg::PointCloud2& cloud)
    {
        auto msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
        if (!toCustomMsg(cloud, *msg))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Ignoring a cloud on %s without x/y/z float fields.",
                cloud_sub_->get_topic_name());
            return;
        }
        custom_pub_->publish(std::move(msg));
    }

    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_sub_;
};

}  // namespace g1_sensor_relay

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_sensor_relay::LivoxBridge>());
    rclcpp::shutdown();
    return 0;
}
