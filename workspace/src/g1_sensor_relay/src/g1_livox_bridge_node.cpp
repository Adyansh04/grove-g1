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
 * too, and g1_sensor_relay publishes /livox/imu straight off the sensor socket. It used to be the
 * pelvis IMU relayed out of LowState, which does not work on this robot: three actuated waist
 * joints lie between pelvis and sensor and the walking policy drives them through tens of degrees,
 * so the one constant lidar-to-IMU extrinsic FAST-LIO takes was wrong by a different amount every
 * scan and the scan match came apart whenever the robot turned.
 *
 * SIMULATION ONLY. On hardware the real driver publishes this topic and this node must not run --
 * two publishers on /livox/custom_msg would interleave scans from different sources.
 */

#include <cmath>
#include <cstdint>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <utility>

namespace g1_sensor_relay
{
namespace
{
/// Whether the three fields the iterators below will ask for are actually present as floats.
bool hasXyz(const sensor_msgs::msg::PointCloud2& cloud)
{
    int found = 0;
    for (const auto& field : cloud.fields)
    {
        if ((field.name == "x" || field.name == "y" || field.name == "z") &&
            field.datatype == sensor_msgs::msg::PointField::FLOAT32)
        {
            ++found;
        }
    }
    return found == 3;
}
}  // namespace

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
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(*msg); });
    }

private:
    void onCloud(const sensor_msgs::msg::PointCloud2& cloud)
    {
        // The iterators throw when a field is missing, and an exception out of a subscription
        // callback takes the whole node down. Checked instead, so a cloud from something other
        // than the relay is a warning rather than a dead bridge.
        if (!hasXyz(cloud))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Ignoring a cloud on %s without x/y/z float fields.",
                cloud_sub_->get_topic_name());
            return;
        }

        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");

        livox_ros_driver2::msg::CustomMsg msg;
        msg.header = cloud.header;
        // Informational only -- FAST-LIO times scans off header.stamp -- but a real driver puts
        // the first point's absolute time here, so match that rather than leave it zero.
        msg.timebase = static_cast<std::uint64_t>(rclcpp::Time(cloud.header.stamp).nanoseconds());
        msg.lidar_id = 0;
        msg.points.reserve(cloud.width * cloud.height);

        for (; x != x.end(); ++x, ++y, ++z)
        {
            // Misses come through as non-finite. Dropping them here rather than passing them on
            // keeps point_num honest, and FAST-LIO's own range gate would discard them anyway.
            if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
            {
                continue;
            }
            livox_ros_driver2::msg::CustomPoint point;
            point.x = *x;
            point.y = *y;
            point.z = *z;
            // Zero, and correct rather than merely convenient. The simulator raycasts against a
            // frozen mjData, so every point in a frame really is sampled at the same instant.
            // FAST-LIO reads this as milliseconds-since-scan-start into its motion
            // undistortion, which then finds nothing to undo -- which is the truth here.
            point.offset_time = 0;
            // Both are gates in FAST-LIO's Livox handler, not decoration: `line` must be under
            // scan_line, and tag bits 4-5 must read 00 or 01 or the point is discarded.
            point.line = 0;
            point.tag  = 0;
            // The sweep carries no intensity, and nothing downstream registers on it.
            point.reflectivity = 0;
            msg.points.push_back(point);
        }

        msg.point_num = static_cast<std::uint32_t>(msg.points.size());
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
