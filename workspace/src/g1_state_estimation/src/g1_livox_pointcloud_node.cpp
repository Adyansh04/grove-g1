/**
 * @file g1_livox_pointcloud_node.cpp
 * @brief Republishes the Livox CustomMsg as a PointCloud2, so both formats exist at once.
 *
 * livox_ros_driver2 emits ONE format per run, chosen by its xfer_format parameter, and the two
 * consumers here disagree about which they want. FAST-LIO needs CustomMsg: it is the only
 * format carrying a per-point timestamp, without which there is nothing to undistort a scan
 * taken while the robot walks. Everything else -- both Nav2 costmaps, MoveIt's octomap, and
 * pointcloud_to_laserscan feeding AMCL -- takes the PointCloud2 on /livox/lidar. So the driver
 * runs in CustomMsg mode and this fills the gap.
 *
 * Deliberately downstream of the driver rather than folded into it: the point cloud everything
 * else depends on then does not go through FAST-LIO, and cannot be taken down by it.
 *
 * HARDWARE ONLY. In simulation the relay already publishes /livox/lidar and g1_livox_bridge
 * runs this conversion in the other direction.
 */

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <utility>

namespace g1_state_estimation
{

class LivoxPointCloud : public rclcpp::Node
{
public:
    LivoxPointCloud()
      : rclcpp::Node("g1_livox_pointcloud")
    {
        // Sensor QoS out, matching what g1_sensor_relay publishes on this topic in simulation
        // so the consumers cannot tell the two apart.
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            declare_parameter<std::string>("cloud_topic", "/livox/lidar"),
            rclcpp::SensorDataQoS());
        // Reliable in, because that is what the driver offers (lddc.cpp CreatePublisher takes
        // a bare queue size). A best-effort subscriber would match, but would also drop scans
        // under load for no reason.
        custom_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            declare_parameter<std::string>("custom_msg_topic", "/livox/custom_msg"),
            rclcpp::QoS(20),
            [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) { onCustom(*msg); });
    }

private:
    void onCustom(const livox_ros_driver2::msg::CustomMsg& custom)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = custom.header;
        cloud.height = 1;
        cloud.width  = static_cast<std::uint32_t>(custom.points.size());
        // Unordered and possibly containing nothing at all, which is what is_dense=false says.
        cloud.is_dense     = false;
        cloud.is_bigendian = false;

        // xyz + intensity, the layout every consumer here already reads. The per-point time
        // and line index are dropped: only FAST-LIO uses them, and it reads the CustomMsg.
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(2, "xyz", "intensity");
        modifier.resize(custom.points.size());

        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");

        for (const auto& point : custom.points)
        {
            *x         = point.x;
            *y         = point.y;
            *z         = point.z;
            *intensity = static_cast<float>(point.reflectivity);
            ++x, ++y, ++z, ++intensity;
        }
        cloud_pub_->publish(std::move(cloud));
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        cloud_pub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_sub_;
};

}  // namespace g1_state_estimation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_state_estimation::LivoxPointCloud>());
    rclcpp::shutdown();
    return 0;
}
