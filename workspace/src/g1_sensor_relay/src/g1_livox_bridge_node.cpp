/**
 * @file g1_livox_bridge_node.cpp
 * @brief Makes the simulator's sensors look like a Livox Mid360 to FAST-LIO.
 *
 * FAST-LIO does not consume what the rest of the stack consumes. It wants the Livox CustomMsg,
 * because that is the only format carrying a per-point timestamp, and it wants an IMU stream
 * beside it. On the robot both come out of livox_ros_driver2. In simulation there is no driver:
 * the sweep arrives as a PointCloud2 from g1_sensor_relay and the IMU lives inside
 * unitree_hg::LowState. This node restates both in the driver's own formats so the odometry
 * pipeline below it is identical in either place.
 *
 * One thing it cannot restate: on the robot the IMU FAST-LIO fuses sits INSIDE the Mid360, so
 * the lidar-to-IMU extrinsic is a constant machined into the housing. MuJoCo models no IMU
 * there, so the pelvis IMU stands in for it -- and the pelvis is three actuated waist joints
 * away from the sensor, all of which the walking policy drives (up to 25 deg of yaw and 16 deg
 * of pitch, measured). A constant extrinsic is simply wrong across that chain. So the sweep is
 * rotated into the IMU's own frame here, per scan, using the waist state TF already carries;
 * FAST-LIO then gets a rigid pair and an identity extrinsic, which is the situation the robot
 * gives it for free.
 *
 * SIMULATION ONLY. On hardware the real driver publishes these topics and this node must not
 * run -- two publishers on /livox/custom_msg would interleave scans from different sources.
 */

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unitree_hg/msg/low_state.hpp>
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
        // RELIABLE, with the same depths livox_ros_driver2 uses (lddc.cpp CreatePublisher
        // passes a bare queue size, which is reliable by default). Not a style choice:
        // FAST-LIO subscribes to both of these reliably, and a best-effort publisher is
        // silently incompatible with that -- DDS drops the match and logs one warning about
        // RELIABILITY_QOS_POLICY that is easy to read past.
        custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
            declare_parameter<std::string>("custom_msg_topic", "/livox/custom_msg"),
            rclcpp::QoS(20));
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            declare_parameter<std::string>("imu_topic", "/livox/imu"),
            rclcpp::QoS(10));

        // The inputs keep sensor QoS: these are the simulator's own streams, published
        // best-effort by g1_sensor_relay and the DDS bridge.
        const auto sensor_qos = rclcpp::SensorDataQoS();
        cloud_sub_            = create_subscription<sensor_msgs::msg::PointCloud2>(
            declare_parameter<std::string>("cloud_topic", "/livox/lidar"),
            sensor_qos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(*msg); });
        // Depth 200, not SensorDataQoS's 5. LowState arrives near 1 kHz and the sweep callback
        // beside it takes tens of milliseconds, during which a five-deep queue throws away
        // ~45 samples. FAST-LIO integrates gyro across whatever gap that leaves, so the loss
        // comes back as heading error at exactly the moments the robot is turning.
        low_state_sub_ = create_subscription<unitree_hg::msg::LowState>(
            declare_parameter<std::string>("low_state_topic", "/lowstate"),
            rclcpp::QoS(200).best_effort(),
            [this](unitree_hg::msg::LowState::ConstSharedPtr msg) { onLowState(*msg); });

        imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "pelvis");
        imu_period_s_ = 1.0 / declare_parameter<double>("imu_rate_hz", 200.0);
        // The frame the sweep is restated in, which must be the frame the IMU above reports in.
        // Empty passes the cloud through untouched, for a sim that one day models an IMU in the
        // sensor itself.
        cloud_target_frame_ = declare_parameter<std::string>("cloud_target_frame", imu_frame_id_);
        if (!cloud_target_frame_.empty())
        {
            tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        }
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

        tf2::Matrix3x3 rotation = tf2::Matrix3x3::getIdentity();
        tf2::Vector3   offset(0.0, 0.0, 0.0);
        if (!cloud_target_frame_.empty() && !lookUpSensorPose(cloud.header, rotation, offset))
        {
            return;
        }

        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");

        livox_ros_driver2::msg::CustomMsg msg;
        msg.header = cloud.header;
        if (!cloud_target_frame_.empty())
        {
            msg.header.frame_id = cloud_target_frame_;
        }
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
            const tf2::Vector3 p = rotation * tf2::Vector3(*x, *y, *z) + offset;
            livox_ros_driver2::msg::CustomPoint point;
            point.x = static_cast<float>(p.x());
            point.y = static_cast<float>(p.y());
            point.z = static_cast<float>(p.z());
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

    /// Pose of the sweep's own frame in cloud_target_frame_, at the instant the sweep describes.
    bool lookUpSensorPose(const std_msgs::msg::Header& header,
                          tf2::Matrix3x3&              rotation,
                          tf2::Vector3&                offset)
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            // Stamped, not TimePointZero: the waist swings tens of degrees per gait cycle and
            // the relay deliberately stamps clouds ~35 ms before they arrive, so "newest
            // available" would pair this sweep with a waist state it never had.
            tf = tf_buffer_->lookupTransform(cloud_target_frame_, header.frame_id, header.stamp);
        }
        catch (const tf2::ExtrapolationException&)
        {
            // /joint_states runs near 300 Hz, so TF trails the sweep's stamp by a few
            // milliseconds now and then and the exact instant is not in the buffer yet. Waiting
            // for it would block this callback and starve the 1 kHz IMU relay beside it, and
            // dropping the sweep costs a whole 10 Hz scan. A waist angle a few milliseconds
            // stale is the cheapest of the three.
            try
            {
                tf = tf_buffer_->lookupTransform(
                    cloud_target_frame_, header.frame_id, tf2::TimePointZero);
            }
            catch (const tf2::TransformException& e)
            {
                warnNoTransform(header, e);
                return false;
            }
        }
        catch (const tf2::TransformException& e)
        {
            warnNoTransform(header, e);
            return false;
        }

        rotation.setRotation(tf2::Quaternion{ tf.transform.rotation.x,
                                              tf.transform.rotation.y,
                                              tf.transform.rotation.z,
                                              tf.transform.rotation.w });
        offset = tf2::Vector3{ tf.transform.translation.x,
                               tf.transform.translation.y,
                               tf.transform.translation.z };
        return true;
    }

    void warnNoTransform(const std_msgs::msg::Header& header, const tf2::TransformException& e)
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Dropping sweeps: no %s -> %s transform (%s).",
                             cloud_target_frame_.c_str(),
                             header.frame_id.c_str(),
                             e.what());
    }

    void onLowState(const unitree_hg::msg::LowState& state)
    {
        // LowState arrives near 1 kHz; a Mid360 publishes its IMU at 200 Hz. Relaying the full
        // rate is not generosity, it is a lie about what the robot will hand FAST-LIO -- and an
        // expensive one, because FAST-LIO does its whole update inside one timer callback on a
        // single-threaded executor and takes no IMU while it runs. Its queue holds 10 samples:
        // 50 ms at the real rate, 11 ms at 1 kHz. Past that the samples are simply gone, and
        // FAST-LIO integrates the gap on the last reading it saw, which while turning is
        // straight heading error whose size tracks how busy the machine was.
        const rclcpp::Time arrival = now();
        if (have_imu_stamp_ && (arrival - last_imu_stamp_).seconds() < imu_period_s_)
        {
            return;
        }
        last_imu_stamp_  = arrival;
        have_imu_stamp_  = true;

        sensor_msgs::msg::Imu imu;
        // The simulator publishes no /clock and stamps nothing itself, so arrival time is the
        // only stamp available. LowState comes over loopback DDS, so this is within a
        // millisecond or two of the instant it describes -- small beside the 100 ms between
        // sweeps, and the sweeps carry the simulator's own capture time (see g1_sensor_relay).
        imu.header.stamp    = arrival;
        imu.header.frame_id = imu_frame_id_;

        // Unitree order the quaternion w-first.
        imu.orientation.w = state.imu_state.quaternion[0];
        imu.orientation.x = state.imu_state.quaternion[1];
        imu.orientation.y = state.imu_state.quaternion[2];
        imu.orientation.z = state.imu_state.quaternion[3];

        imu.angular_velocity.x = state.imu_state.gyroscope[0];
        imu.angular_velocity.y = state.imu_state.gyroscope[1];
        imu.angular_velocity.z = state.imu_state.gyroscope[2];

        // Proper acceleration, gravity included, which is what MuJoCo's accelerometer sensor
        // reports and what a real IMU reads. FAST-LIO normalises by the measured magnitude
        // during its init, so the units only have to be self-consistent.
        imu.linear_acceleration.x = state.imu_state.accelerometer[0];
        imu.linear_acceleration.y = state.imu_state.accelerometer[1];
        imu.linear_acceleration.z = state.imu_state.accelerometer[2];

        imu_pub_->publish(std::move(imu));
    }

    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             imu_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_sub_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr      low_state_sub_;
    std::string                                                     imu_frame_id_;
    std::string                                                     cloud_target_frame_;
    double                                                          imu_period_s_{};
    rclcpp::Time                                                    last_imu_stamp_;
    bool                                                            have_imu_stamp_{ false };
    std::unique_ptr<tf2_ros::Buffer>                                tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener>                     tf_listener_;
};

}  // namespace g1_sensor_relay

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_sensor_relay::LivoxBridge>());
    rclcpp::shutdown();
    return 0;
}
