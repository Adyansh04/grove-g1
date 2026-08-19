/**
 * @file livox_custom_msg.cpp
 * @brief The PointCloud2 -> Livox CustomMsg conversion FAST-LIO consumes in simulation.
 */

#include "g1_sensor_relay/livox_custom_msg.hpp"

#include <cmath>
#include <cstdint>
#include <rclcpp/time.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <utility>

namespace g1_sensor_relay
{

bool hasXyzFloatFields(const sensor_msgs::msg::PointCloud2& cloud)
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

bool toCustomMsg(const sensor_msgs::msg::PointCloud2& cloud, livox_ros_driver2::msg::CustomMsg& out)
{
    // The iterators throw when a field is missing, and an exception out of a subscription
    // callback takes the whole node down.
    if (!hasXyzFloatFields(cloud))
    {
        return false;
    }

    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");

    out.header = cloud.header;
    // Informational only, since FAST-LIO times scans off header.stamp, but a real driver puts the
    // first point's absolute time here, so match that rather than leave it zero.
    out.timebase = static_cast<std::uint64_t>(rclcpp::Time(cloud.header.stamp).nanoseconds());
    out.lidar_id = 0;
    out.points.clear();
    out.points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);

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
        // FAST-LIO reads this as milliseconds-since-scan-start into its motion undistortion,
        // which then finds nothing to undo, which is the truth here.
        point.offset_time = 0;
        // Both are gates in FAST-LIO's Livox handler, not decoration: `line` must be under
        // scan_line, and tag bits 4-5 must read 00 or 01 or the point is discarded.
        point.line = 0;
        point.tag  = 0;
        // The sweep carries no intensity, and nothing downstream registers on it.
        point.reflectivity = 0;
        out.points.push_back(point);
    }

    out.point_num = static_cast<std::uint32_t>(out.points.size());
    return true;
}

}  // namespace g1_sensor_relay
