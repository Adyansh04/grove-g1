/**
 * @file livox_cloud.cpp
 * @brief The hardware Livox CustomMsg -> PointCloud2 conversion.
 */

#include "g1_state_estimation/livox_cloud.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace g1_state_estimation
{

void toPointCloud2(
    const livox_ros_driver2::msg::CustomMsg& custom, sensor_msgs::msg::PointCloud2& cloud,
    double min_range_m)
{
    const auto min_sq   = static_cast<float>(min_range_m * min_range_m);
    const auto in_range = [min_sq](const livox_ros_driver2::msg::CustomPoint& point) {
        return (point.x * point.x) + (point.y * point.y) + (point.z * point.z) >= min_sq;
    };
    const auto kept = static_cast<std::size_t>(std::ranges::count_if(custom.points, in_range));

    cloud.header = custom.header;
    cloud.height = 1;
    cloud.width  = static_cast<std::uint32_t>(kept);
    // The driver's points carry no NaN, but nothing here checks, so is_dense stays false.
    cloud.is_dense     = false;
    cloud.is_bigendian = false;

    // Spelled out: setPointCloud2FieldsByString knows only xyz, rgb and rgba, not intensity.
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        4,
        "x",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "y",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "z",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "intensity",
        1,
        sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(kept);

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");

    for (const auto& point : custom.points)
    {
        if (!in_range(point))
        {
            continue;
        }
        *x         = point.x;
        *y         = point.y;
        *z         = point.z;
        *intensity = static_cast<float>(point.reflectivity);
        ++x;
        ++y;
        ++z;
        ++intensity;
    }
}

}  // namespace g1_state_estimation
