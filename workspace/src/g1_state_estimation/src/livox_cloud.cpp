/**
 * @file livox_cloud.cpp
 * @brief The hardware Livox CustomMsg -> PointCloud2 conversion.
 */

#include "g1_state_estimation/livox_cloud.hpp"

#include <cstdint>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace g1_state_estimation
{

void toPointCloud2(
    const livox_ros_driver2::msg::CustomMsg& custom, sensor_msgs::msg::PointCloud2& cloud)
{
    cloud.header = custom.header;
    cloud.height = 1;
    cloud.width  = static_cast<std::uint32_t>(custom.points.size());
    // Unordered, and a sweep can legitimately return nothing: that is what is_dense=false says.
    cloud.is_dense     = false;
    cloud.is_bigendian = false;

    // Fields spelled out rather than setPointCloud2FieldsByString: that helper only knows
    // "xyz", "rgb" and "rgba", and throws on anything else, including "intensity", which is
    // the field every consumer of /livox/lidar reads.
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
        ++x;
        ++y;
        ++z;
        ++intensity;
    }
}

}  // namespace g1_state_estimation
