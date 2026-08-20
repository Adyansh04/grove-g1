/**
 * @file test_livox_cloud.cpp
 * @brief The hardware CustomMsg -> PointCloud2 conversion.
 *
 * This path never runs in simulation, where the relay publishes /livox/lidar directly, so the
 * sim acceptance test cannot catch a regression in it. Everything reading /livox/lidar on the
 * robot depends on it producing the layout they expect.
 */

#include <gmock/gmock.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "g1_state_estimation/livox_cloud.hpp"

namespace
{

livox_ros_driver2::msg::CustomPoint makePoint(float x, float y, float z, std::uint8_t reflectivity)
{
    livox_ros_driver2::msg::CustomPoint point;
    point.x            = x;
    point.y            = y;
    point.z            = z;
    point.reflectivity = reflectivity;
    return point;
}

TEST(LivoxCloud, CarriesEveryPointThrough)
{
    livox_ros_driver2::msg::CustomMsg custom;
    custom.header.frame_id  = "mid360_link";
    custom.header.stamp.sec = 42;
    custom.points = { makePoint(1.0F, 2.0F, 3.0F, 10), makePoint(-4.0F, 5.5F, -6.25F, 200) };

    sensor_msgs::msg::PointCloud2 cloud;
    g1_state_estimation::toPointCloud2(custom, cloud);

    // The frame and stamp have to survive: a cloud that arrives in the wrong frame is worse
    // than no cloud, because the costmap happily marks obstacles wherever it lands.
    EXPECT_EQ(cloud.header.frame_id, "mid360_link");
    EXPECT_EQ(cloud.header.stamp.sec, 42);
    EXPECT_EQ(cloud.height, 1U);
    EXPECT_EQ(cloud.width, 2U);
    EXPECT_FALSE(cloud.is_dense);

    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<float> intensity(cloud, "intensity");

    EXPECT_FLOAT_EQ(*x, 1.0F);
    EXPECT_FLOAT_EQ(*y, 2.0F);
    EXPECT_FLOAT_EQ(*z, 3.0F);
    EXPECT_FLOAT_EQ(*intensity, 10.0F);

    ++x, ++y, ++z, ++intensity;
    EXPECT_FLOAT_EQ(*x, -4.0F);
    EXPECT_FLOAT_EQ(*y, 5.5F);
    EXPECT_FLOAT_EQ(*z, -6.25F);
    EXPECT_FLOAT_EQ(*intensity, 200.0F);
}

TEST(LivoxCloud, PublishesTheLayoutConsumersRead)
{
    livox_ros_driver2::msg::CustomMsg custom;
    custom.points = { makePoint(0.0F, 0.0F, 0.0F, 0) };

    sensor_msgs::msg::PointCloud2 cloud;
    g1_state_estimation::toPointCloud2(custom, cloud);

    ASSERT_EQ(cloud.fields.size(), 4U);
    EXPECT_EQ(cloud.fields[0].name, "x");
    EXPECT_EQ(cloud.fields[1].name, "y");
    EXPECT_EQ(cloud.fields[2].name, "z");
    EXPECT_EQ(cloud.fields[3].name, "intensity");
    EXPECT_EQ(cloud.row_step, cloud.point_step * cloud.width);
}

TEST(LivoxCloud, AnEmptySweepIsAnEmptyCloudRatherThanAMalformedOne)
{
    livox_ros_driver2::msg::CustomMsg custom;
    custom.header.frame_id = "mid360_link";

    sensor_msgs::msg::PointCloud2 cloud;
    g1_state_estimation::toPointCloud2(custom, cloud);

    EXPECT_EQ(cloud.width, 0U);
    EXPECT_EQ(cloud.height, 1U);
    EXPECT_TRUE(cloud.data.empty());
    EXPECT_EQ(cloud.header.frame_id, "mid360_link");
}

TEST(LivoxCloud, ReusingTheSameMessageDoesNotAccumulate)
{
    // The node publishes from a fresh message today, but resizing onto a dirty cloud is the
    // obvious optimisation to make later, and it must not silently grow the point count.
    sensor_msgs::msg::PointCloud2 cloud;

    livox_ros_driver2::msg::CustomMsg first;
    first.points = { makePoint(1.0F, 0.0F, 0.0F, 1), makePoint(2.0F, 0.0F, 0.0F, 2) };
    g1_state_estimation::toPointCloud2(first, cloud);
    ASSERT_EQ(cloud.width, 2U);

    livox_ros_driver2::msg::CustomMsg second;
    second.points = { makePoint(3.0F, 0.0F, 0.0F, 3) };
    g1_state_estimation::toPointCloud2(second, cloud);

    EXPECT_EQ(cloud.width, 1U);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    EXPECT_FLOAT_EQ(*x, 3.0F);
}

}  // namespace
