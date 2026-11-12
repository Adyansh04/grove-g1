/**
 * @file test_livox_cloud.cpp
 * @brief The hardware CustomMsg -> PointCloud2 conversion, which no simulation path reaches.
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
    g1_state_estimation::toPointCloud2(custom, cloud, 0.0);

    // A cloud in the wrong frame is worse than none: the costmap marks it wherever it lands.
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
    g1_state_estimation::toPointCloud2(custom, cloud, 0.0);

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
    g1_state_estimation::toPointCloud2(custom, cloud, 0.0);

    EXPECT_EQ(cloud.width, 0U);
    EXPECT_EQ(cloud.height, 1U);
    EXPECT_TRUE(cloud.data.empty());
    EXPECT_EQ(cloud.header.frame_id, "mid360_link");
}

TEST(LivoxCloud, ReusingTheSameMessageDoesNotAccumulate)
{
    // Resizing onto a reused cloud must not grow the point count.
    sensor_msgs::msg::PointCloud2 cloud;

    livox_ros_driver2::msg::CustomMsg first;
    first.points = { makePoint(1.0F, 0.0F, 0.0F, 1), makePoint(2.0F, 0.0F, 0.0F, 2) };
    g1_state_estimation::toPointCloud2(first, cloud, 0.0);
    ASSERT_EQ(cloud.width, 2U);

    livox_ros_driver2::msg::CustomMsg second;
    second.points = { makePoint(3.0F, 0.0F, 0.0F, 3) };
    g1_state_estimation::toPointCloud2(second, cloud, 0.0);

    EXPECT_EQ(cloud.width, 1U);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    EXPECT_FLOAT_EQ(*x, 3.0F);
}

TEST(LivoxCloud, DropsTheNoEchoPointsAtTheSensor)
{
    // The driver's no-echo returns sit at the origin; 0.05 m is inside the Mid360's blind range.
    livox_ros_driver2::msg::CustomMsg custom;
    custom.points = { makePoint(0.0F, 0.0F, 0.0F, 0),
                      makePoint(1.5F, 0.0F, -1.0F, 30),
                      makePoint(0.05F, 0.0F, 0.0F, 5),
                      makePoint(0.0F, -0.2F, 0.0F, 40) };

    sensor_msgs::msg::PointCloud2 cloud;
    g1_state_estimation::toPointCloud2(custom, cloud, 0.1);

    ASSERT_EQ(cloud.width, 2U);
    EXPECT_EQ(cloud.row_step, cloud.point_step * 2U);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> intensity(cloud, "intensity");
    EXPECT_FLOAT_EQ(*x, 1.5F);
    EXPECT_FLOAT_EQ(*intensity, 30.0F);
    ++x, ++intensity;
    EXPECT_FLOAT_EQ(*x, 0.0F);
    EXPECT_FLOAT_EQ(*intensity, 40.0F);
}

TEST(LivoxCloud, ASweepOfOnlyNoEchoPointsIsAnEmptyCloud)
{
    livox_ros_driver2::msg::CustomMsg custom;
    custom.points = { makePoint(0.0F, 0.0F, 0.0F, 0), makePoint(0.0F, 0.0F, 0.0F, 0) };

    sensor_msgs::msg::PointCloud2 cloud;
    g1_state_estimation::toPointCloud2(custom, cloud, 0.1);

    EXPECT_EQ(cloud.width, 0U);
    EXPECT_TRUE(cloud.data.empty());
}

}  // namespace
