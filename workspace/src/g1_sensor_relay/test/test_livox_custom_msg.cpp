/**
 * @file test_livox_custom_msg.cpp
 * @brief The sweep as FAST-LIO's Livox handler will read it.
 *
 * Every assertion here is a gate in that handler (preprocess.cpp, avia_handler): a point with
 * `line >= scan_line` is skipped, a point whose tag bits 4-5 are not 00 or 01 is skipped, and
 * `offset_time` is read as milliseconds-since-scan-start into the undistortion. Get one wrong
 * and FAST-LIO silently registers against a fraction of the cloud, or none of it.
 */
#include <gmock/gmock.h>

#include <array>
#include <cmath>
#include <vector>

#include "g1_sensor_relay/livox_custom_msg.hpp"

namespace g1_sensor_relay
{
namespace
{

sensor_msgs::msg::PointCloud2 cloudOf(const std::vector<std::array<float, 3>>& points)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id      = "mid360_link";
    cloud.header.stamp.sec     = 7;
    cloud.header.stamp.nanosec = 250000000U;
    cloud.height               = 1;
    cloud.width                = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian         = false;
    cloud.is_dense             = false;
    cloud.point_step           = 12;
    cloud.row_step             = cloud.point_step * cloud.width;
    cloud.fields.resize(3);
    const std::array<const char*, 3> names = { "x", "y", "z" };
    for (std::size_t i = 0; i < 3; ++i)
    {
        cloud.fields[i].name     = names[i];
        cloud.fields[i].offset   = static_cast<std::uint32_t>(i * 4);
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count    = 1;
    }
    cloud.data.resize(static_cast<std::size_t>(cloud.point_step) * points.size());
    auto* out = reinterpret_cast<float*>(cloud.data.data());
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        out[3 * i + 0] = points[i][0];
        out[3 * i + 1] = points[i][1];
        out[3 * i + 2] = points[i][2];
    }
    return cloud;
}

TEST(LivoxCustomMsg, CarriesEveryPointThroughUnchanged)
{
    const auto cloud = cloudOf({ { 1.0F, 2.0F, 3.0F }, { -4.5F, 0.0F, 0.25F } });
    livox_ros_driver2::msg::CustomMsg out;

    ASSERT_TRUE(toCustomMsg(cloud, out));

    ASSERT_EQ(out.point_num, 2U);
    ASSERT_EQ(out.points.size(), 2U);
    EXPECT_FLOAT_EQ(out.points[0].x, 1.0F);
    EXPECT_FLOAT_EQ(out.points[0].y, 2.0F);
    EXPECT_FLOAT_EQ(out.points[0].z, 3.0F);
    EXPECT_FLOAT_EQ(out.points[1].x, -4.5F);
    EXPECT_FLOAT_EQ(out.points[1].z, 0.25F);
    EXPECT_EQ(out.header.frame_id, "mid360_link");
    EXPECT_EQ(out.header.stamp.sec, 7);
    EXPECT_EQ(out.timebase, 7250000000ULL);
}

TEST(LivoxCustomMsg, EveryPointPassesFastLioGates)
{
    const auto cloud = cloudOf({ { 1.0F, 0.0F, 0.0F }, { 0.0F, 1.0F, 0.0F } });
    livox_ros_driver2::msg::CustomMsg out;

    ASSERT_TRUE(toCustomMsg(cloud, out));

    for (const auto& point : out.points)
    {
        EXPECT_LT(point.line, 4U) << "scan_line is 4; a higher line is discarded outright";
        EXPECT_TRUE((point.tag & 0x30U) == 0x00U || (point.tag & 0x30U) == 0x10U)
            << "tag bits 4-5 outside {00, 01} are discarded outright";
        EXPECT_EQ(point.offset_time, 0U) << "the sweep is one instant; there is nothing to undo";
    }
}

TEST(LivoxCustomMsg, DropsMissesAndKeepsPointNumHonest)
{
    const auto                        nan   = std::numeric_limits<float>::quiet_NaN();
    const auto                        cloud = cloudOf({ { 1.0F, 1.0F, 1.0F },
                                                        { nan, 0.0F, 0.0F },
                                                        { 0.0F, std::numeric_limits<float>::infinity(), 0.0F },
                                                        { 2.0F, 2.0F, 2.0F } });
    livox_ros_driver2::msg::CustomMsg out;

    ASSERT_TRUE(toCustomMsg(cloud, out));

    // point_num is what FAST-LIO iterates to; a count that includes dropped points walks off
    // the end of the vector.
    EXPECT_EQ(out.point_num, 2U);
    EXPECT_EQ(out.points.size(), 2U);
    EXPECT_FLOAT_EQ(out.points[1].x, 2.0F);
}

TEST(LivoxCustomMsg, RefusesACloudWithoutXyz)
{
    sensor_msgs::msg::PointCloud2 cloud = cloudOf({ { 1.0F, 1.0F, 1.0F } });
    cloud.fields[2].name                = "intensity";
    livox_ros_driver2::msg::CustomMsg out;
    out.point_num = 99U;

    EXPECT_FALSE(toCustomMsg(cloud, out));
    EXPECT_EQ(out.point_num, 99U) << "a refused cloud must not half-fill the message";
}

TEST(LivoxCustomMsg, ReusingOneMessageDoesNotAccumulate)
{
    livox_ros_driver2::msg::CustomMsg out;
    ASSERT_TRUE(toCustomMsg(cloudOf({ { 1.0F, 1.0F, 1.0F }, { 2.0F, 2.0F, 2.0F } }), out));
    ASSERT_TRUE(toCustomMsg(cloudOf({ { 3.0F, 3.0F, 3.0F } }), out));

    EXPECT_EQ(out.point_num, 1U);
    EXPECT_EQ(out.points.size(), 1U);
}

}  // namespace
}  // namespace g1_sensor_relay
