/**
 * @file test_depth_history.cpp
 * @brief Pins the pairing rule that keeps a slow detector honest.
 */

#include <gmock/gmock.h>

#include <memory>

#include "g1_perception/depth_history.hpp"

namespace
{

using g1_perception::DepthHistory;

sensor_msgs::msg::Image::SharedPtr frameAt(double stamp_s)
{
    auto frame              = std::make_shared<sensor_msgs::msg::Image>();
    frame->header.stamp.sec = static_cast<std::int32_t>(stamp_s);
    frame->header.stamp.nanosec =
        static_cast<std::uint32_t>((stamp_s - static_cast<std::int32_t>(stamp_s)) * 1e9);
    frame->width  = 848;
    frame->height = 480;
    return frame;
}

TEST(DepthHistory, FindsTheFrameForALateMask)
{
    DepthHistory history(3.0, 0.005);
    for (int tick = 0; tick <= 20; ++tick)
    {
        history.push(frameAt(100.0 + (0.1 * tick)));
    }

    const auto matched = history.at(100.5);

    ASSERT_NE(matched, nullptr) << "a mask 1.5 s behind the newest frame still has its own";
    EXPECT_NEAR(DepthHistory::stampSeconds(matched->header), 100.5, 1e-6);
}

TEST(DepthHistory, RefusesOutsideTolerance)
{
    DepthHistory history(3.0, 0.005);
    history.push(frameAt(100.0));

    EXPECT_EQ(history.at(100.02), nullptr);
    EXPECT_NE(history.at(100.002), nullptr);
}

TEST(DepthHistory, DropsFramesPastTheWindow)
{
    DepthHistory history(1.0, 0.005);
    history.push(frameAt(100.0));
    history.push(frameAt(100.5));
    history.push(frameAt(102.0));

    EXPECT_EQ(history.size(), 1U);
    EXPECT_EQ(history.at(100.0), nullptr);
    EXPECT_NE(history.at(102.0), nullptr);
}

TEST(DepthHistory, IsEmptyUntilSomethingArrives)
{
    const DepthHistory history(3.0, 0.005);

    EXPECT_EQ(history.at(100.0), nullptr);
    EXPECT_EQ(history.atOrBefore(100.0), nullptr);
}

TEST(DepthHistory, ReachesBackToTheNewestFrameNoLaterThanAsked)
{
    DepthHistory history(3.0, 0.005);
    history.push(frameAt(100.0));
    history.push(frameAt(100.5));
    history.push(frameAt(101.0));

    // Unlike at(), no tolerance: a stamp between two frames takes the earlier one.
    EXPECT_EQ(DepthHistory::stampSeconds(history.atOrBefore(100.7)->header), 100.5);
    EXPECT_EQ(DepthHistory::stampSeconds(history.atOrBefore(101.0)->header), 101.0);
    EXPECT_EQ(DepthHistory::stampSeconds(history.atOrBefore(400.0)->header), 101.0);
    // Older than everything held: the oldest stands in rather than nothing, so a mock asked for
    // a latency longer than it has been running still answers.
    EXPECT_EQ(DepthHistory::stampSeconds(history.atOrBefore(1.0)->header), 100.0);
}

}  // namespace
