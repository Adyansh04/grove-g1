/**
 * @file test_perception_visualizer.cpp
 * @brief Pins which drawn instances count as measured, so the image never boxes one twice.
 */

#include <gmock/gmock.h>

#include <initializer_list>

#include "g1_perception/perception_visualizer_node.hpp"

namespace
{

using g1_perception::measuredFor;

vision_msgs::msg::Detection3DArray withIds(std::initializer_list<const char*> ids)
{
    vision_msgs::msg::Detection3DArray objects;
    for (const char* id : ids)
    {
        objects.detections.emplace_back().id = id;
    }
    return objects;
}

TEST(MeasuredFor, FindsTheTrackAnInstanceIsLabelledWith)
{
    const vision_msgs::msg::Detection3DArray objects = withIds({ "red_cube_0", "red_cube" });

    const vision_msgs::msg::Detection3D* measured = measuredFor("red_cube_0", objects);

    ASSERT_NE(measured, nullptr);
    EXPECT_EQ(measured->id, "red_cube_0");
}

TEST(MeasuredFor, RejectsARawLabelThatEqualsAnAlias)
{
    // Two cups seen, one measured: the other keeps the raw label "cup", which is also the alias.
    EXPECT_EQ(measuredFor("cup", withIds({ "cup_0", "cup" })), nullptr);
}

TEST(MeasuredFor, RejectsARawPhrase)
{
    EXPECT_EQ(measuredFor("red cube", withIds({ "red_cube_0", "red_cube" })), nullptr);
}

}  // namespace
