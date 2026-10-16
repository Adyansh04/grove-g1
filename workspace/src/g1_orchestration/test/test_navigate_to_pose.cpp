/**
 * @file test_navigate_to_pose.cpp
 * @brief Pins the words for why Nav2 did not reach a goal, which its 1.3 result leaves empty.
 */

#include <gmock/gmock.h>

#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>

#include "g1_orchestration/skills/navigate_to_pose.hpp"

namespace
{

using g1_orchestration::describeNavigationCode;
using Plan   = nav2_msgs::action::ComputePathToPose::Result;
using Follow = nav2_msgs::action::FollowPath::Result;

TEST(NavigateToPose, SaysWhyThePlannerRefused)
{
    EXPECT_EQ(describeNavigationCode(Plan::GOAL_OUTSIDE_MAP), "the goal is outside the map");
    EXPECT_THAT(describeNavigationCode(Plan::GOAL_OCCUPIED), testing::HasSubstr("obstacle"));
    EXPECT_THAT(describeNavigationCode(Plan::NO_VALID_PATH), testing::HasSubstr("no path"));
}

TEST(NavigateToPose, SaysWhyTheControllerGaveUp)
{
    EXPECT_THAT(
        describeNavigationCode(Follow::FAILED_TO_MAKE_PROGRESS),
        testing::HasSubstr("progress"));
}

TEST(NavigateToPose, HasNoWordsForNoCode)
{
    // The caller falls back to the result code then.
    EXPECT_EQ(describeNavigationCode(0), "");
    EXPECT_EQ(describeNavigationCode(Plan::UNKNOWN), "");
}

}  // namespace
