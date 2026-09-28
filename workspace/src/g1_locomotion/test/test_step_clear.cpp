/**
 * @file test_step_clear.cpp
 * @brief The step out of Nav2's collision band: which way, how far, and when there is none.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <numbers>
#include <vector>

#include "g1_locomotion/step_clear.hpp"

namespace g1_locomotion
{
namespace
{

constexpr double kWanted = 0.55;  // Nav2's 0.45 m circle plus two costmap cells.

/// Obstacle cells every 5 cm from (x0, y0) to (x1, y1).
void wall(std::vector<ObstaclePoint>& points, double x0, double y0, double x1, double y1)
{
    const int count = static_cast<int>(std::ceil(std::hypot(x1 - x0, y1 - y0) / 0.05));
    for (int i = 0; i <= count; ++i)
    {
        const double t = static_cast<double>(i) / count;
        points.push_back({ x0 + (t * (x1 - x0)), y0 + (t * (y1 - y0)) });
    }
}

/// The plan's path never takes the body nearer anything than it may.
bool pathStaysClear(const std::vector<ObstaclePoint>& points, const StepClearPlan& plan)
{
    const double floor   = std::min(StepClearLimits{}.body_radius_m, clearanceAt(points, 0.0, 0.0));
    const int    samples = static_cast<int>(plan.distance_m / 0.01);
    for (int i = 0; i <= samples; ++i)
    {
        const double s = i * 0.01;
        if (clearanceAt(points, s * std::cos(plan.direction), s * std::sin(plan.direction)) <
            floor - 1e-9)
        {
            return false;
        }
    }
    return true;
}

TEST(StepClear, StaysPutWhenAlreadyClear)
{
    std::vector<ObstaclePoint> points;
    wall(points, -1.0, 1.0, 1.0, 1.0);
    const StepClearPlan plan = planStepClear(points, 0.0, 0.0, 0.0, kWanted, {});
    EXPECT_TRUE(plan.found);
    EXPECT_DOUBLE_EQ(plan.distance_m, 0.0);
}

TEST(StepClear, StepsOffAWallBesideWithoutAQuarterTurn)
{
    // Straight out needs a quarter turn; a diagonal walks a few centimetres more for much less.
    std::vector<ObstaclePoint> points;
    wall(points, -1.0, 0.4, 1.0, 0.4);
    const StepClearPlan plan = planStepClear(points, 0.0, 0.0, 0.0, kWanted, {});
    ASSERT_TRUE(plan.found);
    EXPECT_LT(std::sin(plan.direction), -0.5);
    EXPECT_GT(std::abs(std::cos(plan.direction)), 0.3);
    EXPECT_LT(plan.distance_m, 0.3);
    EXPECT_TRUE(pathStaysClear(points, plan));
}

TEST(StepClear, BacksAwayFromAWallAheadWithoutTurning)
{
    std::vector<ObstaclePoint> points;
    wall(points, 0.4, -1.0, 0.4, 1.0);
    const StepClearPlan plan = planStepClear(points, 0.0, 0.0, 0.0, kWanted, {});
    ASSERT_TRUE(plan.found);
    EXPECT_TRUE(plan.reverse);
    EXPECT_LT(std::cos(plan.direction), -0.99);
    EXPECT_NEAR(plan.distance_m, 0.15, 0.03);
}

TEST(StepClear, LeavesACornerAwayFromBothWalls)
{
    std::vector<ObstaclePoint> points;
    wall(points, 0.4, -1.0, 0.4, 0.4);
    wall(points, -1.0, 0.4, 0.4, 0.4);
    const StepClearPlan plan = planStepClear(points, 0.0, 0.0, std::numbers::pi / 4.0, kWanted, {});
    ASSERT_TRUE(plan.found);
    EXPECT_LT(std::cos(plan.direction), 0.0);
    EXPECT_LT(std::sin(plan.direction), 0.0);
    EXPECT_TRUE(pathStaysClear(points, plan));
}

TEST(StepClear, GoesRoundAPostRatherThanBrushingIt)
{
    // Wall to the left, a post just behind on the right: straight right would brush the post.
    std::vector<ObstaclePoint> points;
    wall(points, -1.0, 0.4, 1.0, 0.4);
    points.push_back({ -0.1, -0.32 });
    const StepClearPlan plan = planStepClear(points, 0.0, 0.0, 0.0, kWanted, {});
    ASSERT_TRUE(plan.found);
    EXPECT_TRUE(pathStaysClear(points, plan));
    EXPECT_GT(std::cos(plan.direction), 0.0);
}

TEST(StepClear, ReportsARobotBoxedIn)
{
    std::vector<ObstaclePoint> points;
    for (int k = 0; k < 64; ++k)
    {
        const double a = 2.0 * std::numbers::pi * k / 64;
        points.push_back({ 0.4 * std::cos(a), 0.4 * std::sin(a) });
    }
    EXPECT_FALSE(planStepClear(points, 0.0, 0.0, 0.0, kWanted, {}).found);
}

}  // namespace
}  // namespace g1_locomotion
