/**
 * @file step_clear.cpp
 * @brief Straight-step search out of Nav2's collision band.
 */

#include "g1_locomotion/step_clear.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace g1_locomotion
{

namespace
{

constexpr int    kDirections = 72;     // Every 5 degrees.
constexpr double kSampleM    = 0.025;  // Half a local costmap cell.

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace

double clearanceAt(std::span<const ObstaclePoint> obstacles, double x, double y)
{
    double nearest = std::numeric_limits<double>::infinity();
    for (const ObstaclePoint& obstacle : obstacles)
    {
        nearest = std::min(nearest, std::hypot(obstacle.x - x, obstacle.y - y));
    }
    return nearest;
}

StepClearPlan planStepClear(
    std::span<const ObstaclePoint> obstacles, double x, double y, double yaw, double wanted_m,
    const StepClearLimits& limits)
{
    StepClearPlan plan;
    const double  start = clearanceAt(obstacles, x, y);
    if (start >= wanted_m)
    {
        plan.found = true;
        return plan;
    }
    const double floor   = std::min(limits.body_radius_m, start);
    const int    samples = static_cast<int>(std::floor((limits.max_step_m / kSampleM) + 1e-9));
    double       best    = std::numeric_limits<double>::infinity();
    for (int k = 0; k < kDirections; ++k)
    {
        const double direction = wrap(2.0 * std::numbers::pi * k / kDirections);
        const double c         = std::cos(direction);
        const double s         = std::sin(direction);
        for (int i = 1; i <= samples; ++i)
        {
            const double step      = i * kSampleM;
            const double clearance = clearanceAt(obstacles, x + (step * c), y + (step * s));
            if (clearance < floor)
            {
                break;
            }
            if (clearance < wanted_m)
            {
                continue;
            }
            const double ahead   = std::abs(wrap(direction - yaw));
            const bool   reverse = ahead > std::numbers::pi / 2.0;
            const double turn    = reverse ? std::numbers::pi - ahead : ahead;
            const double cost    = step + (limits.turn_cost_m_per_rad * turn);
            if (cost < best)
            {
                best = cost;
                plan = { true, direction, reverse, step };
            }
            break;
        }
    }
    return plan;
}

}  // namespace g1_locomotion
