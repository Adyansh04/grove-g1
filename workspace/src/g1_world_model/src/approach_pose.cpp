/**
 * @file approach_pose.cpp
 * @brief Ring sampling around a footprint for a reachable, clear, facing pose.
 */

#include "g1_world_model/approach_pose.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace g1_world_model
{

namespace
{

/// Distance from a point to a centred axis-aligned rectangle, 0 inside it.
double outsideDistance(double x, double y, double half_x, double half_y)
{
    return std::hypot(std::max(std::abs(x) - half_x, 0.0), std::max(std::abs(y) - half_y, 0.0));
}

}  // namespace

std::optional<Pose2D> approachPose(
    const cv::Mat& clearance, const std::vector<float>& travel, const GridGeometry& geometry,
    const Footprint& target, double standoff, const ApproachParams& params)
{
    if (clearance.empty() || travel.size() != geometry.cellCount())
    {
        return std::nullopt;
    }
    const double needed = params.robot_radius + params.margin;
    const double c      = std::cos(target.yaw);
    const double s      = std::sin(target.yaw);
    const double half_x = 0.5 * target.size.x();
    const double half_y = 0.5 * target.size.y();
    const double first  = standoff > 0.0 ? standoff : params.standoff;

    const int rings = static_cast<int>((params.max_standoff - first) / params.ring_step + 1e-9);
    for (int ring_index = 0; ring_index <= rings; ++ring_index)
    {
        const double ring = first + (ring_index * params.ring_step);
        // Points at `ring` from the footprint, one per direction from its centre.
        const double perimeter = (4.0 * (half_x + half_y)) + (2.0 * std::numbers::pi * ring);
        const int    samples   = std::max(16, static_cast<int>(perimeter / params.sample_step));
        double       best_cost = std::numeric_limits<double>::infinity();
        Pose2D       best;
        for (int i = 0; i < samples; ++i)
        {
            const double angle = 2.0 * std::numbers::pi * i / samples;
            const double ux    = std::cos(angle);
            const double uy    = std::sin(angle);
            // outsideDistance grows monotonically along the ray: bisect for `ring`.
            double low  = 0.0;
            double high = std::hypot(half_x, half_y) + ring + 1.0;
            for (int step = 0; step < 30; ++step)
            {
                const double mid = 0.5 * (low + high);
                (outsideDistance(mid * ux, mid * uy, half_x, half_y) < ring ? low : high) = mid;
            }
            const double    local_x = high * ux;
            const double    local_y = high * uy;
            const double    x       = target.centre.x() + (c * local_x) - (s * local_y);
            const double    y       = target.centre.y() + (s * local_x) + (c * local_y);
            const CellIndex cell    = geometry.toCell(x, y);
            if (!geometry.contains(cell) || clearance.at<float>(cell.y, cell.x) < needed)
            {
                continue;
            }
            const double cost = travel[static_cast<std::size_t>(geometry.index(cell))];
            if (std::isfinite(cost) && cost < best_cost)
            {
                best_cost = cost;
                best      = { x, y, std::atan2(target.centre.y() - y, target.centre.x() - x) };
            }
        }
        if (std::isfinite(best_cost))
        {
            return best;
        }
    }
    return std::nullopt;
}

}  // namespace g1_world_model
