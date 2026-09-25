#ifndef G1_WORLD_MODEL__APPROACH_POSE_HPP_
#define G1_WORLD_MODEL__APPROACH_POSE_HPP_

/**
 * @file approach_pose.hpp
 * @brief A reachable standing pose facing a target, for Nav2.
 *
 * Candidates ring the target's footprint at the standoff distance, widening until one is both
 * clear of obstacles by the robot's radius and reachable; the shortest walk wins. The heading
 * faces the footprint's centre, which also puts the target in the downward-pitched camera's view
 * once the robot stops.
 */

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

#include "g1_world_model/grid.hpp"
#include "g1_world_model/viewpoint_planner.hpp"

namespace g1_world_model
{

struct ApproachParams
{
    double robot_radius = 0.45;  ///< Clearance a standing pose needs, m.
    double margin       = 0.10;  ///< Extra clearance, m.
    double standoff     = 0.60;  ///< Default distance from the footprint's edge, m.
    double max_standoff = 1.80;  ///< Widest ring tried, m.
    double ring_step    = 0.15;  ///< Standoff increment between rings, m.
    double sample_step  = 0.15;  ///< Spacing of candidates along a ring, m.
};

/// A rectangular footprint in the map frame.
struct Footprint
{
    Eigen::Vector2d centre = Eigen::Vector2d::Zero();
    Eigen::Vector2d size   = Eigen::Vector2d::Zero();  ///< Along the box axes, m.
    double          yaw    = 0.0;
};

/**
 * @brief The nearest reachable pose facing @p target.
 *
 * @param clearance CV_32F clearance, m, from ViewpointPlanner::clearance().
 * @param travel    Path lengths from the robot, from ViewpointPlanner::travel().
 * @param geometry  Grid placement.
 * @param target    What to face.
 * @param standoff  Distance from the footprint, m; 0 uses params.standoff.
 * @return The pose, or nothing when no ring out to max_standoff has a reachable clear spot.
 */
std::optional<Pose2D> approachPose(
    const cv::Mat& clearance, const std::vector<float>& travel, const GridGeometry& geometry,
    const Footprint& target, double standoff, const ApproachParams& params);

}  // namespace g1_world_model

#endif  // G1_WORLD_MODEL__APPROACH_POSE_HPP_
