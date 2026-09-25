#ifndef G1_WORLD_MODEL__ROOM_SEGMENTATION_HPP_
#define G1_WORLD_MODEL__ROOM_SEGMENTATION_HPP_

/**
 * @file room_segmentation.hpp
 * @brief Splits an occupancy grid into rooms at its doorways.
 *
 * A room is a peak of the clearance map, the distance from each free cell to the nearest obstacle,
 * that stays separate from its neighbours over a clear range of heights: its middle stands well
 * above the doorway it drains through. Free cells are added highest clearance first; each new
 * peak starts a component, and where two components meet the younger one dies (the elder rule
 * of 0-dimensional persistence). A component becomes a room when its peak stands at least
 * min_persistence above the pinch where it died, and that pinch is a doorway: no wider than
 * max_doorway_width, or narrow against the room itself (max_pinch_ratio). The second test is
 * what keeps a table from splitting a room: the gap beside it is wide and nearly as open as the
 * room. Cells follow steepest ascent to their peak, so neighbouring rooms meet at the doorway.
 * Hydra detects rooms from the same clearance idea with a threshold sweep.
 *
 * A corridor that opens into a room at its own width has no pinch to find. So, last, the parts
 * of a region too narrow for a disc of corridor_width, and long and thin besides, are split off:
 * morphological opening, then a shape test. Run it on a grid of walls only (see the node), or
 * the passage between a table and a counter reads as a corridor too.
 */

#include <opencv2/core.hpp>
#include <vector>

#include "g1_world_model/grid.hpp"

namespace g1_world_model
{

struct RoomSegmentationParams
{
    double min_persistence   = 0.15;   ///< Peak height above its doorway for a room, m.
    double min_peak          = 0.80;   ///< A room is at least 1.6 m across somewhere, m.
    double max_doorway_width = 1.5;    ///< Widest opening that always separates rooms, m.
    double max_pinch_ratio   = 0.7;    ///< Wider openings separate below this share of the peak.
    double min_room_area     = 2.0;    ///< Smaller regions merge into a neighbour, m^2.
    double min_doorway_width = 0.5;    ///< Narrower contacts between regions are map noise, m.
    double outline_tolerance = 0.075;  ///< Polygon simplification, m.
    double corridor_width    = 2.4;    ///< Narrower strips of a region can be corridors, m.
    double corridor_aspect   = 3.5;    ///< Least length over width for a strip to be one.
    double min_corridor_area = 4.0;    ///< m^2.
};

/// Where one region touches another.
struct RegionContact
{
    int    to_label = 0;    ///< Label of the region on the other side.
    double x        = 0.0;  ///< Map-frame centre of the contact, m.
    double y        = 0.0;
    double width    = 0.0;  ///< Clear width at its widest point, m.
};

struct Region
{
    int    label          = 0;    ///< Value of its cells in Segmentation::labels.
    double area           = 0.0;  ///< m^2.
    double centroid_x     = 0.0;  ///< Map frame, m.
    double centroid_y     = 0.0;
    double mean_clearance = 0.0;  ///< Mean distance of its cells to the nearest obstacle, m.
    double length         = 0.0;  ///< Long side of its minimum-area rectangle, m.
    double width          = 0.0;  ///< Short side, m.

    std::vector<cv::Point2d>   outline;  ///< Map frame, counter-clockwise.
    std::vector<RegionContact> contacts;
};

struct Segmentation
{
    cv::Mat             labels;   ///< CV_32S; 0 outside every room, else the region's label.
    std::vector<Region> regions;  ///< regions[i].label == i + 1.
};

/**
 * @brief Segments the free space of @p cells into rooms.
 *
 * @param cells    CV_8U Cell image, from classifyOccupancy().
 * @param geometry Placement of @p cells.
 * @param params   Thresholds.
 * @return Regions labelled 1..n; unknown and occupied cells stay 0.
 */
Segmentation segmentRooms(
    const cv::Mat& cells, const GridGeometry& geometry, const RoomSegmentationParams& params);

/**
 * @brief Pairs each current region with the previous region it continues.
 *
 * Greedy by overlap: a previous region continues at most one current region.
 *
 * @param previous       CV_32S labels of the earlier segmentation, same geometry as @p current.
 * @param previous_count Highest label in @p previous.
 * @param current        CV_32S labels of the new segmentation.
 * @param current_count  Highest label in @p current.
 * @param min_iou        Least intersection over union for two regions to be the same room.
 * @return For each current label l, element l - 1 is the previous label it continues, or 0.
 */
std::vector<int> matchRegions(
    const cv::Mat& previous, int previous_count, const cv::Mat& current, int current_count,
    double min_iou);

/**
 * @brief Re-grids a label image onto another geometry, nearest cell; unmapped cells become 0.
 */
cv::Mat resampleLabels(const cv::Mat& labels, const GridGeometry& from, const GridGeometry& to);

}  // namespace g1_world_model

#endif  // G1_WORLD_MODEL__ROOM_SEGMENTATION_HPP_
