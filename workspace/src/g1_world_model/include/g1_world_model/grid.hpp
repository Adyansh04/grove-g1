#ifndef G1_WORLD_MODEL__GRID_HPP_
#define G1_WORLD_MODEL__GRID_HPP_

/**
 * @file grid.hpp
 * @brief The 2D grid every layer of the world model is indexed by: the SLAM map's own cells.
 *
 * Layers share the occupancy grid's geometry so a cell index means the same place in all of
 * them. Row y grows with map y, so an image of a layer is upside down on screen.
 */

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

namespace g1_world_model
{

/// Occupancy classes, as stored in a CV_8U image.
enum Cell : std::uint8_t
{
    kUnknown  = 0,
    kFree     = 1,
    kOccupied = 2,
};

/// A cell by column and row, counted from the grid origin.
struct CellIndex
{
    int x = 0;
    int y = 0;

    friend bool operator==(const CellIndex&, const CellIndex&) = default;
};

/**
 * @brief Where a grid sits in the map frame and how big it is.
 */
struct GridGeometry
{
    double resolution = 0.05;  ///< Cell edge, m.
    double origin_x   = 0.0;   ///< Map x of the outer corner of cell (0, 0), m.
    double origin_y   = 0.0;   ///< Map y of the outer corner of cell (0, 0), m.
    int    width      = 0;     ///< Columns.
    int    height     = 0;     ///< Rows.

    [[nodiscard]] bool contains(int x, int y) const
    {
        return x >= 0 && y >= 0 && x < width && y < height;
    }

    [[nodiscard]] bool contains(CellIndex cell) const { return contains(cell.x, cell.y); }

    [[nodiscard]] CellIndex toCell(double map_x, double map_y) const
    {
        return { static_cast<int>(std::floor((map_x - origin_x) / resolution)),
                 static_cast<int>(std::floor((map_y - origin_y) / resolution)) };
    }

    [[nodiscard]] double centreX(int x) const { return origin_x + ((x + 0.5) * resolution); }

    [[nodiscard]] double centreY(int y) const { return origin_y + ((y + 0.5) * resolution); }

    [[nodiscard]] int index(int x, int y) const { return (y * width) + x; }

    [[nodiscard]] int index(CellIndex cell) const { return index(cell.x, cell.y); }

    [[nodiscard]] std::size_t cellCount() const
    {
        return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    }

    /// Cells spanned by @p metres, rounded to the nearest.
    [[nodiscard]] int cellsFor(double metres) const
    {
        return static_cast<int>(std::lround(metres / resolution));
    }

    friend bool operator==(const GridGeometry&, const GridGeometry&) = default;
};

/// A rectangular footprint in the map frame.
struct Footprint
{
    Eigen::Vector2d centre = Eigen::Vector2d::Zero();
    Eigen::Vector2d size   = Eigen::Vector2d::Zero();  ///< Along the box axes, m.
    double          yaw    = 0.0;
};

/**
 * @brief A per-cell layer carried onto another grid, as when SLAM grows or shifts its map.
 *
 * Each cell of @p to takes the @p from cell under its centre, or @p fill where there is none.
 */
template <typename T>
std::vector<T>
remapLayer(const std::vector<T>& layer, const GridGeometry& from, const GridGeometry& to, T fill)
{
    std::vector<T> out(to.cellCount(), fill);
    if (layer.size() != from.cellCount())
    {
        return out;
    }
    for (int y = 0; y < to.height; ++y)
    {
        const int source_y = from.toCell(to.centreX(0), to.centreY(y)).y;
        if (source_y < 0 || source_y >= from.height)
        {
            continue;
        }
        for (int x = 0; x < to.width; ++x)
        {
            const int source_x = from.toCell(to.centreX(x), to.centreY(y)).x;
            if (source_x >= 0 && source_x < from.width)
            {
                out[static_cast<std::size_t>(to.index(x, y))] =
                    layer[static_cast<std::size_t>(from.index(source_x, source_y))];
            }
        }
    }
    return out;
}

/**
 * @brief The walls' yaw in [0, pi/2), where projections stack occupied cells into the sharpest
 *        peaks; 0 when nothing is occupied. Indoor walls meet square, so one angle gives both.
 */
[[nodiscard]] double dominantAxis(const cv::Mat& cells);

/**
 * @brief Settles the unknown pockets a map encloses: occupied beside anything solid (furniture
 *        insides, unseen wall), free among floor alone; unknown reaching the edge stays.
 *
 * A wide pocket that also opens onto free floor is solid only under @p furniture, where the
 * camera has mapped an object; elsewhere it is floor no beam reached, behind a wardrobe.
 *
 * @param furniture CV_8U on the same grid, non-zero where a floor object stands; may be empty.
 */
[[nodiscard]] cv::Mat settleEnclosedUnknown(const cv::Mat& cells, const cv::Mat& furniture = {});

/// How far completeMap() carries solid across unknown, in cells.
struct PlanGaps
{
    int wall      = 0;  ///< Through a wall running straight on both sides for wall_run cells.
    int furniture = 0;  ///< Across anything else: one piece of furniture's depth.
    int wall_run  = 0;
    int crack     = 0;  ///< Through free cells, in such a wall only: narrower than any door.
};

/**
 * @brief The map drawn as a floor plan: each room closed, each piece of furniture solid.
 *
 * Wall-height hits fill unknown cells (free wins, so doorways stay open), walls hidden at every
 * height run straight on across the unknown, furniture fills across its depth, then pockets settle.
 *
 * @param cells     CV_8U Cell values of the scan map.
 * @param walls     CV_8U, non-zero where the LiDAR hit wall height.
 * @param yaw       The walls' yaw, from dominantAxis(), rad.
 * @param limits    How far to carry solid across unknown.
 * @param furniture As for settleEnclosedUnknown().
 */
[[nodiscard]] cv::Mat completeMap(
    const cv::Mat& cells, const cv::Mat& walls, double yaw, const PlanGaps& limits,
    const cv::Mat& furniture = {});

/**
 * @brief Classifies nav_msgs occupancy values into Cell values.
 *
 * Trinary, with map_saver's thresholds, so a map saved and served back classifies the same as
 * the live one it came from.
 *
 * @param data           width x height values, row major from the origin: -1 unknown, 0..100
 *                       occupancy probability in percent.
 * @param geometry       Size of @p data.
 * @param free_below     Values at or below this are free.
 * @param occupied_above Values at or above this are occupied; between the two is unknown.
 * @return A CV_8U image of Cell values, height rows by width columns.
 */
cv::Mat classifyOccupancy(
    const std::int8_t* data, const GridGeometry& geometry, int free_below = 25,
    int occupied_above = 65);

}  // namespace g1_world_model

#endif  // G1_WORLD_MODEL__GRID_HPP_
