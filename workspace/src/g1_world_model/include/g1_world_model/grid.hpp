#ifndef G1_WORLD_MODEL__GRID_HPP_
#define G1_WORLD_MODEL__GRID_HPP_

/**
 * @file grid.hpp
 * @brief The 2D grid every layer of the world model is indexed by: the SLAM map's own cells.
 *
 * Layers share the occupancy grid's geometry so a cell index means the same place in all of
 * them. Row y grows with map y, so an image of a layer is upside down on screen.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>

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
