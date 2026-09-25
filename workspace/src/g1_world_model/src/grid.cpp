/**
 * @file grid.cpp
 * @brief Occupancy classification and grid fingerprints.
 */

#include "g1_world_model/grid.hpp"

namespace g1_world_model
{

cv::Mat classifyOccupancy(
    const std::int8_t* data, const GridGeometry& geometry, int free_below, int occupied_above)
{
    cv::Mat           cells(geometry.height, geometry.width, CV_8UC1);
    const std::size_t count = geometry.cellCount();
    auto*             out   = cells.ptr<std::uint8_t>(0);
    for (std::size_t i = 0; i < count; ++i)
    {
        // Signed by definition: -1 is unknown.
        const auto value = static_cast<int>(data[i]);  // NOLINT(bugprone-signed-char-misuse)
        if (value < 0)
        {
            out[i] = kUnknown;
        }
        else if (value >= occupied_above)
        {
            out[i] = kOccupied;
        }
        else
        {
            out[i] = value <= free_below ? kFree : kUnknown;
        }
    }
    return cells;
}

}  // namespace g1_world_model
