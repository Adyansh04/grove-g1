/**
 * @file depth_image.cpp
 * @brief Reading one depth pixel.
 */

#include "canopy_perception/depth_image.hpp"

#include <cstring>
#include <limits>

namespace canopy_perception
{

double DepthView::at(std::uint32_t u, std::uint32_t v) const
{
    if (u >= width || v >= height)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t offset = (static_cast<std::size_t>(v) * step) + (u * sizeof(float));
    if (offset + sizeof(float) > data.size())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    float metres = 0.0F;
    std::memcpy(&metres, data.data() + offset, sizeof(float));
    return static_cast<double>(metres);
}

}  // namespace canopy_perception
