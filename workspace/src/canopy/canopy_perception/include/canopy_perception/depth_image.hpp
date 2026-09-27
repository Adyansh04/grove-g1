#ifndef CANOPY_PERCEPTION__DEPTH_IMAGE_HPP_
#define CANOPY_PERCEPTION__DEPTH_IMAGE_HPP_

/**
 * @file depth_image.hpp
 * @brief Pinhole intrinsics and a borrowed view over a depth image, without ROS.
 */

#include <cstdint>
#include <span>

namespace canopy_perception
{

/// Pinhole parameters, read from CameraInfo::k rather than assumed.
struct Intrinsics
{
    double fx{ 0.0 };
    double fy{ 0.0 };
    double cx{ 0.0 };
    double cy{ 0.0 };
};

/**
 * @brief A borrowed view over a 32FC1 depth image in metres.
 *
 * Rows can be padded, so `step` is carried rather than derived from the width.
 */
struct DepthView
{
    std::span<const std::uint8_t> data;
    std::uint32_t                 width{ 0 };
    std::uint32_t                 height{ 0 };
    std::uint32_t                 step{ 0 };

    /// Depth at a pixel, or NaN where the sensor reported nothing or the pixel is out of bounds.
    [[nodiscard]] double at(std::uint32_t u, std::uint32_t v) const;
};

}  // namespace canopy_perception

#endif  // CANOPY_PERCEPTION__DEPTH_IMAGE_HPP_
