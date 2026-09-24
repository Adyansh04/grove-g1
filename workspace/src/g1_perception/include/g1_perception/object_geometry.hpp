#ifndef G1_PERCEPTION__OBJECT_GEOMETRY_HPP_
#define G1_PERCEPTION__OBJECT_GEOMETRY_HPP_

/**
 * @file object_geometry.hpp
 * @brief Turns an instance mask plus the depth frame it was cut from into an oriented box.
 *
 * ROS-free, so a unit test drives every step directly.
 */

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace g1_perception
{

/// A point in the frame the depth image was captured in, metres.
struct Point3
{
    double x{ 0.0 };
    double y{ 0.0 };
    double z{ 0.0 };
};

[[nodiscard]] inline double dot(const Point3& a, const Point3& b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

[[nodiscard]] inline Point3 cross(const Point3& a, const Point3& b)
{
    return { (a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x) };
}

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

/// A mask cropped to its own bounding box: `data` is `width * height` bytes, zero outside.
struct MaskView
{
    std::span<const std::uint8_t> data;
    std::uint32_t                 x{ 0 };
    std::uint32_t                 y{ 0 };
    std::uint32_t                 width{ 0 };
    std::uint32_t                 height{ 0 };
};

/// An axis-aligned-in-its-own-frame box: full widths along three orthonormal axes.
struct OrientedBox
{
    Point3 centre;
    double size_x{ 0.0 };
    double size_y{ 0.0 };
    double size_z{ 0.0 };
    /// First in-plane axis, perpendicular to the up vector the fit was given.
    Point3 axis_x;
    /// Second in-plane axis, completing a right-handed frame with axis_x and up.
    Point3 axis_y;
};

/// Turns a noun phrase into the object id it is published under. Shared with the mock detector.
[[nodiscard]] std::string slugify(std::string_view phrase);

/**
 * @brief Shrinks a mask by @p iterations passes of a 3x3 minimum filter.
 *
 * Depth at an object's edge blends the object with what is behind it.
 *
 * @return A crop of the same dimensions as @p mask.
 */
[[nodiscard]] std::vector<std::uint8_t> erodeMask(const MaskView& mask, int iterations);

/// Unit vectors spanning the plane perpendicular to @p up, chosen deterministically from it.
void planeBasis(const Point3& up, Point3& first, Point3& second);

/**
 * @brief Deprojects every masked pixel that carries usable depth.
 *
 * @param mask_data Overrides `mask.data`, so an eroded copy can be used without rebuilding the
 *                  view. Must hold `mask.width * mask.height` bytes.
 */
[[nodiscard]] std::vector<Point3> deproject(
    const DepthView& depth, const MaskView& mask, std::span<const std::uint8_t> mask_data,
    const Intrinsics& intrinsics, double min_depth_m, double max_depth_m);

/**
 * @brief Deprojects the pixels in a ring just outside the mask: the surface the object stands on.
 *
 * The object's own lowest visible point will not do: for a sphere that is its equator.
 */
[[nodiscard]] std::vector<Point3> supportRing(
    const DepthView& depth, const MaskView& mask, const Intrinsics& intrinsics, int ring_px,
    double min_depth_m, double max_depth_m);

/**
 * @brief Drops points further than @p gate_m from the median depth, in place.
 *
 * A mask that leaks a few pixels onto the background would otherwise drag the box out toward it.
 */
void gateByMedianDepth(std::vector<Point3>& points, double gate_m);

/**
 * @brief Median height of the ring points that could be the surface @p object_points stand on.
 *
 * @param band_m Half-width of the band around the object's lowest point, which keeps the floor
 *               and a taller neighbour out of the median.
 * @return Nothing when fewer than @p min_points survive the band.
 */
[[nodiscard]] std::optional<double> supportHeight(
    std::span<const Point3> ring, std::span<const Point3> object_points, const Point3& up,
    double band_m, int min_points);

/**
 * @brief Fits a box standing on a support surface to points seen from one side.
 *
 * Height runs from the support to the highest visible point, which is what makes one view enough.
 *
 * @param up             Unit up vector, in the same frame as @p points.
 * @param support_height Height of the supporting surface along @p up. Without it the object's own
 *                       lowest points stand in, which underestimates anything rounded.
 * @return Nothing when the points are too few or the box is outside the extent bounds.
 */
[[nodiscard]] std::optional<OrientedBox> fitOrientedBox(
    std::span<const Point3> points, const Point3& up, std::optional<double> support_height,
    double min_extent_m, double max_extent_m);

}  // namespace g1_perception

#endif  // G1_PERCEPTION__OBJECT_GEOMETRY_HPP_
