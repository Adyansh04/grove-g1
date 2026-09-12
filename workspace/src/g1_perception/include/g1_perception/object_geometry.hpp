#ifndef G1_PERCEPTION__OBJECT_GEOMETRY_HPP_
#define G1_PERCEPTION__OBJECT_GEOMETRY_HPP_

/**
 * @file object_geometry.hpp
 * @brief Turns an instance mask plus the depth frame it was computed from into an oriented box.
 *
 * ROS-free on purpose, so every step is driven directly by a unit test. The node above it only
 * pairs messages, applies parameters and publishes.
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

    /// Depth at a pixel, or NaN where the sensor reported nothing and where the pixel is outside.
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

/**
 * @brief Turns a noun phrase into the object id it is published under.
 *
 * Shared with the mock detector so the two agree on what "red cube" becomes without a table.
 */
[[nodiscard]] std::string slugify(std::string_view phrase);

/**
 * @brief Shrinks a mask by @p iterations passes of a 3x3 minimum filter.
 *
 * Depth at an object's edge is a blend of the object and whatever is behind it, so the outermost
 * ring of a mask reads as points that are neither.
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
    const DepthView&              depth,
    const MaskView&               mask,
    std::span<const std::uint8_t> mask_data,
    const Intrinsics&             intrinsics,
    double                        min_depth_m,
    double                        max_depth_m);

/**
 * @brief Deprojects the pixels in a ring just outside the mask: the surface the object stands on.
 *
 * The alternative, taking the lowest point of the object itself, is wrong for anything that hides
 * its own base: the lowest visible point of a sphere is its equator.
 */
[[nodiscard]] std::vector<Point3> supportRing(
    const DepthView&  depth,
    const MaskView&   mask,
    const Intrinsics& intrinsics,
    int               ring_px,
    double            min_depth_m,
    double            max_depth_m);

/**
 * @brief Drops points further than @p gate_m from the median depth, in place.
 *
 * A mask that leaks a few pixels onto the background would otherwise drag the box out toward it.
 */
void gateByMedianDepth(std::vector<Point3>& points, double gate_m);

/**
 * @brief Fits a box standing on a support surface to points seen from one side.
 *
 * The vertical extent runs from the support height to the highest visible point, which is the one
 * assumption that makes a single view enough: we see the top of an object, and we know what it
 * rests on.
 *
 * @param up             Unit up vector, in the same frame as @p points.
 * @param support_height Height of the supporting surface along @p up. Without it the lowest
 *                       points of the object stand in, which underestimates anything rounded.
 * @return Nothing when the points are too few or the box is outside the extent bounds, which is
 *         what a mask that ran onto the table looks like.
 */
[[nodiscard]] std::optional<OrientedBox> fitOrientedBox(
    std::span<const Point3> points,
    const Point3&           up,
    std::optional<double>   support_height,
    double                  min_extent_m,
    double                  max_extent_m);

}  // namespace g1_perception

#endif  // G1_PERCEPTION__OBJECT_GEOMETRY_HPP_
