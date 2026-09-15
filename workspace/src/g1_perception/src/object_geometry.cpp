#include "g1_perception/object_geometry.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>

namespace g1_perception
{
namespace
{

Point3 scaled(const Point3& v, double factor)
{
    return { v.x * factor, v.y * factor, v.z * factor };
}

Point3 normalised(const Point3& v)
{
    const double length = std::sqrt(dot(v, v));
    return length > 0.0 ? scaled(v, 1.0 / length) : Point3{ 1.0, 0.0, 0.0 };
}

/// Extent of a point set rotated into the axis pair at some angle.
struct Bounds
{
    double min_major{ std::numeric_limits<double>::max() };
    double max_major{ std::numeric_limits<double>::lowest() };
    double min_minor{ std::numeric_limits<double>::max() };
    double max_minor{ std::numeric_limits<double>::lowest() };

    [[nodiscard]] double major() const { return max_major - min_major; }
    [[nodiscard]] double minor() const { return max_minor - min_minor; }
};

Bounds boundsAt(const std::vector<double>& first, const std::vector<double>& second, double angle)
{
    const double cosine = std::cos(angle);
    const double sine   = std::sin(angle);
    Bounds       bounds;
    for (std::size_t i = 0; i < first.size(); ++i)
    {
        const double major = (first[i] * cosine) + (second[i] * sine);
        const double minor = -(first[i] * sine) + (second[i] * cosine);
        bounds.min_major   = std::min(bounds.min_major, major);
        bounds.max_major   = std::max(bounds.max_major, major);
        bounds.min_minor   = std::min(bounds.min_minor, minor);
        bounds.max_minor   = std::max(bounds.max_minor, minor);
    }
    return bounds;
}

/// The one deprojection, shared by the mask and its support ring.
std::optional<Point3> deprojectPixel(
    const DepthView& depth, std::uint32_t u, std::uint32_t v, const Intrinsics& intrinsics,
    double min_depth_m, double max_depth_m)
{
    const double z = depth.at(u, v);
    if (!std::isfinite(z) || z < min_depth_m || z > max_depth_m)
    {
        return std::nullopt;
    }
    return Point3{ (static_cast<double>(u) - intrinsics.cx) * z / intrinsics.fx,
                   (static_cast<double>(v) - intrinsics.cy) * z / intrinsics.fy,
                   z };
}

/// The angle whose bounding rectangle has the least area: a degree, then a tenth around the
/// winner. Rotating calipers would be exact but needs a convex hull, for no gain at this size.
double minimumAreaAngle(const std::vector<double>& first, const std::vector<double>& second)
{
    const auto area = [&first, &second](double angle) {
        const Bounds bounds = boundsAt(first, second, angle);
        return bounds.major() * bounds.minor();
    };
    // A rectangle repeats every quarter turn, so a quadrant covers every distinct orientation.
    double best      = 0.0;
    double best_area = area(0.0);
    for (int step = 1; step < 90; ++step)
    {
        const double angle = step * M_PI / 180.0;
        const double value = area(angle);
        if (value < best_area)
        {
            best_area = value;
            best      = angle;
        }
    }
    for (int step = -9; step <= 9; ++step)
    {
        const double angle = best + (step * M_PI / 1800.0);
        const double value = area(angle);
        if (value < best_area)
        {
            best_area = value;
            best      = angle;
        }
    }
    return best;
}

double median(std::vector<double>& values)
{
    const std::size_t middle = values.size() / 2;
    std::nth_element(
        values.begin(),
        values.begin() + static_cast<std::ptrdiff_t>(middle),
        values.end());
    return values[middle];
}

}  // namespace

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

std::string slugify(std::string_view phrase)
{
    std::string out;
    out.reserve(phrase.size());
    for (const char character : phrase)
    {
        const auto raw = static_cast<unsigned char>(character);
        if (std::isalnum(raw) != 0)
        {
            out.push_back(static_cast<char>(std::tolower(raw)));
        }
        else if (!out.empty() && out.back() != '_')
        {
            out.push_back('_');
        }
    }
    while (!out.empty() && out.back() == '_')
    {
        out.pop_back();
    }
    return out;
}

std::vector<std::uint8_t> erodeMask(const MaskView& mask, int iterations)
{
    std::vector<std::uint8_t> current(mask.data.begin(), mask.data.end());
    current.resize(static_cast<std::size_t>(mask.width) * mask.height, 0);
    for (int pass = 0; pass < iterations; ++pass)
    {
        std::vector<std::uint8_t> next(current.size(), 0);
        for (std::uint32_t row = 0; row < mask.height; ++row)
        {
            for (std::uint32_t col = 0; col < mask.width; ++col)
            {
                bool keep = current[(static_cast<std::size_t>(row) * mask.width) + col] != 0;
                for (int dy = -1; keep && dy <= 1; ++dy)
                {
                    for (int dx = -1; keep && dx <= 1; ++dx)
                    {
                        const auto ny = static_cast<std::int64_t>(row) + dy;
                        const auto nx = static_cast<std::int64_t>(col) + dx;
                        // Outside the crop is background: a border pixel is what erosion drops.
                        keep = ny >= 0 && nx >= 0 && ny < mask.height && nx < mask.width &&
                               current[(static_cast<std::size_t>(ny) * mask.width) + nx] != 0;
                    }
                }
                next[(static_cast<std::size_t>(row) * mask.width) + col] = keep ? 255 : 0;
            }
        }
        current.swap(next);
    }
    return current;
}

void planeBasis(const Point3& up, Point3& first, Point3& second)
{
    // Picked by component rather than by a threshold, so the basis does not flip as the camera
    // tilts.
    const Point3 seed  = std::abs(up.z) < 0.9 ? Point3{ 0.0, 0.0, 1.0 } : Point3{ 1.0, 0.0, 0.0 };
    const double along = dot(seed, up);
    first =
        normalised({ seed.x - (up.x * along), seed.y - (up.y * along), seed.z - (up.z * along) });
    second = cross(up, first);
}

std::vector<Point3> deproject(
    const DepthView& depth, const MaskView& mask, std::span<const std::uint8_t> mask_data,
    const Intrinsics& intrinsics, double min_depth_m, double max_depth_m)
{
    std::vector<Point3> points;
    if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0)
    {
        return points;
    }
    points.reserve(mask_data.size() / 2);
    for (std::uint32_t row = 0; row < mask.height; ++row)
    {
        for (std::uint32_t col = 0; col < mask.width; ++col)
        {
            const std::size_t index = (static_cast<std::size_t>(row) * mask.width) + col;
            if (index >= mask_data.size() || mask_data[index] == 0)
            {
                continue;
            }
            const std::optional<Point3> point = deprojectPixel(
                depth,
                mask.x + col,
                mask.y + row,
                intrinsics,
                min_depth_m,
                max_depth_m);
            if (point)
            {
                points.push_back(*point);
            }
        }
    }
    return points;
}

std::vector<Point3> supportRing(
    const DepthView& depth, const MaskView& mask, const Intrinsics& intrinsics, int ring_px,
    double min_depth_m, double max_depth_m)
{
    std::vector<Point3> points;
    if (ring_px <= 0 || intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0)
    {
        return points;
    }
    const auto left   = static_cast<std::int64_t>(mask.x) - ring_px;
    const auto top    = static_cast<std::int64_t>(mask.y) - ring_px;
    const auto right  = static_cast<std::int64_t>(mask.x + mask.width) + ring_px;
    const auto bottom = static_cast<std::int64_t>(mask.y + mask.height) + ring_px;

    for (std::int64_t v = std::max<std::int64_t>(top, 0);
         v < std::min<std::int64_t>(bottom, depth.height);
         ++v)
    {
        for (std::int64_t u = std::max<std::int64_t>(left, 0);
             u < std::min<std::int64_t>(right, depth.width);
             ++u)
        {
            const std::int64_t col = u - mask.x;
            const std::int64_t row = v - mask.y;
            const bool inside_roi  = col >= 0 && row >= 0 && col < mask.width && row < mask.height;
            if (inside_roi && mask.data[(static_cast<std::size_t>(row) * mask.width) + col] != 0)
            {
                continue;
            }
            const std::optional<Point3> point = deprojectPixel(
                depth,
                static_cast<std::uint32_t>(u),
                static_cast<std::uint32_t>(v),
                intrinsics,
                min_depth_m,
                max_depth_m);
            if (point)
            {
                points.push_back(*point);
            }
        }
    }
    return points;
}

void gateByMedianDepth(std::vector<Point3>& points, double gate_m)
{
    if (points.size() < 3 || gate_m <= 0.0)
    {
        return;
    }
    std::vector<double> depths;
    depths.reserve(points.size());
    for (const Point3& point : points)
    {
        depths.push_back(point.z);
    }
    const double middle = median(depths);
    std::erase_if(points, [middle, gate_m](const Point3& point) {
        return std::abs(point.z - middle) > gate_m;
    });
}

std::optional<double> supportHeight(
    std::span<const Point3> ring, std::span<const Point3> object_points, const Point3& up,
    double band_m, int min_points)
{
    double lowest = std::numeric_limits<double>::max();
    for (const Point3& point : object_points)
    {
        lowest = std::min(lowest, dot(point, up));
    }

    std::vector<double> heights;
    heights.reserve(ring.size());
    for (const Point3& point : ring)
    {
        const double height = dot(point, up);
        if (std::abs(height - lowest) <= band_m)
        {
            heights.push_back(height);
        }
    }
    if (heights.empty() || static_cast<int>(heights.size()) < min_points)
    {
        return std::nullopt;
    }
    return median(heights);
}

std::optional<OrientedBox> fitOrientedBox(
    std::span<const Point3> points, const Point3& up, std::optional<double> support_height,
    double min_extent_m, double max_extent_m)
{
    if (points.size() < 3)
    {
        return std::nullopt;
    }
    const Point3 vertical = normalised(up);
    Point3       first;
    Point3       second;
    planeBasis(vertical, first, second);

    std::vector<double> heights;
    heights.reserve(points.size());
    for (const Point3& point : points)
    {
        heights.push_back(dot(point, vertical));
    }
    const double top = *std::max_element(heights.begin(), heights.end());
    const double support =
        support_height.value_or(*std::min_element(heights.begin(), heights.end()));
    const double size_z = top - support;

    std::vector<double> along_first;
    std::vector<double> along_second;
    along_first.reserve(points.size());
    along_second.reserve(points.size());
    for (const Point3& point : points)
    {
        along_first.push_back(dot(point, first));
        along_second.push_back(dot(point, second));
    }
    // Least area, not covariance: a square footprint has no principal direction, so a covariance
    // fit lands at 45 degrees and reports a 6 cm cube as 8.5 cm across.
    const double angle  = minimumAreaAngle(along_first, along_second);
    const double cosine = std::cos(angle);
    const double sine   = std::sin(angle);
    const Bounds bounds = boundsAt(along_first, along_second, angle);

    OrientedBox box;
    box.size_x = bounds.major();
    box.size_y = bounds.minor();
    box.size_z = size_z;
    if (std::min({ box.size_x, box.size_y, box.size_z }) < min_extent_m ||
        std::max({ box.size_x, box.size_y, box.size_z }) > max_extent_m)
    {
        return std::nullopt;
    }

    box.axis_x = { (first.x * cosine) + (second.x * sine),
                   (first.y * cosine) + (second.y * sine),
                   (first.z * cosine) + (second.z * sine) };
    box.axis_y = cross(vertical, box.axis_x);

    const double centre_major = 0.5 * (bounds.min_major + bounds.max_major);
    const double centre_minor = 0.5 * (bounds.min_minor + bounds.max_minor);
    const double centre_up    = support + (0.5 * size_z);
    box.centre                = {
        (box.axis_x.x * centre_major) + (box.axis_y.x * centre_minor) + (vertical.x * centre_up),
        (box.axis_x.y * centre_major) + (box.axis_y.y * centre_minor) + (vertical.y * centre_up),
        (box.axis_x.z * centre_major) + (box.axis_y.z * centre_minor) + (vertical.z * centre_up)
    };
    return box;
}

}  // namespace g1_perception
