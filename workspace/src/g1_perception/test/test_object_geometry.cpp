/**
 * @file test_object_geometry.cpp
 * @brief Drives the mask-to-box arithmetic directly, without a camera or a simulator.
 *
 * Two of these cover cases the simulator cannot produce at all: MuJoCo's depth is exact, so the
 * dropout handling a real D435i needs every frame is only ever exercised here.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <vector>

#include "g1_perception/object_geometry.hpp"

namespace
{

using g1_perception::deproject;
using g1_perception::DepthView;
using g1_perception::erodeMask;
using g1_perception::fitOrientedBox;
using g1_perception::gateByMedianDepth;
using g1_perception::Intrinsics;
using g1_perception::MaskView;
using g1_perception::Point3;
using g1_perception::slugify;
using g1_perception::supportHeight;
using g1_perception::supportRing;

constexpr double kFocal   = 432.98;
constexpr double kCentreX = 424.0;
constexpr double kCentreY = 240.0;
const Intrinsics kIntrinsics{ kFocal, kFocal, kCentreX, kCentreY };
const Point3     kUp{ 0.0, 0.0, 1.0 };

/// A depth image filled with one value, optionally with padded rows.
class DepthImage
{
public:
    DepthImage(std::uint32_t width, std::uint32_t height, double value, std::uint32_t padding = 0)
      : width_(width)
      , height_(height)
      , step_((width * sizeof(float)) + padding)
    {
        bytes_.assign(static_cast<std::size_t>(step_) * height_, 0);
        for (std::uint32_t v = 0; v < height_; ++v)
        {
            for (std::uint32_t u = 0; u < width_; ++u)
            {
                set(u, v, value);
            }
        }
    }

    void set(std::uint32_t u, std::uint32_t v, double value)
    {
        const auto        metres = static_cast<float>(value);
        const std::size_t offset = (static_cast<std::size_t>(v) * step_) + (u * sizeof(float));
        std::memcpy(bytes_.data() + offset, &metres, sizeof(float));
    }

    [[nodiscard]] DepthView view() const { return { bytes_, width_, height_, step_ }; }

private:
    std::uint32_t             width_;
    std::uint32_t             height_;
    std::uint32_t             step_;
    std::vector<std::uint8_t> bytes_;
};

MaskView filledMask(
    std::vector<std::uint8_t>& storage, std::uint32_t x, std::uint32_t y, std::uint32_t width,
    std::uint32_t height)
{
    storage.assign(static_cast<std::size_t>(width) * height, 255);
    return { storage, x, y, width, height };
}

TEST(Slugify, MakesAPhraseIntoAnObjectId)
{
    EXPECT_EQ(slugify("Red Cube"), "red_cube");
    EXPECT_EQ(slugify("red  cube!"), "red_cube");
    EXPECT_EQ(slugify("  "), "");
}

TEST(Erode, ShrinksASquareByOneRing)
{
    std::vector<std::uint8_t> storage;
    const MaskView            mask = filledMask(storage, 0, 0, 5, 5);

    const std::vector<std::uint8_t> eroded = erodeMask(mask, 1);

    int kept = 0;
    for (const std::uint8_t value : eroded)
    {
        kept += value != 0 ? 1 : 0;
    }
    EXPECT_EQ(kept, 9) << "a 5x5 square should leave its 3x3 core";
}

TEST(Erode, RemovesAOnePixelLine)
{
    std::vector<std::uint8_t> storage(25, 0);
    for (std::uint32_t row = 0; row < 5; ++row)
    {
        storage[(row * 5) + 2] = 255;
    }
    const MaskView mask{ storage, 0, 0, 5, 5 };

    const std::vector<std::uint8_t> eroded = erodeMask(mask, 1);

    EXPECT_THAT(eroded, testing::Each(testing::Eq(0)));
}

TEST(Deproject, RecoversAKnownPlane)
{
    const DepthImage          depth(848, 480, 0.5);
    std::vector<std::uint8_t> storage;
    const MaskView            centre = filledMask(storage, 424, 240, 1, 1);

    const std::vector<Point3> at_centre =
        deproject(depth.view(), centre, storage, kIntrinsics, 0.1, 3.0);

    ASSERT_EQ(at_centre.size(), 1U);
    EXPECT_NEAR(at_centre.front().x, 0.0, 1e-9);
    EXPECT_NEAR(at_centre.front().y, 0.0, 1e-9);
    EXPECT_NEAR(at_centre.front().z, 0.5, 1e-6);

    const MaskView            offset = filledMask(storage, 524, 240, 1, 1);
    const std::vector<Point3> shifted =
        deproject(depth.view(), offset, storage, kIntrinsics, 0.1, 3.0);

    ASSERT_EQ(shifted.size(), 1U);
    EXPECT_NEAR(shifted.front().x, 100.0 * 0.5 / kFocal, 1e-6);
}

TEST(Deproject, DropsNaNInfZeroAndOutOfRange)
{
    DepthImage depth(10, 10, 0.5);
    depth.set(0, 0, std::numeric_limits<double>::quiet_NaN());
    depth.set(1, 0, std::numeric_limits<double>::infinity());
    depth.set(2, 0, 0.0);
    depth.set(3, 0, 9.0);

    std::vector<std::uint8_t> storage;
    const MaskView            mask = filledMask(storage, 0, 0, 10, 1);

    const std::vector<Point3> points =
        deproject(depth.view(), mask, storage, kIntrinsics, 0.1, 3.0);

    EXPECT_EQ(points.size(), 6U) << "four of the ten pixels carry no usable depth";
}

TEST(Deproject, RespectsStepWhenRowsArePadded)
{
    DepthImage depth(10, 10, 0.5, /*padding=*/8);
    depth.set(4, 3, 0.9);

    std::vector<std::uint8_t> storage;
    const MaskView            mask = filledMask(storage, 4, 3, 1, 1);

    const std::vector<Point3> points =
        deproject(depth.view(), mask, storage, kIntrinsics, 0.1, 3.0);

    ASSERT_EQ(points.size(), 1U);
    EXPECT_NEAR(points.front().z, 0.9, 1e-6);
}

TEST(SupportRing, TakesPixelsOutsideTheMaskOnly)
{
    const DepthImage          depth(20, 20, 0.8);
    std::vector<std::uint8_t> storage;
    const MaskView            mask = filledMask(storage, 5, 5, 4, 4);

    const std::vector<Point3> ring = supportRing(depth.view(), mask, kIntrinsics, 2, 0.1, 3.0);

    // An 8x8 box around a 4x4 mask, minus the mask itself.
    EXPECT_EQ(ring.size(), (8U * 8U) - (4U * 4U));
    for (const Point3& point : ring)
    {
        EXPECT_NEAR(point.z, 0.8, 1e-6);
    }
}

TEST(GateByMedianDepth, KeepsTheNearMode)
{
    std::vector<Point3> points;
    points.reserve(25);
    for (int i = 0; i < 20; ++i)
    {
        points.push_back({ 0.0, 0.0, 0.50 });
    }
    for (int i = 0; i < 5; ++i)
    {
        points.push_back({ 0.0, 0.0, 0.90 });
    }

    gateByMedianDepth(points, 0.10);

    EXPECT_EQ(points.size(), 20U);
}

/// The top face of a box, sampled on a grid, rotated about the up axis.
std::vector<Point3> rotatedTopFace(double size_x, double size_y, double height, double yaw)
{
    std::vector<Point3> points;
    for (int i = 0; i <= 20; ++i)
    {
        for (int j = 0; j <= 20; ++j)
        {
            const double local_x = (-0.5 + (i / 20.0)) * size_x;
            const double local_y = (-0.5 + (j / 20.0)) * size_y;
            points.push_back({ (local_x * std::cos(yaw)) - (local_y * std::sin(yaw)),
                               (local_x * std::sin(yaw)) + (local_y * std::cos(yaw)),
                               height });
        }
    }
    return points;
}

TEST(FitOrientedBox, RecoversYawOfATiltedFace)
{
    const double              yaw    = 30.0 * M_PI / 180.0;
    const std::vector<Point3> points = rotatedTopFace(0.10, 0.04, 0.83, yaw);

    const auto box = fitOrientedBox(points, kUp, 0.80, 0.01, 0.40);

    ASSERT_TRUE(box.has_value());
    EXPECT_NEAR(box->size_x, 0.10, 1e-3);
    EXPECT_NEAR(box->size_y, 0.04, 1e-3);
    const double recovered = std::atan2(box->axis_x.y, box->axis_x.x) * 180.0 / M_PI;
    EXPECT_NEAR(std::fmod(recovered + 180.0, 180.0), 30.0, 1.0);
}

TEST(FitOrientedBox, DoesNotInflateASquareFootprint)
{
    // A square has no principal direction, so a covariance fit lands at 45 degrees and reports a
    // 6 cm cube as 8.5 cm across, which then becomes a collision box half again too wide.
    const std::vector<Point3> points = rotatedTopFace(0.06, 0.06, 0.83, 0.0);

    const auto box = fitOrientedBox(points, kUp, 0.80, 0.01, 0.40);

    ASSERT_TRUE(box.has_value());
    EXPECT_NEAR(box->size_x, 0.06, 2e-3);
    EXPECT_NEAR(box->size_y, 0.06, 2e-3);
}

TEST(FitOrientedBox, UsesTheSupportHeightForTheVerticalExtent)
{
    // The visible cap of a sphere resting on a table: everything from its equator up.
    const double        radius = 0.037;
    const double        table  = 0.80;
    std::vector<Point3> points;
    for (int i = 0; i <= 20; ++i)
    {
        for (int j = 0; j <= 20; ++j)
        {
            const double polar   = (M_PI / 2.0) * (i / 20.0);
            const double azimuth = 2.0 * M_PI * (j / 20.0);
            points.push_back({ radius * std::sin(polar) * std::cos(azimuth),
                               radius * std::sin(polar) * std::sin(azimuth),
                               table + radius + (radius * std::cos(polar)) });
        }
    }

    const auto with_support = fitOrientedBox(points, kUp, table, 0.01, 0.40);
    const auto without      = fitOrientedBox(points, kUp, std::nullopt, 0.01, 0.40);

    ASSERT_TRUE(with_support.has_value());
    ASSERT_TRUE(without.has_value());
    EXPECT_NEAR(with_support->size_z, 2.0 * radius, 2e-3);
    EXPECT_NEAR(with_support->centre.z, table + radius, 2e-3);
    EXPECT_NEAR(without->size_z, radius, 2e-3) << "without the table only the visible cap counts";
}

TEST(SupportHeight, IgnoresRingPointsOffTheSurfaceTheObjectStandsOn)
{
    // A cube resting at 0.80 on a table, with a ring that catches the table, the floor a long
    // way below it, and a neighbour standing on it. Only the table may reach the median.
    const std::vector<Point3> object{ { 0.0, 0.0, 0.80 }, { 0.01, 0.0, 0.82 } };
    std::vector<Point3>       ring{ { 0.05, 0.0, 0.40 }, { 0.06, 0.0, 0.40 } };
    for (int i = 0; i < 8; ++i)
    {
        ring.push_back({ 0.05 + (0.001 * i), 0.0, 0.799 });
        ring.push_back({ 0.10, 0.001 * i, 0.95 });
    }

    const std::optional<double> height = supportHeight(ring, object, kUp, 0.06, 4);

    ASSERT_TRUE(height.has_value());
    EXPECT_NEAR(*height, 0.799, 1e-9);
}

TEST(SupportHeight, RefusesARingThatLandedOnNothingFlat)
{
    const std::vector<Point3> object{ { 0.0, 0.0, 0.80 } };
    const std::vector<Point3> ring{ { 0.05, 0.0, 0.40 }, { 0.06, 0.0, 0.95 } };

    EXPECT_FALSE(supportHeight(ring, object, kUp, 0.06, 4).has_value());
    EXPECT_FALSE(supportHeight({}, object, kUp, 0.06, 0).has_value());
}

TEST(FitOrientedBox, RejectsAMaskThatRanOntoTheTable)
{
    const std::vector<Point3> points = rotatedTopFace(0.60, 0.50, 0.80, 0.0);

    EXPECT_FALSE(fitOrientedBox(points, kUp, 0.79, 0.01, 0.40).has_value());
}

TEST(FitOrientedBox, RefusesTooFewPoints)
{
    const std::vector<Point3> points{ { 0.0, 0.0, 0.8 }, { 0.01, 0.0, 0.8 } };

    EXPECT_FALSE(fitOrientedBox(points, kUp, 0.79, 0.01, 0.40).has_value());
}

}  // namespace
