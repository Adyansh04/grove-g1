/**
 * @file test_object_map.cpp
 * @brief Object fusion against ray-cast boxes, no ROS.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <limits>
#include <numbers>

#include "g1_world_model/object_map.hpp"

namespace g1_world_model
{
namespace
{

struct Box
{
    std::string     label;
    Eigen::Vector3d low;
    Eigen::Vector3d high;
};

struct Camera
{
    Intrinsics intrinsics{ 433.0, 433.0, 424.0, 240.0, 848, 480 };
    double     height = 1.21;
    double     pitch  = 0.8307767;

    [[nodiscard]] Eigen::Isometry3d pose(double x, double y, double yaw) const
    {
        const Eigen::Vector3d forward(
            std::cos(pitch) * std::cos(yaw),
            std::cos(pitch) * std::sin(yaw),
            -std::sin(pitch));
        const Eigen::Vector3d right(std::sin(yaw), -std::cos(yaw), 0.0);
        Eigen::Isometry3d     pose = Eigen::Isometry3d::Identity();
        pose.linear().col(0)       = right;
        pose.linear().col(1)       = forward.cross(right);
        pose.linear().col(2)       = forward;
        pose.translation()         = Eigen::Vector3d(x, y, height);
        return pose;
    }
};

/// Nearest hit of a ray with the floor or any box, as distance along the ray.
double castRay(
    const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const std::vector<Box>& boxes)
{
    double nearest =
        direction.z() < 0.0 ? -origin.z() / direction.z() : std::numeric_limits<double>::infinity();
    for (const Box& box : boxes)
    {
        double enter = 0.0;
        double leave = std::numeric_limits<double>::infinity();
        for (int axis = 0; axis < 3; ++axis)
        {
            if (std::abs(direction[axis]) < 1e-12)
            {
                if (origin[axis] < box.low[axis] || origin[axis] > box.high[axis])
                {
                    enter = leave + 1.0;
                }
                continue;
            }
            double t0 = (box.low[axis] - origin[axis]) / direction[axis];
            double t1 = (box.high[axis] - origin[axis]) / direction[axis];
            if (t0 > t1)
            {
                std::swap(t0, t1);
            }
            enter = std::max(enter, t0);
            leave = std::min(leave, t1);
        }
        if (enter <= leave && enter > 0.0)
        {
            nearest = std::min(nearest, enter);
        }
    }
    return nearest;
}

struct Frame
{
    std::vector<float>                     depth;
    std::vector<std::vector<std::uint8_t>> masks;
    std::vector<MaskInput>                 inputs;
    FrameInput                             input;
};

/// Renders depth and cuts one mask per box from it, as the mock detector does.
Frame render(
    const Camera& camera, const Eigen::Isometry3d& pose, const std::vector<Box>& scene,
    const std::vector<Box>& detected, double stamp)
{
    const Intrinsics& k = camera.intrinsics;
    Frame             frame;
    frame.depth.assign(
        static_cast<std::size_t>(k.width) * k.height,
        std::numeric_limits<float>::quiet_NaN());
    for (int v = 0; v < k.height; ++v)
    {
        for (int u = 0; u < k.width; ++u)
        {
            const Eigen::Vector3d ray((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
            const double t = castRay(pose.translation(), pose.linear() * ray.normalized(), scene);
            if (std::isfinite(t) && t < 8.0)
            {
                frame.depth[(static_cast<std::size_t>(v) * k.width) + u] =
                    static_cast<float>(t / ray.norm());
            }
        }
    }
    for (const Box& box : detected)
    {
        std::vector<std::uint8_t> full(static_cast<std::size_t>(k.width) * k.height, 0);
        int                       x0 = k.width;
        int                       y0 = k.height;
        int                       x1 = -1;
        int                       y1 = -1;
        for (int v = 0; v < k.height; ++v)
        {
            for (int u = 0; u < k.width; ++u)
            {
                const float z = frame.depth[(static_cast<std::size_t>(v) * k.width) + u];
                if (!std::isfinite(z))
                {
                    continue;
                }
                const Eigen::Vector3d point =
                    pose * Eigen::Vector3d((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z);
                if ((point.array() >= box.low.array() - 0.01).all() &&
                    (point.array() <= box.high.array() + 0.01).all())
                {
                    full[(static_cast<std::size_t>(v) * k.width) + u] = 255;
                    x0                                                = std::min(x0, u);
                    y0                                                = std::min(y0, v);
                    x1                                                = std::max(x1, u);
                    y1                                                = std::max(y1, v);
                }
            }
        }
        if (x1 < 0)
        {
            continue;
        }
        std::vector<std::uint8_t> crop(static_cast<std::size_t>(x1 - x0 + 1) * (y1 - y0 + 1));
        for (int v = y0; v <= y1; ++v)
        {
            for (int u = x0; u <= x1; ++u)
            {
                crop[(static_cast<std::size_t>(v - y0) * (x1 - x0 + 1)) + (u - x0)] =
                    full[(static_cast<std::size_t>(v) * k.width) + u];
            }
        }
        frame.masks.push_back(std::move(crop));
        MaskInput input;
        input.label  = box.label;
        input.score  = 0.9F;
        input.x      = x0;
        input.y      = y0;
        input.width  = x1 - x0 + 1;
        input.height = y1 - y0 + 1;
        frame.inputs.push_back(input);
    }
    for (std::size_t i = 0; i < frame.inputs.size(); ++i)
    {
        frame.inputs[i].mask = frame.masks[i].data();
    }
    frame.input.stamp = stamp;
    frame.input.depth = { frame.depth.data(), k.width, k.height, static_cast<std::size_t>(k.width) };
    frame.input.intrinsics      = k;
    frame.input.map_from_camera = pose;
    return frame;
}

const Box kTable{ "table", { 2.0, 1.0, 0.0 }, { 3.2, 1.8, 0.75 } };
const Box kMug{ "mug", { 2.5, 1.3, 0.75 }, { 2.58, 1.38, 0.85 } };
const Box kWall{ "wall", { -1.0, 5.0, 0.0 }, { 8.0, 5.2, 2.5 } };

int countActive(const ObjectMap& map)
{
    return static_cast<int>(
        std::count_if(map.objects().begin(), map.objects().end(), [](const MappedObject& o) {
            return o.state == ObjectState::kActive;
        }));
}

const MappedObject* byLabel(const ObjectMap& map, const std::string& label)
{
    for (const MappedObject& object : map.objects())
    {
        if (object.label() == label && object.state != ObjectState::kRemoved)
        {
            return &object;
        }
    }
    return nullptr;
}

TEST(ObjectMap, FusesATableAndTheMugOnItAcrossViews)
{
    const Camera           camera;
    const std::vector<Box> scene{ kTable, kMug, kWall };
    ObjectMap              map;

    // From the table's near side, then from its left end.
    Frame first =
        render(camera, camera.pose(2.6, 0.2, std::numbers::pi / 2.0), scene, { kTable, kMug }, 1.0);
    map.integrate(first.inputs, first.input);
    Frame second = render(camera, camera.pose(1.2, 1.4, 0.0), scene, { kTable, kMug }, 2.0);
    map.integrate(second.inputs, second.input);

    ASSERT_EQ(countActive(map), 2);
    const MappedObject* table = byLabel(map, "table");
    const MappedObject* mug   = byLabel(map, "mug");
    ASSERT_NE(table, nullptr);
    ASSERT_NE(mug, nullptr);
    EXPECT_EQ(table->observations, 2);
    EXPECT_EQ(mug->observations, 2);
    EXPECT_EQ(mug->support, table->id);
    EXPECT_NEAR(mug->centroid.x(), 2.54, 0.05);
    EXPECT_NEAR(mug->centroid.y(), 1.34, 0.05);
    EXPECT_NEAR(table->z_max, 0.75, 0.06);
    // The table's box spans what was seen of it: most of its top.
    EXPECT_GT(table->box_size.maxCoeff(), 0.9);

    const std::vector<Surface> surfaces = map.surfaces();
    ASSERT_EQ(surfaces.size(), 1U);
    EXPECT_NEAR(surfaces[0].height, 0.75, 0.06);
}

TEST(ObjectMap, ToleratesLocalisationDriftBetweenVisits)
{
    const Camera           camera;
    const std::vector<Box> scene{ kTable, kMug, kWall };
    ObjectMap              map;
    Frame                  first =
        render(camera, camera.pose(2.6, 0.2, std::numbers::pi / 2.0), scene, { kTable, kMug }, 1.0);
    map.integrate(first.inputs, first.input);

    // Same view, but the map now believes the robot stands 0.15 m further right.
    Eigen::Isometry3d drifted = camera.pose(2.6, 0.2, std::numbers::pi / 2.0);
    Frame             again   = render(camera, drifted, scene, { kTable, kMug }, 2.0);
    again.input.map_from_camera.translation() += Eigen::Vector3d(0.15, 0.10, 0.0);
    map.integrate(again.inputs, again.input);

    EXPECT_EQ(countActive(map), 2);
}

TEST(ObjectMap, ForgetsAnObjectThatIsGone)
{
    const Camera camera;
    ObjectMap    map;
    const auto   pose = camera.pose(2.6, 0.6, std::numbers::pi / 2.0);
    Frame        seen = render(camera, pose, { kTable, kMug, kWall }, { kTable, kMug }, 1.0);
    map.integrate(seen.inputs, seen.input);
    ASSERT_NE(byLabel(map, "mug"), nullptr);

    for (int step = 0; step < 12; ++step)
    {
        Frame gone = render(camera, pose, { kTable, kWall }, { kTable }, 2.0 + step);
        map.integrate(gone.inputs, gone.input);
    }
    const auto mug =
        std::find_if(map.objects().begin(), map.objects().end(), [](const MappedObject& o) {
            return o.label() == "mug";
        });
    ASSERT_NE(mug, map.objects().end());
    EXPECT_EQ(mug->state, ObjectState::kRemoved);
    EXPECT_NE(byLabel(map, "table"), nullptr);
}

TEST(ObjectMap, KeepsAnUndetectedObjectThatIsStillThere)
{
    const Camera camera;
    ObjectMap    map;
    const auto   pose = camera.pose(2.6, 0.6, std::numbers::pi / 2.0);
    Frame        seen = render(camera, pose, { kTable, kMug, kWall }, { kTable, kMug }, 1.0);
    map.integrate(seen.inputs, seen.input);

    // The detector misses the mug, but depth still finds it where it was.
    for (int step = 0; step < 12; ++step)
    {
        Frame missed = render(camera, pose, { kTable, kMug, kWall }, { kTable }, 2.0 + step);
        map.integrate(missed.inputs, missed.input);
    }
    const MappedObject* mug = byLabel(map, "mug");
    ASSERT_NE(mug, nullptr);
    EXPECT_EQ(mug->state, ObjectState::kActive);
}

TEST(ObjectMap, MergesTwoSidesOfATableThatShareNoVoxels)
{
    // A chair hides the middle of a 3 m table, so each view masks one end of it.
    const Camera camera;
    const Box    table{ "table", { 1.0, 1.0, 0.0 }, { 4.0, 1.8, 0.75 } };
    const Box    left{ "table", { 1.0, 1.0, 0.0 }, { 2.5, 1.8, 0.75 } };
    const Box    right{ "table", { 2.55, 1.0, 0.0 }, { 4.0, 1.8, 0.75 } };
    ObjectMap    map;
    Frame        first =
        render(camera, camera.pose(1.75, 0.0, std::numbers::pi / 2.0), { table }, { left }, 1.0);
    map.integrate(first.inputs, first.input);
    Frame second =
        render(camera, camera.pose(3.25, 0.0, std::numbers::pi / 2.0), { table }, { right }, 2.0);
    map.integrate(second.inputs, second.input);
    ASSERT_EQ(countActive(map), 2);

    EXPECT_EQ(map.mergeDuplicates(), 1);
    ASSERT_EQ(countActive(map), 1);
    EXPECT_GT(byLabel(map, "table")->box_size.maxCoeff(), 2.5);
}

TEST(ObjectMap, JoinsTheEndsOfATableButNotTwoChairs)
{
    // Each gap is 0.6 m: a chair hides a table's middle, and two chairs stand side by side.
    const Camera           camera;
    const Box              table{ "table", { 1.0, 1.0, 0.0 }, { 4.0, 1.8, 0.75 } };
    const Box              left{ "table", { 1.0, 1.0, 0.0 }, { 1.9, 1.8, 0.75 } };
    const Box              right{ "table", { 2.5, 1.0, 0.0 }, { 4.0, 1.8, 0.75 } };
    const Box              chair_a{ "chair", { 5.0, 1.0, 0.0 }, { 5.45, 1.45, 0.9 } };
    const Box              chair_b{ "chair", { 6.05, 1.0, 0.0 }, { 6.5, 1.45, 0.9 } };
    ObjectMap              map;
    const std::vector<Box> scene{ table, chair_a, chair_b };
    for (const auto& [x, masks] :
         std::vector<std::pair<double, std::vector<Box>>>{ { 1.45, { left } },
                                                           { 3.25, { right } },
                                                           { 5.75, { chair_a, chair_b } } })
    {
        Frame frame = render(camera, camera.pose(x, -0.2, std::numbers::pi / 2.0), scene, masks, x);
        map.integrate(frame.inputs, frame.input);
    }
    ASSERT_EQ(countActive(map), 4);

    map.mergeDuplicates();
    EXPECT_EQ(countActive(map), 3);
    EXPECT_GT(byLabel(map, "table")->box_size.maxCoeff(), 2.5);
}

TEST(ObjectMap, DropsAFragmentNoSecondSightingConfirms)
{
    const Camera camera;
    const Box    vase{ "vase", { 2.0, 1.2, 0.75 }, { 2.1, 1.3, 0.95 } };
    ObjectMap    map;
    const auto   pose  = camera.pose(2.6, 0.6, std::numbers::pi / 2.0);
    Frame        first = render(camera, pose, { kTable, vase, kWall }, { kTable, vase }, 1.0);
    map.integrate(first.inputs, first.input);
    Frame second = render(camera, pose, { kTable, vase, kWall }, { kTable }, 2.0);
    map.integrate(second.inputs, second.input);

    EXPECT_EQ(map.pruneUnconfirmed(10.0), 0);  // Still within the window: it may yet be seen.
    EXPECT_EQ(map.pruneUnconfirmed(40.0), 1);
    EXPECT_EQ(byLabel(map, "vase"), nullptr);
    EXPECT_NE(byLabel(map, "table"), nullptr);
}

TEST(ObjectMap, KeepsABookApartFromTheShelfItStandsIn)
{
    // An open shelf with a book on its lower tier. The shelf's mask takes in everything inside its
    // box, the book included, as the mock detector's does, so the book lies within the shelf.
    const Camera           camera;
    const Box              shelf{ "shelf", { 2.0, 1.4, 0.0 }, { 3.2, 1.8, 1.2 } };
    const Box              book{ "book", { 2.4, 1.5, 0.3 }, { 2.6, 1.7, 0.55 } };
    const std::vector<Box> scene{
        { "back", { 2.0, 1.75, 0.0 }, { 3.2, 1.8, 1.2 } },
        { "side", { 2.0, 1.4, 0.0 }, { 2.05, 1.8, 1.2 } },
        { "side", { 3.15, 1.4, 0.0 }, { 3.2, 1.8, 1.2 } },
        { "tier", { 2.0, 1.4, 0.25 }, { 3.2, 1.8, 0.3 } },
        book,
        kWall,
    };
    // The shelf first; then frames in which only the book is detected.
    ObjectMap map;
    for (int step = 0; step < 3; ++step)
    {
        Frame frame = render(
            camera,
            camera.pose(2.6, 0.3, std::numbers::pi / 2.0),
            scene,
            { step == 0 ? shelf : book },
            1.0 + step);
        map.integrate(frame.inputs, frame.input);
    }
    EXPECT_EQ(countActive(map), 2);
    ASSERT_NE(byLabel(map, "book"), nullptr);
    EXPECT_EQ(byLabel(map, "shelf")->votes.count("book"), 0U);
}

TEST(ObjectMap, KeysRoundTrip)
{
    const ObjectMap       map;
    const Eigen::Vector3d point(-3.21, 7.5, 0.83);
    const Eigen::Vector3d centre = map.centreOf(map.keyOf(point));
    EXPECT_LT((centre - point).cwiseAbs().maxCoeff(), map.params().voxel);
}

}  // namespace
}  // namespace g1_world_model
