/**
 * @file test_object_map.cpp
 * @brief Object fusion against ray-cast boxes, no ROS.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <functional>
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

TEST(ObjectMap, TakesNoTopFromFurnitureTallerThanTheViewReached)
{
    // A bookshelf a metre off, taller than the head camera looks up: its mask runs off the image
    // and its highest voxel is only where the view stopped. The table beside it is seen whole.
    const Camera camera;
    const Box    shelf{ "bookshelf", { 2.0, 1.8, 0.0 }, { 3.0, 2.15, 2.0 } };
    const Box    table{ "table", { 3.4, 1.6, 0.0 }, { 4.4, 2.0, 0.75 } };
    ObjectMap    map;
    Frame        frame = render(
        camera,
        camera.pose(3.2, 0.9, std::numbers::pi / 2.0),
        { shelf, table },
        { shelf, table },
        1.0);
    map.integrate(frame.inputs, frame.input);

    ASSERT_NE(byLabel(map, "bookshelf"), nullptr);
    ASSERT_NE(byLabel(map, "table"), nullptr);
    EXPECT_LT(byLabel(map, "bookshelf")->z_max, 1.2);
    const std::vector<Surface> surfaces = map.surfaces();
    ASSERT_EQ(surfaces.size(), 1U);
    EXPECT_EQ(surfaces[0].id, byLabel(map, "table")->id);
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

TEST(ObjectMap, MergesALowFragmentUnderTheBedItBelongsTo)
{
    // A bed frame, and one of its legs seen alone in another view: too low to touch the frame.
    const Camera camera;
    const Box    bed{ "bed", { 1.0, 1.2, 0.35 }, { 3.0, 2.2, 0.72 } };
    const Box    leg{ "bed", { 2.6, 1.3, 0.0 }, { 2.9, 1.6, 0.15 } };
    ObjectMap    map;
    const auto   pose  = camera.pose(2.0, 0.2, std::numbers::pi / 2.0);
    Frame        first = render(camera, pose, { bed, leg }, { bed }, 1.0);
    map.integrate(first.inputs, first.input);
    Frame second = render(camera, pose, { bed, leg }, { leg }, 2.0);
    map.integrate(second.inputs, second.input);
    ASSERT_EQ(countActive(map), 2);

    map.mergeDuplicates();
    EXPECT_EQ(countActive(map), 1);
}

TEST(ObjectMap, OffersASecondLookAtAGlimpseButNotAtAPieceOfWhatItKnows)
{
    // A bed seen twice, a vase in front of it once, and one of the bed's legs once on its own.
    const Camera camera;
    const Box    bed{ "bed", { 1.0, 1.2, 0.35 }, { 3.0, 2.2, 0.72 } };
    const Box    leg{ "bed", { 2.6, 1.3, 0.0 }, { 2.9, 1.6, 0.15 } };
    const Box    vase{ "vase", { 1.5, 0.9, 0.0 }, { 1.7, 1.1, 0.4 } };
    ObjectMap    map;
    const auto   pose  = camera.pose(2.0, 0.2, std::numbers::pi / 2.0);
    Frame        first = render(camera, pose, { bed, leg, vase }, { bed, vase }, 1.0);
    map.integrate(first.inputs, first.input);
    Frame second = render(camera, pose, { bed, leg, vase }, { bed, leg }, 2.0);
    map.integrate(second.inputs, second.input);
    ASSERT_EQ(countActive(map), 3);

    const std::vector<const MappedObject*> glimpses = map.glimpses();
    ASSERT_EQ(glimpses.size(), 1U);
    EXPECT_EQ(glimpses[0]->label(), "vase");
    EXPECT_NEAR(glimpses[0]->seen_from.x(), 2.0, 1e-9);
    EXPECT_NEAR(glimpses[0]->seen_from.y(), 0.2, 1e-9);
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

    const double window = ObjectMapParams{}.confirm_s;
    EXPECT_EQ(map.pruneUnconfirmed(window - 10.0), 0);  // Within the window: it may yet be seen.
    EXPECT_EQ(map.pruneUnconfirmed(window + 10.0), 1);
    EXPECT_EQ(byLabel(map, "vase"), nullptr);
    EXPECT_NE(byLabel(map, "table"), nullptr);
}

TEST(ObjectMap, JoinsTwoGlimpsesOfATableBeforeDroppingEither)
{
    // One end of the table seen once, the other once more than the confirmation window later.
    const Camera    camera;
    const Box       table{ "table", { 1.0, 1.0, 0.0 }, { 4.0, 1.8, 0.75 } };
    const Box       left{ "table", { 1.0, 1.0, 0.0 }, { 2.5, 1.8, 0.75 } };
    const Box       right{ "table", { 2.55, 1.0, 0.0 }, { 4.0, 1.8, 0.75 } };
    ObjectMapParams params;
    params.merge_every = 1;
    ObjectMap map(params);
    Frame     first =
        render(camera, camera.pose(1.75, 0.0, std::numbers::pi / 2.0), { table }, { left }, 1.0);
    map.integrate(first.inputs, first.input);
    Frame second = render(
        camera,
        camera.pose(3.25, 0.0, std::numbers::pi / 2.0),
        { table },
        { right },
        params.confirm_s + 10.0);
    map.integrate(second.inputs, second.input);

    ASSERT_EQ(countActive(map), 1);
    EXPECT_GT(byLabel(map, "table")->box_size.maxCoeff(), 2.5);
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

/// An object made of voxels at the given floor points, 0.2-0.6 m up, fitted with the walls at
/// @p frame_yaw.
MappedObject fitted(const std::vector<Eigen::Vector2d>& points, double frame_yaw)
{
    ObjectMap    map;
    MappedObject object;
    object.id           = 1;
    object.votes["box"] = 1.0F;
    object.observations = 3;
    for (const Eigen::Vector2d& point : points)
    {
        for (int layer = 1; layer <= 3; ++layer)
        {
            object.voxels.push_back(map.keyOf({ point.x(), point.y(), 0.2 * layer }));
        }
    }
    std::sort(object.voxels.begin(), object.voxels.end());
    object.voxels.erase(
        std::unique(object.voxels.begin(), object.voxels.end()),
        object.voxels.end());
    map.restore({ object });
    map.setFrame(frame_yaw);
    return map.objects().front();
}

/// Points every 2 cm over a rectangle turned by @p yaw about @p centre, kept where @p keep says.
std::vector<Eigen::Vector2d> rectangle(
    Eigen::Vector2d centre, double length, double width, double yaw,
    const std::function<bool(double, double)>& keep = [](double, double) { return true; })
{
    std::vector<Eigen::Vector2d> points;
    for (int i = 0; i * 0.02 <= length; ++i)
    {
        for (int j = 0; j * 0.02 <= width; ++j)
        {
            const double u = (i * 0.02) - (0.5 * length);
            const double v = (j * 0.02) - (0.5 * width);
            if (keep(u, v))
            {
                points.emplace_back(
                    centre.x() + (u * std::cos(yaw)) - (v * std::sin(yaw)),
                    centre.y() + (u * std::sin(yaw)) + (v * std::cos(yaw)));
            }
        }
    }
    return points;
}

double axisError(double a, double b)
{
    return std::abs(std::remainder(a - b, std::numbers::pi / 2.0));
}

TEST(ObjectBox, LaysAPartlySeenFootprintAlongTheWalls)
{
    // Half a 1.0 x 0.6 m fridge, cut along its diagonal: the tightest box around it is diagonal.
    const MappedObject fridge = fitted(
        rectangle(
            { 2.0, 1.0 },
            1.0,
            0.6,
            0.0,
            [](double u, double v) { return (u / 0.5) + (v / 0.3) <= 0.0; }),
        0.0);
    EXPECT_LT(axisError(fridge.box_yaw, 0.0), 0.01);
    EXPECT_NEAR(fridge.box_size.maxCoeff(), 1.0, 0.1);
    EXPECT_NEAR(fridge.box_size.minCoeff(), 0.6, 0.1);
}

TEST(ObjectBox, LaysARoundBinAlongTheWallsWhateverTheirAngle)
{
    const auto bin = rectangle({ 1.0, 1.0 }, 0.3, 0.3, 0.0, [](double u, double v) {
        return std::hypot(u, v) <= 0.15;
    });
    for (const double walls : { 0.0, 0.5 })
    {
        const MappedObject fitted_bin = fitted(bin, walls);
        EXPECT_LT(axisError(fitted_bin.box_yaw, walls), 0.01) << walls;
        EXPECT_LT(fitted_bin.box_size.maxCoeff(), 0.42) << walls;
    }
}

TEST(ObjectBox, TurnsWithALongObjectSetAtAnAngle)
{
    const double       yaw  = 35.0 * std::numbers::pi / 180.0;
    const MappedObject sofa = fitted(rectangle({ 3.0, 2.0 }, 2.0, 0.8, yaw), 0.0);
    EXPECT_LT(axisError(sofa.box_yaw, yaw), 0.05);
    EXPECT_NEAR(sofa.box_size.maxCoeff(), 2.0, 0.1);
    EXPECT_NEAR(sofa.box_size.minCoeff(), 0.8, 0.1);
}

TEST(ObjectBox, IsNotStretchedByAFewStrayVoxels)
{
    std::vector<Eigen::Vector2d> table  = rectangle({ 2.0, 2.0 }, 1.0, 1.0, 0.0);
    const std::size_t            strays = table.size() / 100;
    for (std::size_t i = 0; i < strays; ++i)
    {
        table.emplace_back(2.9, 2.0 + (0.01 * static_cast<double>(i % 10)));
    }
    const MappedObject box = fitted(table, 0.0);
    EXPECT_LT(box.box_size.maxCoeff(), 1.1);
    EXPECT_NEAR(box.box_centre.x(), 2.0, 0.05);
}

TEST(MapFit, TakesTheOutlineTheMapShowsUnderEachObject)
{
    const GridGeometry geometry{ 0.05, 0.0, 0.0, 120, 80 };
    cv::Mat            furniture(geometry.height, geometry.width, CV_8UC1, cv::Scalar(0));
    const auto         blob = [&](double x0, double y0, double x1, double y1) {
        furniture(cv::Rect(
            geometry.toCell(x0, y0).x,
            geometry.toCell(x0, y0).y,
            geometry.toCell(x1, y1).x - geometry.toCell(x0, y0).x,
            geometry.toCell(x1, y1).y - geometry.toCell(x0, y0).y)) = 255;
    };
    blob(2.0, 1.0, 4.0, 2.0);  // A table.
    blob(2.5, 2.3, 3.0, 2.8);  // A chair.
    blob(0.5, 0.5, 1.5, 1.5);  // Two nightstands side by side, one blob in the map.
    blob(0.3, 2.5, 2.0, 3.8);  // A bed, under something much smaller.

    const auto object = [](int id, double x, double y, double sx, double sy, int support = 0) {
        MappedObject out;
        out.id         = id;
        out.box_centre = { x, y };
        out.box_size   = { sx, sy };
        out.support    = support;
        return out;
    };
    const std::vector<MappedObject> objects{
        object(1, 3.2, 1.6, 1.6, 0.7),     // The table, fused from its near half.
        object(2, 2.75, 2.55, 0.3, 0.3),   // The chair.
        object(3, 3.0, 1.5, 0.1, 0.1, 1),  // A mug on the table.
        object(4, 0.8, 1.0, 0.4, 0.8),     // The nightstands, each taking its half.
        object(5, 1.2, 1.0, 0.4, 0.8),
        object(6, 1.2, 3.1, 0.3, 0.3),  // A bin on a far bigger blob.
    };
    const std::map<int, Footprint> fitted = fitToMap(objects, furniture, geometry);

    ASSERT_TRUE(fitted.contains(1));
    EXPECT_NEAR(fitted.at(1).centre.x(), 3.0, 0.05);
    EXPECT_NEAR(fitted.at(1).centre.y(), 1.5, 0.05);
    EXPECT_NEAR(fitted.at(1).size.x(), 2.0, 0.06);
    EXPECT_NEAR(fitted.at(1).size.y(), 1.0, 0.06);
    ASSERT_TRUE(fitted.contains(2));
    EXPECT_NEAR(fitted.at(2).size.x(), 0.5, 0.06);
    EXPECT_FALSE(fitted.contains(3));
    ASSERT_TRUE(fitted.contains(4));
    ASSERT_TRUE(fitted.contains(5));
    EXPECT_NEAR(fitted.at(4).centre.x(), 0.75, 0.05);
    EXPECT_NEAR(fitted.at(5).centre.x(), 1.25, 0.05);
    EXPECT_NEAR(fitted.at(4).size.x(), 0.5, 0.06);
    EXPECT_FALSE(fitted.contains(6));
}

TEST(MapFit, SplitsATableFromTheChairsPushedUnderIt)
{
    // The scan band sees a table and the chairs against it as one blob.
    const GridGeometry geometry{ 0.05, 0.0, 0.0, 120, 80 };
    cv::Mat            furniture(geometry.height, geometry.width, CV_8UC1, cv::Scalar(0));
    const auto         blob = [&](double x0, double y0, double x1, double y1) {
        const CellIndex low  = geometry.toCell(x0, y0);
        const CellIndex high = geometry.toCell(x1, y1);
        furniture(cv::Rect(low.x, low.y, high.x - low.x, high.y - low.y)) = 255;
    };
    blob(2.0, 1.0, 4.0, 2.0);  // The table.
    blob(2.5, 2.0, 3.0, 2.5);  // A chair on one side...
    blob(3.2, 0.5, 3.7, 1.0);  // ...and one on the other.
    const auto object = [](int id, double x, double y, double sx, double sy) {
        MappedObject out;
        out.id         = id;
        out.box_centre = { x, y };
        out.box_size   = { sx, sy };
        return out;
    };
    // The table fused from its near end only; the chairs about right.
    const std::map<int, Footprint> fitted = fitToMap(
        { object(1, 3.4, 1.5, 1.2, 0.8),
          object(2, 2.75, 2.25, 0.4, 0.4),
          object(3, 3.45, 0.75, 0.4, 0.4) },
        furniture,
        geometry);

    ASSERT_TRUE(fitted.contains(1));
    EXPECT_NEAR(fitted.at(1).centre.x(), 3.0, 0.05);
    EXPECT_NEAR(fitted.at(1).size.x(), 2.0, 0.06);
    EXPECT_NEAR(fitted.at(1).size.y(), 1.0, 0.06);
    // A chair may take a sliver of table edge nearer its box than the table's: still a chair.
    ASSERT_TRUE(fitted.contains(2));
    EXPECT_NEAR(fitted.at(2).centre.y(), 2.25, 0.05);
    EXPECT_LT(fitted.at(2).size.x(), 0.7);
    ASSERT_TRUE(fitted.contains(3));
    EXPECT_NEAR(fitted.at(3).centre.y(), 0.75, 0.05);
    EXPECT_LT(fitted.at(3).size.x(), 0.7);
}

TEST(MapFit, GivesACounterItsBackRatherThanTheStoveBesideIt)
{
    // A counter and the stove at its end, one blob; the counter fused from its front 0.3 m only.
    const GridGeometry geometry{ 0.05, 0.0, 0.0, 120, 60 };
    cv::Mat            furniture(geometry.height, geometry.width, CV_8UC1, cv::Scalar(0));
    furniture(cv::Rect(20, 20, 52, 13)) = 255;
    furniture(cv::Rect(72, 20, 15, 14)) = 255;
    const auto object                   = [](int id, double x, double y, double sx, double sy) {
        MappedObject out;
        out.id         = id;
        out.box_centre = { x, y };
        out.box_size   = { sx, sy };
        return out;
    };
    const std::map<int, Footprint> fitted = fitToMap(
        { object(1, 2.3, 1.15, 2.6, 0.3), object(2, 3.97, 1.34, 0.74, 0.68) },
        furniture,
        geometry);

    ASSERT_TRUE(fitted.contains(1));
    EXPECT_NEAR(fitted.at(1).size.y(), 0.65, 0.06);
    ASSERT_TRUE(fitted.contains(2));
    EXPECT_NEAR(fitted.at(2).centre.x(), 3.97, 0.06);
    EXPECT_NEAR(fitted.at(2).size.x(), 0.75, 0.12);
}

TEST(MapFit, TakesTheMapsOutlineForABoxThatDriftedOverFreeFloor)
{
    // A crate against a wall whose voxels SLAM smeared 2.3 m along the floor beside it.
    const GridGeometry geometry{ 0.05, 0.0, 0.0, 80, 60 };
    cv::Mat            plan(geometry.height, geometry.width, CV_8UC1, cv::Scalar(kFree));
    cv::Mat            furniture(plan.size(), CV_8UC1, cv::Scalar(0));
    plan(cv::Rect(60, 10, 11, 22)).setTo(kOccupied);
    furniture(cv::Rect(60, 10, 11, 22)).setTo(255);
    MappedObject crate;
    crate.id         = 1;
    crate.box_centre = { 3.0, 1.1 };
    crate.box_size   = { 1.2, 2.3 };

    EXPECT_FALSE(fitToMap({ crate }, furniture, geometry).contains(1));
    const std::map<int, Footprint> fitted = fitToMap({ crate }, furniture, geometry, {}, plan);
    ASSERT_TRUE(fitted.contains(1));
    EXPECT_NEAR(fitted.at(1).size.x(), 0.55, 0.06);
    EXPECT_NEAR(fitted.at(1).size.y(), 1.1, 0.06);
}

TEST(MapFit, KeepsAPlantsBoxOverTheFloorItsLeavesCover)
{
    // The scan sees a plant's pot, the camera its leaves out over the floor round it.
    const GridGeometry geometry{ 0.05, 0.0, 0.0, 60, 60 };
    cv::Mat            plan(geometry.height, geometry.width, CV_8UC1, cv::Scalar(kFree));
    cv::Mat            furniture(plan.size(), CV_8UC1, cv::Scalar(0));
    plan(cv::Rect(28, 28, 3, 3)).setTo(kOccupied);
    furniture(cv::Rect(28, 28, 3, 3)).setTo(255);
    MappedObject plant;
    plant.id         = 1;
    plant.box_centre = { 1.475, 1.475 };
    plant.box_size   = { 0.7, 0.66 };
    EXPECT_FALSE(fitToMap({ plant }, furniture, geometry, {}, plan).contains(1));
}

TEST(MapFit, MovesEachSideToWhereTheObjectEnds)
{
    // A chair out on the floor whose blob the scan blurred a cell wider all round, and a counter
    // the wall's band cut 0.15 m short of its wall; both fitted to exactly what the blobs gave.
    const GridGeometry geometry{ 0.05, 0.0, 0.0, 80, 60 };
    cv::Mat            plan(geometry.height, geometry.width, CV_8UC1, cv::Scalar(kFree));
    cv::Mat            walls(plan.size(), CV_8UC1, cv::Scalar(0));
    plan.rowRange(50, 53).setTo(kOccupied);
    walls.rowRange(50, 53).setTo(255);
    plan(cv::Rect(19, 36, 34, 14)).setTo(kOccupied);  // The counter, y 1.80-2.50.
    plan(cv::Rect(59, 9, 12, 12)).setTo(kOccupied);   // The chair, x 2.95-3.55.
    std::map<int, Footprint> fitted{
        { 1, { { 1.8, 2.075 }, { 1.7, 0.55 }, 0.0 } },
        { 2, { { 3.25, 0.75 }, { 0.6, 0.6 }, 0.0 } },
    };
    settleSides(fitted, plan, walls, geometry);

    EXPECT_NEAR(fitted.at(1).size.y(), 0.65, 1e-6);  // In at the front, back to the wall.
    EXPECT_NEAR(fitted.at(1).centre.y(), 2.175, 1e-6);
    EXPECT_NEAR(fitted.at(2).size.x(), 0.5, 1e-6);
    EXPECT_NEAR(fitted.at(2).size.y(), 0.5, 1e-6);
}

TEST(MapFit, LeavesTheWallAWardrobeStandsAgainst)
{
    // The floor plan fuses a wardrobe to a thick stretch of wall that runs on 3 m past it.
    const GridGeometry geometry{ 0.05, 0.0, 0.0, 120, 80 };
    cv::Mat            furniture(geometry.height, geometry.width, CV_8UC1, cv::Scalar(0));
    furniture(cv::Rect(20, 10, 34, 22)) = 255;  // The wardrobe, 1.7 x 1.1 m.
    furniture(cv::Rect(20, 32, 94, 7))  = 255;  // The wall behind it, 4.7 m.
    MappedObject wardrobe;
    wardrobe.id         = 1;
    wardrobe.box_centre = { 1.9, 1.05 };  // Its front half only, as the camera saw it.
    wardrobe.box_size   = { 1.6, 0.5 };

    // The wall adds to its share: let the area through to see where the box stops.
    MapFitParams params;
    params.max_growth                     = 5.0;
    const std::map<int, Footprint> fitted = fitToMap({ wardrobe }, furniture, geometry, params);
    ASSERT_TRUE(fitted.contains(1));
    EXPECT_LT(fitted.at(1).size.x(), 1.6 + 2.0 + 0.06);  // At most a metre either side...
    EXPECT_GT(fitted.at(1).size.y(), 1.0);               // ...and back to the wall.
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
