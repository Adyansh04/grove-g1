/**
 * @file test_viewpoint_planner.cpp
 * @brief Coverage and frontier planning against a ray-cast camera, no ROS.
 */

#include <gmock/gmock.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>

#include "g1_world_model/coverage_map.hpp"
#include "g1_world_model/room_segmentation.hpp"
#include "g1_world_model/viewpoint_planner.hpp"

namespace g1_world_model
{
namespace
{

constexpr double kResolution = 0.05;
constexpr double kWallHeight = 2.5;

int cellsOf(double metres) { return static_cast<int>(std::lround(metres / kResolution)); }

/// A floor plan with a height per cell: 0 free, the obstacle's top otherwise.
struct World
{
    cv::Mat      heights;  // CV_32F
    cv::Mat      cells;    // CV_8U Cell
    GridGeometry geometry;

    void box(double x0, double y0, double x1, double y1, float height)
    {
        cv::rectangle(
            heights,
            cv::Point(cellsOf(x0), cellsOf(y0)),
            cv::Point(cellsOf(x1) - 1, cellsOf(y1) - 1),
            cv::Scalar(height),
            cv::FILLED);
    }

    void finish()
    {
        cells = cv::Mat(heights.size(), CV_8UC1);
        for (int y = 0; y < heights.rows; ++y)
        {
            for (int x = 0; x < heights.cols; ++x)
            {
                cells.at<std::uint8_t>(y, x) = heights.at<float>(y, x) > 0.0F ? kOccupied : kFree;
            }
        }
        geometry = { kResolution, 0.0, 0.0, heights.cols, heights.rows };
    }
};

/// Two 5 x 5 m rooms over a 1.6 m corridor, a table in one of them.
World twoRoomsAndACorridor()
{
    World world;
    world.heights = cv::Mat(cellsOf(7.0), cellsOf(10.2), CV_32F, cv::Scalar(kWallHeight));
    world.box(0.1, 0.1, 10.1, 1.7, 0.0F);
    world.box(0.1, 1.8, 5.0, 6.9, 0.0F);
    world.box(5.1, 1.8, 10.1, 6.9, 0.0F);
    world.box(1.9, 1.7, 3.1, 1.8, 0.0F);
    world.box(6.9, 1.7, 8.1, 1.8, 0.0F);
    world.box(2.0, 4.0, 3.2, 4.8, 0.75F);  // A table.
    world.finish();
    return world;
}

/// The simulated head camera: 848 x 480, 58 degrees vertical, pitched 47.6 degrees down.
struct Camera
{
    Intrinsics  intrinsics{ 433.0, 433.0, 424.0, 240.0, 848, 480 };
    CameraModel model;
    int         stride = 4;

    Camera()
    {
        model.height         = 1.21;
        model.pitch          = 0.8307767;
        model.vertical_fov   = 2.0 * std::atan(intrinsics.height / (2.0 * intrinsics.fy));
        model.horizontal_fov = 2.0 * std::atan(intrinsics.width / (2.0 * intrinsics.fx));
    }

    [[nodiscard]] Eigen::Isometry3d pose(double x, double y, double yaw) const
    {
        const Eigen::Vector3d forward(
            std::cos(model.pitch) * std::cos(yaw),
            std::cos(model.pitch) * std::sin(yaw),
            -std::sin(model.pitch));
        const Eigen::Vector3d right(std::sin(yaw), -std::cos(yaw), 0.0);
        const Eigen::Vector3d down = forward.cross(right);
        Eigen::Isometry3d     pose = Eigen::Isometry3d::Identity();
        pose.linear().col(0)       = right;
        pose.linear().col(1)       = down;
        pose.linear().col(2)       = forward;
        pose.translation()         = Eigen::Vector3d(x, y, model.height);
        return pose;
    }

    /// Marches each sampled pixel through the height field; unsampled pixels stay NaN.
    [[nodiscard]] std::vector<float> render(const World& world, const Eigen::Isometry3d& pose) const
    {
        std::vector<float> depth(
            static_cast<std::size_t>(intrinsics.width) * intrinsics.height,
            std::numeric_limits<float>::quiet_NaN());
        for (int v = stride / 2; v < intrinsics.height; v += stride)
        {
            for (int u = stride / 2; u < intrinsics.width; u += stride)
            {
                const Eigen::Vector3d in_camera(
                    (u - intrinsics.cx) / intrinsics.fx,
                    (v - intrinsics.cy) / intrinsics.fy,
                    1.0);
                const Eigen::Vector3d direction = pose.linear() * in_camera.normalized();
                for (int step = 5; step < 600; ++step)
                {
                    const double          t     = 0.01 * step;
                    const Eigen::Vector3d point = pose.translation() + (t * direction);
                    const CellIndex       cell  = world.geometry.toCell(point.x(), point.y());
                    if (!world.geometry.contains(cell))
                    {
                        break;
                    }
                    if (point.z() <= 0.0 || point.z() <= world.heights.at<float>(cell.y, cell.x))
                    {
                        depth[(static_cast<std::size_t>(v) * intrinsics.width) + u] =
                            static_cast<float>(t / in_camera.norm());
                        break;
                    }
                }
            }
        }
        return depth;
    }
};

/// Stands at @p viewpoint and integrates a frame per heading; the pose the robot ends in.
Pose2D
look(const World& world, const Camera& camera, CoverageMap& coverage, const Viewpoint& viewpoint)
{
    for (const double heading : viewpoint.headings)
    {
        const Eigen::Isometry3d  pose  = camera.pose(viewpoint.x, viewpoint.y, heading);
        const std::vector<float> depth = camera.render(world, pose);
        coverage.integrate(
            { depth.data(),
              camera.intrinsics.width,
              camera.intrinsics.height,
              static_cast<std::size_t>(camera.intrinsics.width) },
            camera.intrinsics,
            pose);
    }
    return { viewpoint.x, viewpoint.y, viewpoint.headings.back() };
}

struct Totals
{
    double floor_share;
    double face_share;
    int    unobservable;
};

Totals totals(const CoverageMap& coverage)
{
    int floor        = 0;
    int floor_seen   = 0;
    int face         = 0;
    int face_seen    = 0;
    int unobservable = 0;
    for (int index = 0; index < static_cast<int>(coverage.geometry().cellCount()); ++index)
    {
        if (coverage.unobservable(index))
        {
            ++unobservable;
            continue;
        }
        if (coverage.kind(index) == TargetKind::kFloor)
        {
            ++floor;
            floor_seen += static_cast<int>(coverage.wellSeen(index));
        }
        else if (coverage.kind(index) == TargetKind::kFace)
        {
            ++face;
            face_seen += static_cast<int>(coverage.wellSeen(index));
        }
    }
    return { static_cast<double>(floor_seen) / std::max(floor, 1),
             static_cast<double>(face_seen) / std::max(face, 1),
             unobservable };
}

TEST(CoverageMap, CreditsWhatTheCameraSees)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);

    // Standing in the right room facing +y, towards its far wall.
    const Eigen::Isometry3d  pose  = camera.pose(7.5, 3.0, std::numbers::pi / 2.0);
    const std::vector<float> depth = camera.render(world, pose);
    const DepthImage         image{ depth.data(),
                            camera.intrinsics.width,
                            camera.intrinsics.height,
                            static_cast<std::size_t>(camera.intrinsics.width) };
    EXPECT_GT(coverage.integrate(image, camera.intrinsics, pose), 500U);

    const auto at = [&](double x, double y) {
        return world.geometry.index(world.geometry.toCell(x, y));
    };
    EXPECT_TRUE(coverage.wellSeen(at(7.5, 4.5)));   // 1.5 m ahead: plainly in view.
    EXPECT_FALSE(coverage.wellSeen(at(7.5, 2.5)));  // Behind the camera.
    EXPECT_FALSE(coverage.wellSeen(at(7.5, 3.1)));  // Under the image: the pitch hides it.
    EXPECT_FALSE(coverage.wellSeen(at(5.5, 4.5)));  // Beyond the horizontal field of view.
}

TEST(CoverageMap, CreditsWallsDespiteLocalisationError)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);

    // Rendered where the robot is; integrated where AMCL believes it is, 0.15 m off.
    const Eigen::Isometry3d  truth    = camera.pose(7.5, 5.0, std::numbers::pi / 2.0);
    const std::vector<float> depth    = camera.render(world, truth);
    Eigen::Isometry3d        believed = truth;
    believed.translation() += Eigen::Vector3d(0.0, -0.15, 0.0);
    coverage.integrate(
        { depth.data(),
          camera.intrinsics.width,
          camera.intrinsics.height,
          static_cast<std::size_t>(camera.intrinsics.width) },
        camera.intrinsics,
        believed);

    // The far wall of the right room, 1.8 m ahead.
    int seen = 0;
    for (int step = 0; step <= 20; ++step)
    {
        const CellIndex cell = world.geometry.toCell(7.0 + (0.05 * step), 6.92);
        seen += static_cast<int>(coverage.wellSeen(world.geometry.index(cell)));
    }
    EXPECT_GE(seen, 15);
}

TEST(CoverageMap, CreditsTheNearSideOfAThinWall)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);

    // In the right room facing the 0.1 m wall to the left room; AMCL believes the robot is
    // 0.15 m closer to it, so every sample lands past the wall's middle.
    const Eigen::Isometry3d  truth    = camera.pose(6.6, 4.5, std::numbers::pi);
    const std::vector<float> depth    = camera.render(world, truth);
    Eigen::Isometry3d        believed = truth;
    believed.translation() += Eigen::Vector3d(-0.15, 0.0, 0.0);
    coverage.integrate(
        { depth.data(),
          camera.intrinsics.width,
          camera.intrinsics.height,
          static_cast<std::size_t>(camera.intrinsics.width) },
        camera.intrinsics,
        believed);

    // The right room's face of the wall is the column at x = 5.05..5.10.
    int near_side = 0;
    int far_side  = 0;
    for (int step = 0; step <= 20; ++step)
    {
        const double y = 4.0 + (0.05 * step);
        near_side += static_cast<int>(
            coverage.wellSeen(world.geometry.index(world.geometry.toCell(5.07, y))));
        far_side += static_cast<int>(
            coverage.wellSeen(world.geometry.index(world.geometry.toCell(5.02, y))));
    }
    EXPECT_GE(near_side, 15);
    EXPECT_EQ(far_side, 0);
}

TEST(CoverageMap, SharesLeaveOutWhatWasWrittenOffUnseen)
{
    const World world = twoRoomsAndACorridor();
    CoverageMap coverage;
    coverage.setMap(world.cells, world.geometry);
    const cv::Mat       labels(world.cells.size(), CV_32S, cv::Scalar(1));
    const CoverageTally before = coverage.tally(labels, 1)[1];
    coverage.markUnobservable(world.geometry.index(world.geometry.toCell(1.0, 0.9)));
    const CoverageTally after = coverage.tally(labels, 1)[1];
    EXPECT_EQ(after.floor, before.floor - 1);
    EXPECT_EQ(after.unobservable, before.unobservable + 1);
}

TEST(ViewpointPlanner, CoversTwoRoomsAndACorridor)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    ViewpointPlanner   planner({}, camera.model);

    Pose2D     robot{ 1.0, 0.9, 0.0 };
    int        viewpoints = 0;
    int        headings   = 0;
    double     longest_ms = 0.0;
    PlanStatus status     = PlanStatus::kViewpoint;
    while (viewpoints < 60)
    {
        const auto start = std::chrono::steady_clock::now();
        const Plan plan  = planner.nextCoverage(coverage, rooms.labels, robot);
        longest_ms       = std::max(
            longest_ms,
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count());
        status = plan.status;
        if (status != PlanStatus::kViewpoint)
        {
            break;
        }
        ++viewpoints;
        headings += static_cast<int>(plan.viewpoint.headings.size());
        robot = look(world, camera, coverage, plan.viewpoint);
        planner.report(coverage, plan.viewpoint.id, true);
    }

    ASSERT_EQ(status, PlanStatus::kDone);
    const Totals result = totals(coverage);
    RecordProperty("viewpoints", viewpoints);
    RecordProperty("headings", headings);
    RecordProperty("longest_plan_ms", static_cast<int>(longest_ms));
    std::printf(
        "viewpoints %d, headings %d, floor %.3f, faces %.3f, unobservable %d, slowest plan %.1f "
        "ms\n",
        viewpoints,
        headings,
        result.floor_share,
        result.face_share,
        result.unobservable,
        longest_ms);
    EXPECT_GE(result.floor_share, 0.95);
    EXPECT_GE(result.face_share, 0.90);
    EXPECT_LE(viewpoints, 30);
}

TEST(ViewpointPlanner, SendsTheRobotToTheFrontierThenStops)
{
    World world = twoRoomsAndACorridor();
    // Everything past the left room's wall is still unknown: only the corridor leads on.
    cv::Mat cells = world.cells.clone();
    cells(cv::Rect(cellsOf(5.1), 0, cells.cols - cellsOf(5.1), cells.rows)).setTo(kUnknown);

    ViewpointPlanner planner;
    const Plan       first = planner.nextFrontier(cells, world.geometry, { 1.0, 0.9, 0.0 });
    ASSERT_EQ(first.status, PlanStatus::kViewpoint);
    EXPECT_NEAR(first.viewpoint.x, 4.5, 0.6);
    EXPECT_NEAR(first.viewpoint.y, 0.9, 0.6);
    ASSERT_EQ(first.viewpoint.headings.size(), 1U);
    EXPECT_NEAR(first.viewpoint.headings[0], 0.0, 0.5);  // Facing the unknown.

    const Plan done = planner.nextFrontier(world.cells, world.geometry, { 4.5, 0.9, 0.0 });
    EXPECT_EQ(done.status, PlanStatus::kDone);
}

TEST(ViewpointPlanner, AvoidsAViewpointNavigationCouldNotReach)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    ViewpointPlanner planner({}, camera.model);

    const Plan first = planner.nextCoverage(coverage, {}, { 1.0, 0.9, 0.0 });
    ASSERT_EQ(first.status, PlanStatus::kViewpoint);
    planner.report(coverage, first.viewpoint.id, false);
    const Plan second = planner.nextCoverage(coverage, {}, { 1.0, 0.9, 0.0 });
    planner.report(coverage, second.viewpoint.id, false);
    const Plan third = planner.nextCoverage(coverage, {}, { 1.0, 0.9, 0.0 });
    ASSERT_EQ(third.status, PlanStatus::kViewpoint);
    // Failed once it costs three times as much; failed twice it is off the list.
    const auto moved = [&first](const Plan& plan) {
        return std::hypot(
                   plan.viewpoint.x - first.viewpoint.x,
                   plan.viewpoint.y - first.viewpoint.y) > 0.3;
    };
    EXPECT_TRUE(moved(second) || moved(third));
}

TEST(ViewpointPlanner, KeepsOutOfADoorwayNav2WouldNotPlanThrough)
{
    // The left room's door, 1.2 m, narrowed to 0.9 m: twice the robot radius.
    World narrow = twoRoomsAndACorridor();
    narrow.box(1.9, 1.7, 3.1, 1.8, kWallHeight);
    narrow.box(2.05, 1.7, 2.95, 1.8, 0.0F);
    narrow.finish();
    const World wide = twoRoomsAndACorridor();

    const auto reaches_the_room = [](const World& world) {
        ViewpointPlanner planner;
        planner.prepare(world.cells, world.geometry, { 1.0, 0.9, 0.0 });
        const CellIndex inside = world.geometry.toCell(1.0, 3.0);
        return std::isfinite(
            planner.travel()[static_cast<std::size_t>(world.geometry.index(inside.x, inside.y))]);
    };
    EXPECT_FALSE(reaches_the_room(narrow));
    EXPECT_TRUE(reaches_the_room(wide));
}

TEST(ViewpointPlanner, GivesUpOnARoomNavigationCannotEnter)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    ViewpointPlanner   planner({}, camera.model);

    // Nav2 refuses every goal in the right-hand room, as it would behind a shut door.
    const auto shut = [](const Viewpoint& viewpoint) {
        return viewpoint.x > 5.1 && viewpoint.y > 1.8;
    };
    Pose2D robot{ 1.0, 0.9, 0.0 };
    Plan   plan;
    int    refused = 0;
    for (int step = 0; step < 60; ++step)
    {
        plan = planner.nextCoverage(coverage, rooms.labels, robot);
        if (plan.status != PlanStatus::kViewpoint)
        {
            break;
        }
        const bool reached = !shut(plan.viewpoint);
        if (reached)
        {
            robot = look(world, camera, coverage, plan.viewpoint);
        }
        refused += static_cast<int>(!reached);
        planner.report(coverage, plan.viewpoint.id, reached);
    }

    EXPECT_EQ(plan.status, PlanStatus::kDone);
    EXPECT_EQ(refused, planner.params().max_room_failures);
}

TEST(ViewpointPlanner, KeepsARoomItHasBeenInsideDespiteFailures)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    ViewpointPlanner   planner({}, camera.model);

    // Once inside the right-hand room, its next three viewpoints fail: furniture, not the door.
    const auto in_right = [](const Viewpoint& viewpoint) {
        return viewpoint.x > 5.1 && viewpoint.y > 1.8;
    };
    Pose2D robot{ 1.0, 0.9, 0.0 };
    Plan   plan;
    bool   entered   = false;
    int    to_refuse = 3;
    for (int step = 0; step < 80; ++step)
    {
        plan = planner.nextCoverage(coverage, rooms.labels, robot);
        if (plan.status != PlanStatus::kViewpoint)
        {
            break;
        }
        if (entered && in_right(plan.viewpoint) && to_refuse > 0)
        {
            --to_refuse;
            planner.report(coverage, plan.viewpoint.id, false);
            continue;
        }
        entered = entered || in_right(plan.viewpoint);
        robot   = look(world, camera, coverage, plan.viewpoint);
        planner.report(coverage, plan.viewpoint.id, true);
    }

    ASSERT_EQ(plan.status, PlanStatus::kDone);
    EXPECT_EQ(to_refuse, 0);
    EXPECT_GE(totals(coverage).floor_share, 0.95);
}

TEST(ViewpointPlanner, WaitsOutAStuckRobotInsteadOfGivingUpEveryRoom)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    ViewpointPlanner   planner({}, camera.model);

    // Wedged in the corridor: Nav2 refuses every goal, wherever it is.
    const Pose2D wedged{ 4.0, 0.9, 0.0 };
    Plan         plan;
    int          refused = 0;
    for (; refused < 20; ++refused)
    {
        plan = planner.nextCoverage(coverage, rooms.labels, wedged);
        if (plan.status != PlanStatus::kViewpoint)
        {
            break;
        }
        planner.report(coverage, plan.viewpoint.id, false);
    }
    EXPECT_EQ(plan.status, PlanStatus::kUnavailable);
    EXPECT_EQ(refused, planner.params().max_failures_in_a_row);
    EXPECT_EQ(totals(coverage).unobservable, 0);

    // Freed and carried a metre on: planning resumes, and no room was given up on the way.
    plan = planner.nextCoverage(coverage, rooms.labels, { 5.0, 0.9, 0.0 });
    EXPECT_EQ(plan.status, PlanStatus::kViewpoint);
}

TEST(ViewpointPlanner, WritesOffWhatNoPoseCanSeeWhenCandidatesAreCapped)
{
    // A wardrobe 0.15 m off the right room's back wall: the floor behind it is out of sight.
    World world = twoRoomsAndACorridor();
    world.box(7.0, 6.2, 9.0, 6.75, kWallHeight);
    world.finish();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    PlannerParams      params;
    params.max_candidates = 40;  // Far fewer than the rooms hold, so the ranked list is cut.
    ViewpointPlanner planner(params, camera.model);

    Pose2D robot{ 1.0, 0.9, 0.0 };
    Plan   plan;
    for (int step = 0; step < 80; ++step)
    {
        plan = planner.nextCoverage(coverage, rooms.labels, robot);
        if (plan.status != PlanStatus::kViewpoint)
        {
            break;
        }
        robot = look(world, camera, coverage, plan.viewpoint);
        planner.report(coverage, plan.viewpoint.id, true);
    }

    ASSERT_EQ(plan.status, PlanStatus::kDone);
    const CellIndex behind = world.geometry.toCell(8.0, 6.82);
    EXPECT_TRUE(coverage.unobservable(world.geometry.index(behind)));
}

}  // namespace
}  // namespace g1_world_model
