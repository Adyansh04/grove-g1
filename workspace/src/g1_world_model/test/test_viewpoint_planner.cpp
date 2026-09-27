/**
 * @file test_viewpoint_planner.cpp
 * @brief Coverage and frontier planning against a ray-cast camera, no ROS.
 */

#include <gmock/gmock.h>

#include <algorithm>
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

TEST(CoverageMap, ForgetsATopThatTurnsOutHigherThanItWasSeenAt)
{
    // The table first mapped from its front face alone, 0.2 m short of its real top.
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    Surface table;
    table.id        = 1;
    table.height    = 0.55;
    table.footprint = { { 2.0, 4.0 }, { 3.2, 4.0 }, { 3.2, 4.8 }, { 2.0, 4.8 } };
    coverage.setSurfaces({ table });

    const Eigen::Isometry3d  pose  = camera.pose(2.6, 2.9, std::numbers::pi / 2.0);
    const std::vector<float> depth = camera.render(world, pose);
    coverage.integrate(
        { depth.data(),
          camera.intrinsics.width,
          camera.intrinsics.height,
          static_cast<std::size_t>(camera.intrinsics.width) },
        camera.intrinsics,
        pose);
    const int front = world.geometry.index(world.geometry.toCell(2.6, 4.02));
    ASSERT_TRUE(coverage.surfaceWellSeen(front));  // Its front face, at the height believed.

    table.height = 0.75;
    coverage.setSurfaces({ table });
    EXPECT_FALSE(coverage.surfaceWellSeen(front));
}

TEST(CoverageMap, TellsWhatThePlanWillSeeFromWhatNoPoseCan)
{
    const World world = twoRoomsAndACorridor();
    CoverageMap coverage;
    coverage.setMap(world.cells, world.geometry);
    const std::size_t count = world.geometry.cellCount();
    const auto        planned =
        static_cast<std::size_t>(world.geometry.index(world.geometry.toCell(7.5, 4.5)));
    const auto blind =
        static_cast<std::size_t>(world.geometry.index(world.geometry.toCell(2.5, 3.0)));
    EXPECT_EQ(coverage.statusGrid()[blind], 90);  // Before any plan, all still to see.

    std::vector<std::uint8_t> predicted(2 * count, 0);
    predicted[planned] = 1;
    coverage.setPlanned(predicted);
    const std::vector<std::int8_t> grid = coverage.statusGrid();
    EXPECT_EQ(grid[planned], 90);
    EXPECT_EQ(grid[blind], 99);
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

TEST(CoverageMap, KeepsAWallsCreditWhenSlamRedrawsItACellBack)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Eigen::Isometry3d  pose  = camera.pose(7.5, 5.0, std::numbers::pi / 2.0);
    const std::vector<float> depth = camera.render(world, pose);
    coverage.integrate(
        { depth.data(),
          camera.intrinsics.width,
          camera.intrinsics.height,
          static_cast<std::size_t>(camera.intrinsics.width) },
        camera.intrinsics,
        pose);
    const auto seen_along = [&](double y) {
        int seen = 0;
        for (int step = 0; step <= 20; ++step)
        {
            const CellIndex cell = world.geometry.toCell(7.0 + (0.05 * step), y);
            seen += static_cast<int>(coverage.wellSeen(world.geometry.index(cell)));
        }
        return seen;
    };
    ASSERT_GE(seen_along(6.92), 15);

    // A loop closure redraws the right room's far wall a cell further back, and puts a box in the
    // room behind the camera: the wall keeps its credit, and the box, never seen, earns none.
    cv::Mat         redrawn = world.cells.clone();
    const CellIndex face    = world.geometry.toCell(5.1, 6.92);
    redrawn.row(face.y).colRange(face.x, world.geometry.toCell(10.1, 6.92).x).setTo(kFree);
    const CellIndex box = world.geometry.toCell(8.5, 3.5);
    redrawn(cv::Rect(box.x, box.y, 6, 6)).setTo(kOccupied);
    coverage.setMap(redrawn, world.geometry);
    EXPECT_GE(seen_along(6.97), 15);
    int box_seen = 0;
    for (int y = box.y; y < box.y + 6; ++y)
    {
        for (int x = box.x; x < box.x + 6; ++x)
        {
            box_seen += static_cast<int>(coverage.wellSeen(world.geometry.index(x, y)));
        }
    }
    EXPECT_EQ(box_seen, 0);
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

TEST(CoverageMap, KeepsWhatWasSeenWhenTheMapGrows)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Eigen::Isometry3d  pose  = camera.pose(2.5, 3.5, std::numbers::pi / 2.0);
    const std::vector<float> depth = camera.render(world, pose);
    coverage.integrate(
        { depth.data(),
          camera.intrinsics.width,
          camera.intrinsics.height,
          static_cast<std::size_t>(camera.intrinsics.width) },
        camera.intrinsics,
        pose);
    const Totals before = totals(coverage);
    int          seen   = -1;
    for (int index = 0; index < static_cast<int>(world.geometry.cellCount()) && seen < 0; ++index)
    {
        if (coverage.kind(index) == TargetKind::kFloor && coverage.wellSeen(index))
        {
            seen = index;
        }
    }
    ASSERT_GE(seen, 0);
    const double seen_x = world.geometry.centreX(seen % world.geometry.width);
    const double seen_y = world.geometry.centreY(seen / world.geometry.width);

    // SLAM adds a metre of unknown on every side: the same cells, 20 further in.
    const int margin = 20;
    cv::Mat   grown(
        world.cells.rows + (2 * margin),
        world.cells.cols + (2 * margin),
        CV_8UC1,
        cv::Scalar(kUnknown));
    world.cells.copyTo(grown(cv::Rect(margin, margin, world.cells.cols, world.cells.rows)));
    const GridGeometry geometry{ kResolution, -1.0, -1.0, grown.cols, grown.rows };
    coverage.setMap(grown, geometry);

    EXPECT_TRUE(coverage.wellSeen(geometry.index(geometry.toCell(seen_x, seen_y))));
    EXPECT_NEAR(totals(coverage).floor_share, before.floor_share, 1e-9);
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

TEST(ViewpointPlanner, ReachesAFrontierBeforeTheMapHasWalls)
{
    // SLAM's first map: 3 x 2 m of free floor around the robot, unknown all round, no walls yet.
    const GridGeometry geometry{ kResolution, 0.0, 0.0, cellsOf(10.0), cellsOf(10.0) };
    cv::Mat            cells(geometry.height, geometry.width, CV_8UC1, cv::Scalar(kUnknown));
    cells(cv::Rect(cellsOf(3.5), cellsOf(4.0), cellsOf(3.0), cellsOf(2.0))) = kFree;

    ViewpointPlanner planner;
    const Plan       plan = planner.nextFrontier(cells, geometry, { 5.0, 5.0, 0.0 });
    ASSERT_EQ(plan.status, PlanStatus::kViewpoint) << plan.reason;
    const CellIndex goal = geometry.toCell(plan.viewpoint.x, plan.viewpoint.y);
    EXPECT_EQ(cells.at<std::uint8_t>(goal.y, goal.x), kFree);
}

TEST(ViewpointPlanner, LooksPastTheGapsBetweenBeamsInAYoungMap)
{
    // SLAM after one sweep: known all round the robot to 1.5 m, then only along beams 5 deg apart
    // out to 4.5 m, a wall at 4.6 m ahead; unknown between the beams.
    const GridGeometry geometry{ kResolution, 0.0, 0.0, cellsOf(12.0), cellsOf(12.0) };
    cv::Mat            cells(geometry.height, geometry.width, CV_8UC1, cv::Scalar(kUnknown));
    const cv::Point    robot(cellsOf(6.0), cellsOf(6.0));
    cv::circle(cells, robot, cellsOf(1.5), cv::Scalar(kFree), cv::FILLED);
    for (int degrees = 0; degrees < 360; degrees += 5)
    {
        const double angle = degrees * std::numbers::pi / 180.0;
        cv::line(
            cells,
            robot,
            robot + cv::Point(
                        static_cast<int>(std::lround(cellsOf(4.5) * std::cos(angle))),
                        static_cast<int>(std::lround(cellsOf(4.5) * std::sin(angle)))),
            cv::Scalar(kFree));
    }
    cv::line(
        cells,
        { cellsOf(10.6), cellsOf(4.0) },
        { cellsOf(10.6), cellsOf(8.0) },
        cv::Scalar(kOccupied),
        2);

    ViewpointPlanner planner;
    const Plan       plan = planner.nextFrontier(cells, geometry, { 6.0, 6.0, 0.0 });
    ASSERT_EQ(plan.status, PlanStatus::kViewpoint) << plan.reason;
    // Out towards the edge of what is known, not a step from where the robot already stands.
    EXPECT_GT(std::hypot(plan.viewpoint.x - 6.0, plan.viewpoint.y - 6.0), 2.0);
}

TEST(ViewpointPlanner, LeavesThePocketBehindASofaToTheCamera)
{
    // Everything mapped but a 1 x 1 m shadow behind a sofa: a frontier, and nothing beyond it.
    World world = twoRoomsAndACorridor();
    world.box(2.0, 5.2, 4.0, 5.8, 0.8F);  // The sofa, its back 1 m off the wall.
    world.finish();
    cv::Mat cells                                                           = world.cells.clone();
    cells(cv::Rect(cellsOf(2.5), cellsOf(5.8), cellsOf(1.0), cellsOf(1.0))) = kUnknown;

    ViewpointPlanner planner;
    const Plan       plan = planner.nextFrontier(cells, world.geometry, { 1.0, 0.9, 0.0 });
    EXPECT_EQ(plan.status, PlanStatus::kDone) << plan.reason;
}

TEST(ViewpointPlanner, LeavesAShadowItHasLookedIntoFromNearby)
{
    // An armchair and a tall cabinet box in a corner the LiDAR cannot see into.
    World world;
    world.heights = cv::Mat(cellsOf(5.7), cellsOf(6.2), CV_32F, cv::Scalar(kWallHeight));
    world.box(0.1, 0.1, 6.1, 5.6, 0.0F);
    world.box(2.0, 3.0, 4.0, 3.6, 0.8F);  // The armchair.
    world.box(4.0, 3.0, 4.6, 5.6, 2.0F);  // The cabinet.
    world.finish();
    cv::Mat cells                                                           = world.cells.clone();
    cells(cv::Rect(cellsOf(2.0), cellsOf(3.6), cellsOf(2.0), cellsOf(2.0))) = kUnknown;
    CoverageMap coverage;
    coverage.setMap(cells, world.geometry);
    PlannerParams params;
    params.min_frontier_unknown = 1.0;  // As when the shadow joins the unknown outside the walls.
    const Pose2D robot{ 1.0, 1.0, 0.0 };

    ViewpointPlanner planner(params);
    const Plan       first = planner.nextFrontier(cells, world.geometry, robot);
    ASSERT_EQ(first.status, PlanStatus::kViewpoint);
    planner.report(coverage, first.viewpoint.id, true);
    // Until SLAM has folded in what the robot saw from there, the frontier still counts.
    EXPECT_EQ(planner.nextFrontier(cells, world.geometry, robot).status, PlanStatus::kViewpoint);
    planner.mapUpdated();
    const Plan after = planner.nextFrontier(cells, world.geometry, robot);
    EXPECT_EQ(after.status, PlanStatus::kDone) << after.reason;

    // Near, but only across the armchair: still worth the walk.
    ViewpointPlanner across(params);
    across.recordPose({ 2.5, 2.7, 0.0 });
    across.mapUpdated();
    EXPECT_EQ(across.nextFrontier(cells, world.geometry, robot).status, PlanStatus::kViewpoint);
}

TEST(ViewpointPlanner, EndsTheFrontierPassWhenVisitsStopGrowingTheMap)
{
    // One room with eight unknown pockets, each a frontier worth a walk.
    World world;
    world.heights = cv::Mat(cellsOf(12.0), cellsOf(12.0), CV_32F, cv::Scalar(kWallHeight));
    world.box(0.1, 0.1, 11.9, 11.9, 0.0F);
    world.finish();
    std::vector<cv::Point2d> pockets;
    for (const double x : { 0.8, 3.8, 6.8, 9.4 })
    {
        for (const double y : { 1.0, 5.0 })
        {
            pockets.emplace_back(x, y);
        }
    }
    const auto pocket = [](const cv::Point2d& corner) {
        return cv::Rect(cellsOf(corner.x), cellsOf(corner.y), cellsOf(2.2), cellsOf(2.2));
    };
    cv::Mat cells = world.cells.clone();
    for (const cv::Point2d& corner : pockets)
    {
        cells(pocket(corner)) = kUnknown;
    }
    CoverageMap coverage;
    coverage.setMap(cells, world.geometry);
    const Pose2D robot{ 6.0, 10.5, 0.0 };

    PlannerParams params;
    params.frontier_stall_visits = 5;
    params.min_frontier_growth   = 2.0;

    // Five visits that reveal nothing, and the pass is over.
    ViewpointPlanner stalled(params);
    for (int visit = 0; visit < 5; ++visit)
    {
        const Plan plan = stalled.nextFrontier(cells, world.geometry, robot);
        ASSERT_EQ(plan.status, PlanStatus::kViewpoint) << visit;
        stalled.report(coverage, plan.viewpoint.id, true);
    }
    const Plan last = stalled.nextFrontier(cells, world.geometry, robot);
    EXPECT_EQ(last.status, PlanStatus::kDone) << last.reason;

    // Each visit revealing the pocket it went to keeps it going.
    ViewpointPlanner growing(params);
    cv::Mat          revealed = cells.clone();
    for (int visit = 0; visit < 5; ++visit)
    {
        const Plan plan = growing.nextFrontier(revealed, world.geometry, robot);
        ASSERT_EQ(plan.status, PlanStatus::kViewpoint) << visit;
        growing.report(coverage, plan.viewpoint.id, true);
        const auto nearest = std::ranges::min_element(pockets, {}, [&](const cv::Point2d& c) {
            return std::hypot(c.x + 1.1 - plan.viewpoint.x, c.y + 1.1 - plan.viewpoint.y);
        });
        revealed(pocket(*nearest)) = kFree;
    }
    EXPECT_EQ(growing.nextFrontier(revealed, world.geometry, robot).status, PlanStatus::kViewpoint);

    // Walks navigation could not finish are not visits: four of them and one that revealed
    // nothing leave the pass going.
    ViewpointPlanner refused(params);
    for (int visit = 0; visit < 5; ++visit)
    {
        const Plan plan = refused.nextFrontier(cells, world.geometry, robot);
        ASSERT_EQ(plan.status, PlanStatus::kViewpoint) << visit;
        refused.report(coverage, plan.viewpoint.id, visit == 4);
    }
    EXPECT_EQ(refused.nextFrontier(cells, world.geometry, robot).status, PlanStatus::kViewpoint);
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

TEST(ViewpointPlanner, LooksOnceIntoTheCornerBehindAWardrobe)
{
    // A 5 x 4 m room. A wardrobe stands out from the right wall, and the strip between it and the
    // top wall was never scanned. A block of furniture has an unknown inside, which is no pocket.
    cv::Mat cells(80, 100, CV_8UC1, cv::Scalar(kFree));
    cv::rectangle(cells, cv::Rect(0, 0, 100, 80), cv::Scalar(kOccupied), 2);
    cells(cv::Rect(80, 30, 18, 30)).setTo(kOccupied);
    cells(cv::Rect(80, 60, 18, 18)).setTo(kUnknown);
    cells(cv::Rect(20, 20, 12, 12)).setTo(kOccupied);
    cells(cv::Rect(23, 23, 6, 6)).setTo(kUnknown);
    const GridGeometry geometry{ kResolution, 0.0, 0.0, 100, 80 };
    const cv::Mat      inside(cells.size(), CV_8UC1, cv::Scalar(255));
    ViewpointPlanner   planner;

    const Plan look = planner.nextPocketLook(cells, inside, geometry, { 1.5, 2.5, 0.0 });
    ASSERT_EQ(look.status, PlanStatus::kViewpoint);
    EXPECT_LT(look.viewpoint.x, 4.0);  // On the room's side of the strip, looking in.
    ASSERT_EQ(look.viewpoint.headings.size(), 1U);
    const double to_strip = std::atan2(3.45 - look.viewpoint.y, 4.45 - look.viewpoint.x);
    EXPECT_LT(
        std::abs(std::remainder(look.viewpoint.headings[0] - to_strip, 2.0 * std::numbers::pi)),
        0.3);

    // Unknown found after the plan, as a look uncovers a little more, gets no look of its own.
    cells(cv::Rect(40, 60, 10, 10)).setTo(kUnknown);
    EXPECT_EQ(
        planner.nextPocketLook(cells, inside, geometry, { 1.5, 2.5, 0.0 }).status,
        PlanStatus::kDone);
}

TEST(ViewpointPlanner, DropsAPocketLookOnceItsEdgeIsMapped)
{
    // A 10 m room with a corner never scanned at each end, too far apart for one look: behind a
    // wardrobe at the top right, and at the bottom left.
    cv::Mat cells(80, 200, CV_8UC1, cv::Scalar(kFree));
    cv::rectangle(cells, cv::Rect(0, 0, 200, 80), cv::Scalar(kOccupied), 2);
    cells(cv::Rect(180, 30, 18, 30)).setTo(kOccupied);
    cells(cv::Rect(180, 60, 18, 18)).setTo(kUnknown);
    cells(cv::Rect(2, 2, 16, 16)).setTo(kUnknown);
    const GridGeometry geometry{ kResolution, 0.0, 0.0, 200, 80 };
    const cv::Mat      inside(cells.size(), CV_8UC1, cv::Scalar(255));
    const Pose2D       robot{ 6.5, 2.5, 0.0 };

    ViewpointPlanner both;
    const Plan       first = both.nextPocketLook(cells, inside, geometry, robot);
    ASSERT_EQ(first.status, PlanStatus::kViewpoint);
    EXPECT_GT(first.viewpoint.x, 5.0);  // The nearer corner first.
    const Plan second = both.nextPocketLook(cells, inside, geometry, robot);
    ASSERT_EQ(second.status, PlanStatus::kViewpoint);
    EXPECT_LT(second.viewpoint.x, 5.0);

    // Mapped by the time its turn comes, the bottom left corner is not walked to.
    ViewpointPlanner mapped;
    ASSERT_EQ(mapped.nextPocketLook(cells, inside, geometry, robot).status, PlanStatus::kViewpoint);
    cells(cv::Rect(2, 2, 16, 16)).setTo(kFree);
    EXPECT_EQ(mapped.nextPocketLook(cells, inside, geometry, robot).status, PlanStatus::kDone);
}

TEST(ViewpointPlanner, ScoresAHeadingByWhatEveryCameraSees)
{
    // A second camera looking back sees what the front one turns its back on, from the same stop.
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    const Pose2D       robot{ 2.5, 3.0, 0.0 };
    CameraModel        back = camera.model;
    back.yaw                = std::numbers::pi;
    PlannerParams params;
    params.max_headings = 1;

    ViewpointPlanner front(params, camera.model);
    ViewpointPlanner both(params, std::vector<CameraModel>{ camera.model, back });
    const Plan       one = front.nextCoverage(coverage, rooms.labels, robot);
    const Plan       two = both.nextCoverage(coverage, rooms.labels, robot);
    ASSERT_EQ(one.status, PlanStatus::kViewpoint);
    ASSERT_EQ(two.status, PlanStatus::kViewpoint);
    EXPECT_GT(two.viewpoint.gain, 1.5 * one.viewpoint.gain);

    ViewpointPlanner blind(params, std::vector<CameraModel>{});
    EXPECT_EQ(blind.nextCoverage(coverage, rooms.labels, robot).status, PlanStatus::kUnavailable);
}

TEST(ViewpointPlanner, EndsOnTheRateOnlyOnceEveryRoomHasItsShare)
{
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    PlannerParams      impatient;
    impatient.min_viewpoint_rate = 1e9;  // No viewpoint ever pays for itself.

    // Nothing seen yet: every room is short, so the pass goes on regardless.
    ViewpointPlanner early(impatient, camera.model);
    ASSERT_EQ(
        early.nextCoverage(coverage, rooms.labels, { 1.0, 0.9, 0.0 }).status,
        PlanStatus::kViewpoint);

    ViewpointPlanner planner({}, camera.model);
    Pose2D           robot{ 1.0, 0.9, 0.0 };
    for (int visit = 0; visit < 60; ++visit)
    {
        const Plan plan = planner.nextCoverage(coverage, rooms.labels, robot);
        if (plan.status != PlanStatus::kViewpoint)
        {
            break;
        }
        robot = look(world, camera, coverage, plan.viewpoint);
        planner.report(coverage, plan.viewpoint.id, true);
    }
    const PlannerParams              goals;
    const std::vector<CoverageTally> tallies =
        coverage.tally(rooms.labels, static_cast<int>(rooms.regions.size()));
    for (std::size_t room = 1; room < tallies.size(); ++room)
    {
        EXPECT_GE(tallies[room].floor_seen, goals.room_floor_goal * tallies[room].floor) << room;
        EXPECT_GE(tallies[room].face_seen, goals.room_face_goal * tallies[room].face) << room;
    }
    // Every room has its share: the rate ends it.
    ViewpointPlanner late(impatient, camera.model);
    EXPECT_EQ(late.nextCoverage(coverage, rooms.labels, robot).status, PlanStatus::kDone);
}

TEST(ViewpointPlanner, StaysDoneOnceTheCameraPassEnds)
{
    // Everything seen but a patch of floor in the right room: too little for the walk from the
    // corridor's far end, worth a stop from beside it.
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const std::size_t         count = world.geometry.cellCount();
    std::vector<std::uint8_t> quality(count, 255);
    const CellIndex           patch = world.geometry.toCell(8.3, 4.3);
    for (int y = patch.y; y < patch.y + 8; ++y)
    {
        for (int x = patch.x; x < patch.x + 8; ++x)
        {
            quality[static_cast<std::size_t>(world.geometry.index(x, y))] = 0;
        }
    }
    const std::vector<std::uint8_t> none(count, 0);
    ASSERT_TRUE(coverage.restoreLayers(quality, none, none, none));
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    PlannerParams      params;
    params.min_viewpoint_rate = 3.0;
    const Pose2D far{ 1.0, 0.9, 0.0 };
    const Pose2D beside{ 8.0, 3.2, 0.0 };

    ViewpointPlanner fresh(params, camera.model);
    ASSERT_EQ(fresh.nextCoverage(coverage, rooms.labels, beside).status, PlanStatus::kViewpoint);

    ViewpointPlanner planner(params, camera.model);
    ASSERT_EQ(planner.nextCoverage(coverage, rooms.labels, far).status, PlanStatus::kDone);
    EXPECT_EQ(planner.nextCoverage(coverage, rooms.labels, beside).status, PlanStatus::kDone);
}

TEST(ViewpointPlanner, FinishesTheRoomItStandsInBeforeLeaving)
{
    // A glance round the left room leaves it short of its share; the right one, unseen, pays
    // far more. Leaving now means walking back for the rest later.
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    Viewpoint          glance;
    glance.x              = 2.5;
    glance.y              = 3.0;
    glance.headings       = { 0.0, std::numbers::pi / 2.0 };
    const Pose2D    robot = look(world, camera, coverage, glance);
    const CellIndex here  = world.geometry.toCell(robot.x, robot.y);
    const int       room  = rooms.labels.at<int>(here.y, here.x);
    PlannerParams   params;
    params.room_switch_factor = 1.0;  // Nothing but the rule itself holds the robot here.
    ViewpointPlanner planner(params, camera.model);

    const Plan plan = planner.nextCoverage(coverage, rooms.labels, robot);
    ASSERT_EQ(plan.status, PlanStatus::kViewpoint);
    EXPECT_EQ(plan.viewpoint.room, room);
}

TEST(ViewpointPlanner, FinishesARoomPastItsShareWhileAViewThereStillPays)
{
    // No room is short, yet the left one holds views worth the rate: a tour of the two that see
    // the most, both in the unseen right room, would leave them for a walk back.
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    Viewpoint          glance;
    glance.x              = 2.5;
    glance.y              = 3.0;
    glance.headings       = { 0.0, std::numbers::pi / 2.0 };
    const Pose2D    robot = look(world, camera, coverage, glance);
    const CellIndex here  = world.geometry.toCell(robot.x, robot.y);
    PlannerParams   params;
    params.room_switch_factor = 1.0;
    params.room_floor_goal    = 0.0;
    params.room_face_goal     = 0.0;
    params.tour_size          = 2;
    ViewpointPlanner planner(params, camera.model);

    const Plan plan = planner.nextCoverage(coverage, rooms.labels, robot);
    ASSERT_EQ(plan.status, PlanStatus::kViewpoint);
    EXPECT_EQ(plan.viewpoint.room, rooms.labels.at<int>(here.y, here.x));
    EXPECT_GE(plan.viewpoint.gain / plan.viewpoint.cost, params.min_viewpoint_rate);
}

TEST(ViewpointPlanner, StaysInItsRoomWhenCandidatesAreCapped)
{
    // Most of the left room seen already, the right one not at all. Cut by work alone, the kept
    // poses are all in the right room and the robot leaves this one half done, to come back on
    // another lap.
    const World  world = twoRoomsAndACorridor();
    const Camera camera;
    CoverageMap  coverage;
    coverage.setMap(world.cells, world.geometry);
    const Segmentation rooms = segmentRooms(world.cells, world.geometry, {});
    Viewpoint          glance;
    glance.x        = 2.5;
    glance.y        = 3.0;
    glance.headings = { 0.0, std::numbers::pi / 2.0, std::numbers::pi, -std::numbers::pi / 2.0 };
    const Pose2D  robot = look(world, camera, coverage, glance);
    PlannerParams params;
    params.max_candidates = 20;
    ViewpointPlanner planner(params, camera.model);

    const Plan plan = planner.nextCoverage(coverage, rooms.labels, robot);
    ASSERT_EQ(plan.status, PlanStatus::kViewpoint);
    const CellIndex here = world.geometry.toCell(robot.x, robot.y);
    EXPECT_EQ(plan.viewpoint.room, rooms.labels.at<int>(here.y, here.x));
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
