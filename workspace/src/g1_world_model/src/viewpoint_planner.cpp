/**
 * @file viewpoint_planner.cpp
 * @brief Frontier and coverage viewpoints: travel field, ray prediction, heading selection.
 */

#include "g1_world_model/viewpoint_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <numbers>
#include <opencv2/imgproc.hpp>
#include <queue>
#include <utility>

namespace g1_world_model
{

namespace
{

constexpr float  kUnreachable = std::numeric_limits<float>::infinity();
constexpr double kTwoPi       = 2.0 * std::numbers::pi;

double wrapAngle(double angle) { return std::remainder(angle, kTwoPi); }

/// Clears each frontier cell with an unknown neighbour some trail point had in plain view, across
/// free cells only, within @p radius.
void dropSeenFrontier(
    cv::Mat& frontier, const cv::Mat& known, const GridGeometry& geometry,
    const std::vector<cv::Point2d>& trail, double radius)
{
    std::vector<cv::Point> stood;
    for (const cv::Point2d& point : trail)
    {
        const CellIndex cell = geometry.toCell(point.x, point.y);
        if (geometry.contains(cell) && known.at<std::uint8_t>(cell.y, cell.x) == kFree)
        {
            stood.emplace_back(cell.x, cell.y);
        }
    }
    const double reach = radius / geometry.resolution;
    // Free all the way to the unknown cell itself: the frontier cell being visible is not
    // enough, the unknown beside it may be round a door frame.
    const auto in_view = [&](const cv::Point& from, const cv::Point& to) {
        if (std::hypot(to.x - from.x, to.y - from.y) > reach)
        {
            return false;
        }
        cv::LineIterator line(known, from, to, 8);
        for (int i = 0; i + 1 < line.count; ++i, ++line)
        {
            if (**line != kFree)
            {
                return false;
            }
        }
        return true;
    };
    for (int y = 1; y + 1 < frontier.rows; ++y)
    {
        auto* row = frontier.ptr<std::uint8_t>(y);
        for (int x = 1; x + 1 < frontier.cols; ++x)
        {
            if (row[x] == 0)
            {
                continue;
            }
            for (const cv::Point& from : stood)
            {
                if (std::abs(x - from.x) > reach + 1 || std::abs(y - from.y) > reach + 1)
                {
                    continue;
                }
                const bool seen = std::ranges::any_of(
                    std::array{ cv::Point(x + 1, y),
                                cv::Point(x - 1, y),
                                cv::Point(x, y + 1),
                                cv::Point(x, y - 1) },
                    [&](const cv::Point& next) {
                        return known.at<std::uint8_t>(next) == kUnknown && in_view(from, next);
                    });
                if (seen)
                {
                    row[x] = 0;
                    break;
                }
            }
        }
    }
}

}  // namespace

ViewpointPlanner::ViewpointPlanner(PlannerParams params, CameraModel camera)
  : ViewpointPlanner(params, std::vector<CameraModel>{ camera })
{}

ViewpointPlanner::ViewpointPlanner(PlannerParams params, std::vector<CameraModel> cameras)
  : params_(params)
  , cameras_(std::move(cameras))
{}

void ViewpointPlanner::computeTraversable(const cv::Mat& cells, const GridGeometry& geometry)
{
    // Clearance from what is known to be solid. Unknown space is not a wall: counted as one, no
    // pose near a frontier would ever be standable while SLAM is still growing the map.
    const cv::Mat not_occupied = cells != kOccupied;
    cv::distanceTransform(not_occupied, clearance_, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    clearance_ *= geometry.resolution;
    // Nav2 inflates every cell within robot_radius to lethal, centre to centre; the margin keeps
    // a doorway exactly twice that wide from being a path here and a wall there.
    traversable_ = (clearance_ >= params_.robot_radius + params_.travel_margin) & (cells == kFree);
}

void ViewpointPlanner::computeTravel(const GridGeometry& geometry, const Pose2D& robot)
{
    travelFrom(geometry, robot.x, robot.y, travel_);
}

void ViewpointPlanner::travelFrom(
    const GridGeometry& geometry, double x0, double y0, std::vector<float>& field) const
{
    field.assign(geometry.cellCount(), kUnreachable);
    const CellIndex start = geometry.toCell(x0, y0);
    if (!geometry.contains(start))
    {
        return;
    }

    using Entry = std::pair<float, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> queue;
    const auto passable = [this](int x, int y) { return traversable_.at<std::uint8_t>(y, x) != 0; };

    // Standing a little too close to furniture still has to count as somewhere: seed from the
    // passable cells within a metre, charged their distance.
    const int reach = geometry.cellsFor(1.0);
    for (int dy = -reach; dy <= reach; ++dy)
    {
        for (int dx = -reach; dx <= reach; ++dx)
        {
            const int x = start.x + dx;
            const int y = start.y + dy;
            if (!geometry.contains(x, y) || !passable(x, y))
            {
                continue;
            }
            const auto distance = static_cast<float>(std::hypot(dx, dy) * geometry.resolution);
            if (distance > 1.0F)
            {
                continue;
            }
            const int index = geometry.index(x, y);
            if (distance < field[static_cast<std::size_t>(index)])
            {
                field[static_cast<std::size_t>(index)] = distance;
                queue.emplace(distance, index);
            }
        }
    }

    const auto straight = static_cast<float>(geometry.resolution);
    const auto diagonal = static_cast<float>(geometry.resolution * std::numbers::sqrt2);
    while (!queue.empty())
    {
        const auto [cost, index] = queue.top();
        queue.pop();
        if (cost > field[static_cast<std::size_t>(index)])
        {
            continue;
        }
        const int x = index % geometry.width;
        const int y = index / geometry.width;
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                if ((dx == 0 && dy == 0) || !geometry.contains(x + dx, y + dy) ||
                    !passable(x + dx, y + dy))
                {
                    continue;
                }
                // No corner cutting between two blocked cells.
                if (dx != 0 && dy != 0 && (!passable(x + dx, y) || !passable(x, y + dy)))
                {
                    continue;
                }
                const int   next = geometry.index(x + dx, y + dy);
                const float step = (dx != 0 && dy != 0) ? diagonal : straight;
                if (cost + step < field[static_cast<std::size_t>(next)])
                {
                    field[static_cast<std::size_t>(next)] = cost + step;
                    queue.emplace(cost + step, next);
                }
            }
        }
    }
}

void ViewpointPlanner::castRays(
    const CoverageMap& coverage, const CameraModel& camera, double x, double y,
    std::vector<Hit>& hits, std::vector<int>& offsets) const
{
    const GridGeometry&   geometry = coverage.geometry();
    const CoverageParams& measure  = coverage.params();
    const cv::Mat&        cells    = coverage.cells();
    const int             count    = static_cast<int>(geometry.cellCount());

    const double height   = camera.height;
    const double tan_low  = std::tan(camera.pitch + (0.5 * camera.vertical_fov));
    const double tan_high = std::tan(camera.pitch - (0.5 * camera.vertical_fov));
    const double tan_half = std::tan(0.5 * camera.vertical_fov);
    const double horizontal_reach =
        std::sqrt(std::max(0.0, (measure.max_range * measure.max_range) - (height * height)));
    // With the upper edge of the image below the horizon, the floor ends where that ray lands.
    const double floor_far =
        tan_high > 1e-6 ? std::min(height / tan_high, horizontal_reach) : horizontal_reach;
    const double floor_near = height / tan_low;
    const double reach      = floor_far;

    // Quality at the image centre (edge weight 1) and the vertical image offset of a view.
    const auto predict =
        [&](double along, double drop, double cos_incidence, TargetKind kind, int target) {
            const double below = std::atan2(drop, along);
            const double rho_v = std::tan(below - camera.pitch) / tan_half;
            if (std::abs(rho_v) >= 1.0)
            {
                return;
            }
            const double range = std::hypot(along, drop);
            const float  base  = coverage.viewQuality(range, cos_incidence, 1.0, kind);
            if (base < measure.well_seen)
            {
                return;  // No heading can lift it over the bar: edge weights only lower it.
            }
            hits.push_back({ target, base, static_cast<float>(rho_v), kind });
        };

    const double step = 0.5 * geometry.resolution;
    offsets.assign(static_cast<std::size_t>(params_.ray_count) + 1, 0);
    for (int ray = 0; ray < params_.ray_count; ++ray)
    {
        offsets[static_cast<std::size_t>(ray)] = static_cast<int>(hits.size());
        const double angle                     = kTwoPi * ray / params_.ray_count;
        const double dx                        = std::cos(angle);
        const double dy                        = std::sin(angle);
        int          last                      = -1;
        bool         floor_hidden              = false;
        const int    steps                     = static_cast<int>(reach / step);
        for (int n = 1; n <= steps; ++n)
        {
            const double    along = n * step;
            const CellIndex cell  = geometry.toCell(x + (dx * along), y + (dy * along));
            if (!geometry.contains(cell))
            {
                break;
            }
            const int index = geometry.index(cell);
            if (index == last)
            {
                continue;
            }
            last                 = index;
            const auto occupancy = cells.at<std::uint8_t>(cell.y, cell.x);
            if (occupancy == kUnknown)
            {
                break;
            }

            const auto face_hit = [&] {
                if (!coverage.pending(index) || coverage.kind(index) != TargetKind::kFace)
                {
                    return;
                }
                // The part of the face in view at this distance: between the image's lower
                // and upper edges, clipped to the band the measurement credits.
                const double low  = std::max(measure.face_min_z, height - (along * tan_low));
                const double high = std::min(measure.face_max_z, height - (along * tan_high));
                if (high <= low + 0.02)
                {
                    return;
                }
                const double    middle = 0.5 * (low + high);
                const double    drop   = height - middle;
                const cv::Vec2f normal = coverage.faceNormal(index);
                const double    cos_h  = -((dx * normal[0]) + (dy * normal[1]));
                const double    range  = std::hypot(along, drop);
                // As measured: the better of the side and the top.
                predict(
                    along,
                    drop,
                    std::max(cos_h * along / range, drop / range),
                    TargetKind::kFace,
                    index);
            };

            const int slot = coverage.surfaceAt(index);
            if (slot >= 0)
            {
                if (coverage.surfacePending(index))
                {
                    const double drop = height - coverage.surfaceHeight(slot);
                    if (drop > 0.0)
                    {
                        predict(
                            along,
                            drop,
                            drop / std::hypot(along, drop),
                            TargetKind::kSurface,
                            count + index);
                    }
                }
                face_hit();
                // The camera looks over a table, but the floor behind it is in its shadow.
                floor_hidden = true;
                continue;
            }
            if (occupancy == kOccupied)
            {
                face_hit();
                break;
            }
            if (!floor_hidden && along >= floor_near && coverage.pending(index))
            {
                predict(along, height, height / std::hypot(along, height), TargetKind::kFloor, index);
            }
        }
    }
    offsets[static_cast<std::size_t>(params_.ray_count)] = static_cast<int>(hits.size());
}

double ViewpointPlanner::blacklistFactor(double x, double y) const
{
    double factor = 1.0;
    for (const Blacklisted& entry : blacklist_)
    {
        if (std::hypot(entry.x - x, entry.y - y) > params_.blacklist_radius)
        {
            continue;
        }
        if (entry.failures >= 2)
        {
            return std::numeric_limits<double>::infinity();
        }
        factor *= 3.0;
    }
    return factor;
}

double ViewpointPlanner::turnTime(double from_yaw, std::vector<double>& headings) const
{
    // Nearest next: with at most a few headings this is the optimal order or close to it.
    double total   = 0.0;
    double current = from_yaw;
    for (std::size_t i = 0; i < headings.size(); ++i)
    {
        auto nearest = std::min_element(
            headings.begin() + static_cast<std::ptrdiff_t>(i),
            headings.end(),
            [current](double a, double b) {
                return std::abs(wrapAngle(a - current)) < std::abs(wrapAngle(b - current));
            });
        std::iter_swap(headings.begin() + static_cast<std::ptrdiff_t>(i), nearest);
        total += std::abs(wrapAngle(headings[i] - current));
        current = headings[i];
    }
    return total / params_.turn_speed;
}

Plan ViewpointPlanner::nextFrontier(
    const cv::Mat& cells, const GridGeometry& geometry, const Pose2D& robot)
{
    Plan plan;
    if (cells.empty())
    {
        plan.reason = "no map yet";
        return plan;
    }
    if (stuck(robot, plan))
    {
        return plan;
    }
    // Once the rooms are found the frontiers left are corners behind furniture: stop when the
    // last few visits added next to nothing, and let the camera pass fill them in.
    const double known_area = static_cast<double>(cv::countNonZero(cells != kUnknown)) *
                              geometry.resolution * geometry.resolution;
    const auto stall = static_cast<std::size_t>(std::max(1, params_.frontier_stall_visits));
    if (frontier_known_.size() >= stall &&
        known_area - frontier_known_[frontier_known_.size() - stall] < params_.min_frontier_growth)
    {
        plan.status = PlanStatus::kDone;
        plan.reason = std::format(
            "the last {} frontiers added under {:.0f} m2 of map",
            stall,
            params_.min_frontier_growth);
        return plan;
    }
    // A young SLAM map is known only along each beam a few metres out: close the gaps between
    // beams, never over a wall, or every gap is a frontier and none is a place to go.
    cv::Mat known = cells.clone();
    {
        cv::Mat   free_mask = cells == kFree;
        const int gap       = std::max(1, geometry.cellsFor(params_.frontier_gap_close));
        cv::morphologyEx(
            free_mask,
            free_mask,
            cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size((2 * gap) + 1, (2 * gap) + 1)));
        known.setTo(kFree, free_mask & (cells == kUnknown));
    }
    computeTraversable(known, geometry);
    computeTravel(geometry, robot);

    // Free cells beside unknown ones.
    cv::Mat frontier = cv::Mat::zeros(known.size(), CV_8UC1);
    for (int y = 1; y + 1 < known.rows; ++y)
    {
        for (int x = 1; x + 1 < known.cols; ++x)
        {
            if (known.at<std::uint8_t>(y, x) != kFree)
            {
                continue;
            }
            if (known.at<std::uint8_t>(y, x + 1) == kUnknown ||
                known.at<std::uint8_t>(y, x - 1) == kUnknown ||
                known.at<std::uint8_t>(y + 1, x) == kUnknown ||
                known.at<std::uint8_t>(y - 1, x) == kUnknown)
            {
                frontier.at<std::uint8_t>(y, x) = 255;
            }
        }
    }
    // Unknown the robot has already had in plain view from nearby is shadow the LiDAR cannot
    // reach, such as the floor behind an armchair whose wall it never saw, not a way on.
    dropSeenFrontier(frontier, known, geometry, trail_, params_.frontier_seen_radius);
    cv::Mat   labels;
    cv::Mat   stats;
    cv::Mat   centroids;
    const int found =
        cv::connectedComponentsWithStats(frontier, labels, stats, centroids, 8, CV_32S);
    // How much unknown each frontier opens onto: a pocket behind a sofa is a small region shut in
    // by furniture and walls, a doorway into an unexplored room joins the unknown beyond it.
    cv::Mat   unknown_labels;
    cv::Mat   unknown_stats;
    cv::Mat   unknown_centroids;
    const int unknown_regions = cv::connectedComponentsWithStats(
        known == kUnknown,
        unknown_labels,
        unknown_stats,
        unknown_centroids,
        4,
        CV_32S);
    std::vector<int> opens_onto(static_cast<std::size_t>(found), 0);
    for (int y = 1; y + 1 < labels.rows; ++y)
    {
        for (int x = 1; x + 1 < labels.cols; ++x)
        {
            const int component = labels.at<int>(y, x);
            if (component == 0)
            {
                continue;
            }
            for (const auto& [nx, ny] :
                 { std::pair{ x + 1, y }, { x - 1, y }, { x, y + 1 }, { x, y - 1 } })
            {
                const int region = unknown_labels.at<int>(ny, nx);
                if (region > 0 && region < unknown_regions)
                {
                    int& area = opens_onto[static_cast<std::size_t>(component)];
                    area      = std::max(area, unknown_stats.at<int>(region, cv::CC_STAT_AREA));
                }
            }
        }
    }
    const double min_unknown_cells =
        params_.min_frontier_unknown / (geometry.resolution * geometry.resolution);

    // Each frontier's own cell nearest its centroid: the centroid of a curved frontier can lie
    // out in the unknown, where nothing stands.
    std::vector<CellIndex> anchor(static_cast<std::size_t>(found));
    std::vector<double>    anchor_error(
        static_cast<std::size_t>(found),
        std::numeric_limits<double>::infinity());
    for (int y = 0; y < labels.rows; ++y)
    {
        for (int x = 0; x < labels.cols; ++x)
        {
            const int component = labels.at<int>(y, x);
            if (component == 0)
            {
                continue;
            }
            const double error = std::hypot(
                x - centroids.at<double>(component, 0),
                y - centroids.at<double>(component, 1));
            if (error < anchor_error[static_cast<std::size_t>(component)])
            {
                anchor_error[static_cast<std::size_t>(component)] = error;
                anchor[static_cast<std::size_t>(component)]       = { x, y };
            }
        }
    }

    const int min_cells = geometry.cellsFor(params_.min_frontier_size);
    const int search    = geometry.cellsFor(1.5);
    // Wider than a camera pose needs: a frontier's small shadows lead into furniture pockets,
    // which let the robot in and Nav2 cannot bring it out of.
    const double standable     = params_.robot_radius + params_.frontier_clearance;
    double       best          = 0.0;
    double       best_target_x = 0.0;
    double       best_target_y = 0.0;
    for (int component = 1; component < found; ++component)
    {
        const int size = stats.at<int>(component, cv::CC_STAT_AREA);
        if (size < min_cells || opens_onto[static_cast<std::size_t>(component)] < min_unknown_cells)
        {
            continue;
        }
        const double target_x  = geometry.centreX(anchor[static_cast<std::size_t>(component)].x);
        const double target_y  = geometry.centreY(anchor[static_cast<std::size_t>(component)].y);
        const bool   exhausted = std::any_of(
            exhausted_frontiers_.begin(),
            exhausted_frontiers_.end(),
            [&](const auto& entry) {
                return entry.failures >= 2 &&
                       std::hypot(entry.x - target_x, entry.y - target_y) < 0.75;
            });
        if (exhausted)
        {
            continue;
        }

        // The reachable standing cell nearest the frontier's middle, a step back from it so the
        // robot has a direction to face.
        const CellIndex centre     = geometry.toCell(target_x, target_y);
        const int       stand_back = geometry.cellsFor(params_.frontier_stand_back);
        int             goal       = -1;
        double          goal_error = 0.0;
        for (int dy = -search; dy <= search; ++dy)
        {
            for (int dx = -search; dx <= search; ++dx)
            {
                const int x = centre.x + dx;
                const int y = centre.y + dy;
                if (!geometry.contains(x, y) || clearance_.at<float>(y, x) < standable ||
                    (dx * dx) + (dy * dy) < stand_back * stand_back)
                {
                    continue;
                }
                const int index = geometry.index(x, y);
                if (!std::isfinite(travel_[static_cast<std::size_t>(index)]))
                {
                    continue;
                }
                const double error = std::hypot(dx, dy);
                if (goal < 0 || error < goal_error)
                {
                    goal       = index;
                    goal_error = error;
                }
            }
        }
        if (goal < 0)
        {
            continue;
        }
        const double goal_x = geometry.centreX(goal % geometry.width);
        const double goal_y = geometry.centreY(goal / geometry.width);
        const double factor = blacklistFactor(goal_x, goal_y);
        if (!std::isfinite(factor))
        {
            continue;
        }
        std::vector<double> headings{ std::atan2(target_y - goal_y, target_x - goal_x) };
        const double travel_time = travel_[static_cast<std::size_t>(goal)] / params_.travel_speed;
        const double arrival     = std::hypot(goal_x - robot.x, goal_y - robot.y) > 0.3 ?
                                       std::atan2(goal_y - robot.y, goal_x - robot.x) :
                                       robot.yaw;
        const double cost =
            (travel_time + turnTime(arrival, headings) + params_.goal_overhead) * factor;
        const double gain    = size * geometry.resolution;
        const double utility = gain / cost;
        if (utility > best)
        {
            best                    = utility;
            plan.status             = PlanStatus::kViewpoint;
            plan.viewpoint.x        = goal_x;
            plan.viewpoint.y        = goal_y;
            plan.viewpoint.headings = headings;
            plan.viewpoint.gain     = gain;
            plan.viewpoint.cost     = cost;
            best_target_x           = target_x;
            best_target_y           = target_y;
        }
    }

    if (plan.status != PlanStatus::kViewpoint)
    {
        plan.status = PlanStatus::kDone;
        plan.reason = "no reachable frontier left";
        return plan;
    }
    plan.viewpoint.id = next_id_++;
    frontier_known_.push_back(known_area);
    // Kept so a frontier that survives two visits is dropped, and an unreachable goal avoided.
    issued_frontiers_.push_back({ plan.viewpoint.id, best_target_x, best_target_y });
    issued_.push_back(plan.viewpoint);
    if (issued_frontiers_.size() > 16)
    {
        issued_frontiers_.erase(issued_frontiers_.begin());
    }
    if (issued_.size() > 16)
    {
        issued_.erase(issued_.begin());
    }
    return plan;
}

Plan ViewpointPlanner::nextCoverage(
    CoverageMap& coverage, const cv::Mat& room_labels, const Pose2D& robot)
{
    Plan                plan;
    const GridGeometry& geometry = coverage.geometry();
    const cv::Mat&      cells    = coverage.cells();
    if (cells.empty())
    {
        plan.reason = "no map yet";
        return plan;
    }
    if (stuck(robot, plan))
    {
        return plan;
    }
    computeTraversable(cells, geometry);
    computeTravel(geometry, robot);

    const int count = static_cast<int>(geometry.cellCount());
    cv::Mat   pending(cells.size(), CV_8UC1, cv::Scalar(0));
    int       pending_count = 0;
    for (int index = 0; index < count; ++index)
    {
        if (coverage.pending(index) || coverage.surfacePending(index))
        {
            pending.ptr<std::uint8_t>(0)[index] = 255;
            ++pending_count;
        }
    }
    if (pending_count == 0)
    {
        plan.status = PlanStatus::kDone;
        plan.reason = "every observable target has been seen";
        return plan;
    }

    if (cameras_.empty())
    {
        plan.reason = "no camera yet";
        return plan;
    }
    // How far a pending target can be from a pose that might see it: the lowest camera reaches
    // furthest.
    const double lowest = std::ranges::min(cameras_, {}, &CameraModel::height).height;
    const double reach  = std::sqrt(std::max(
        0.0,
        (coverage.params().max_range * coverage.params().max_range) - (lowest * lowest)));
    cv::Mat      to_pending;
    cv::distanceTransform(pending == 0, to_pending, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    to_pending *= geometry.resolution;

    const bool has_rooms = !room_labels.empty() && room_labels.size() == cells.size();
    const auto room_at   = [&](int x, int y) { return has_rooms ? room_labels.at<int>(y, x) : 0; };
    const CellIndex robot_cell = geometry.toCell(robot.x, robot.y);
    const int       current_room =
        geometry.contains(robot_cell) ? room_at(robot_cell.x, robot_cell.y) : 0;

    // A room the robot has never got into after a few tries has a way in Nav2 will not take: too
    // tight, or shut. Once inside, failures are local and the spot blacklist deals with them.
    // Replayed against today's labels, which re-segmentation renumbers.
    struct Attempts
    {
        int  failures = 0;
        bool entered  = false;
    };
    std::vector<Attempts> attempts;
    if (has_rooms)
    {
        for (const Outcome& outcome : outcomes_)
        {
            const CellIndex cell = geometry.toCell(outcome.x, outcome.y);
            const int       room = geometry.contains(cell) ? room_at(cell.x, cell.y) : 0;
            if (room <= 0)
            {
                continue;
            }
            if (room >= static_cast<int>(attempts.size()))
            {
                attempts.resize(static_cast<std::size_t>(room) + 1);
            }
            Attempts& tally = attempts[static_cast<std::size_t>(room)];
            tally.entered   = tally.entered || outcome.reached;
            tally.failures += static_cast<int>(!outcome.reached);
        }
    }
    const auto given_up = [&](int room) {
        return room > 0 && room < static_cast<int>(attempts.size()) &&
               !attempts[static_cast<std::size_t>(room)].entered &&
               attempts[static_cast<std::size_t>(room)].failures >= params_.max_room_failures;
    };

    struct Candidate
    {
        int    index;
        double score;
    };
    std::vector<Candidate> candidates;
    // Standable but not evaluated: out of reach, given up or cut by the cap. Only the done
    // write-off looks at them, so that where the robot happens to be never decides what exists.
    std::vector<Candidate> rest;
    const int              spacing   = std::max(1, geometry.cellsFor(params_.candidate_spacing));
    const double           standable = params_.robot_radius + params_.clearance_margin;
    for (int y = spacing / 2; y < geometry.height; y += spacing)
    {
        for (int x = spacing / 2; x < geometry.width; x += spacing)
        {
            const int index = geometry.index(x, y);
            if (cells.at<std::uint8_t>(y, x) != kFree || clearance_.at<float>(y, x) < standable ||
                to_pending.at<float>(y, x) > reach)
            {
                continue;
            }
            const bool reachable =
                std::isfinite(travel_[static_cast<std::size_t>(index)]) && !given_up(room_at(x, y));
            (reachable ? candidates : rest).push_back({ index, 0.0 });
        }
    }
    // Does the room the robot stands in still hold work worth a viewpoint?
    bool room_has_work = false;
    if (current_room > 0)
    {
        int left = 0;
        for (int index = 0; index < count && !room_has_work; ++index)
        {
            if (pending.ptr<std::uint8_t>(0)[index] != 0 &&
                room_labels.ptr<int>(0)[index] == current_room)
            {
                room_has_work = ++left >= static_cast<int>(params_.min_viewpoint_gain);
            }
        }
    }

    if (static_cast<int>(candidates.size()) > params_.max_candidates)
    {
        // Rank by pending work in sight per second of getting there: by work alone the cut drops
        // the room the robot stands in, which it leaves half done and comes back to later.
        cv::Mat integral;
        cv::integral(pending / 255, integral, CV_32S);
        const int window = geometry.cellsFor(reach);
        for (Candidate& candidate : candidates)
        {
            const int    x    = candidate.index % geometry.width;
            const int    y    = candidate.index / geometry.width;
            const int    x0   = std::max(0, x - window);
            const int    y0   = std::max(0, y - window);
            const int    x1   = std::min(geometry.width, x + window + 1);
            const int    y1   = std::min(geometry.height, y + window + 1);
            const double work = integral.at<int>(y1, x1) - integral.at<int>(y0, x1) -
                                integral.at<int>(y1, x0) + integral.at<int>(y0, x0);
            const double seconds =
                (travel_[static_cast<std::size_t>(candidate.index)] / params_.travel_speed) +
                params_.goal_overhead + params_.dwell_time;
            const bool elsewhere = room_has_work && room_at(x, y) != current_room;
            candidate.score = work / (seconds * (elsewhere ? params_.room_switch_factor : 1.0));
        }
        std::nth_element(
            candidates.begin(),
            candidates.begin() + params_.max_candidates,
            candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.score > b.score; });
        rest.insert(rest.end(), candidates.begin() + params_.max_candidates, candidates.end());
        candidates.resize(static_cast<std::size_t>(params_.max_candidates));
    }

    chosen_mark_.resize(static_cast<std::size_t>(count) * 2, 0);
    heading_mark_.resize(static_cast<std::size_t>(count) * 2, 0);
    std::vector<std::uint8_t> predicted_any(static_cast<std::size_t>(count) * 2, 0);

    const int    per_heading = std::max(1, params_.ray_count / std::max(1, params_.heading_count));
    const double ray_angle   = kTwoPi / params_.ray_count;
    // Per camera: the rays it casts from a candidate, and where across its image each ray falls.
    struct CameraRays
    {
        const CameraModel* camera = nullptr;
        std::vector<Hit>   hits;
        std::vector<int>   offsets;
        int                half_rays = 0;
        int                yaw_rays  = 0;
        std::vector<float> rho_h;  // Horizontal image offset per ray from the optical axis.
    };
    std::vector<CameraRays> views(cameras_.size());
    for (std::size_t c = 0; c < cameras_.size(); ++c)
    {
        CameraRays& view = views[c];
        view.camera      = &cameras_[c];
        view.half_rays =
            static_cast<int>(std::floor(0.5 * view.camera->horizontal_fov / ray_angle));
        view.yaw_rays           = static_cast<int>(std::lround(view.camera->yaw / ray_angle));
        const double tan_half_h = std::tan(0.5 * view.camera->horizontal_fov);
        for (int offset = -view.half_rays; offset <= view.half_rays; ++offset)
        {
            view.rho_h.push_back(static_cast<float>(std::tan(offset * ray_angle) / tan_half_h));
        }
    }
    const auto cast_all = [&](double x, double y) {
        bool any = false;
        for (CameraRays& view : views)
        {
            view.hits.clear();
            castRays(coverage, *view.camera, x, y, view.hits, view.offsets);
            any = any || !view.hits.empty();
        }
        return any;
    };
    // Rooms still short of their share of floor or faces seen; past the rate cut only they count.
    int room_count = 0;
    if (has_rooms)
    {
        double highest = 0.0;
        cv::minMaxLoc(room_labels, nullptr, &highest);
        room_count = static_cast<int>(highest);
    }
    std::vector<std::uint8_t> short_room(static_cast<std::size_t>(room_count) + 1, 0);
    bool                      any_short = false;
    if (room_count > 0)
    {
        const std::vector<CoverageTally> tallies = coverage.tally(room_labels, room_count);
        for (int room = 1; room <= room_count; ++room)
        {
            const CoverageTally& t = tallies[static_cast<std::size_t>(room)];
            const bool           is_short =
                (t.floor > 0 && t.floor_seen < params_.room_floor_goal * t.floor) ||
                (t.face > 0 && t.face_seen < params_.room_face_goal * t.face);
            short_room[static_cast<std::size_t>(room)] = static_cast<std::uint8_t>(is_short);
            any_short                                  = any_short || is_short;
        }
    }
    const auto counts = [&](int target) {
        if (!focusing_)
        {
            return 1.0;
        }
        const int room = coverage.roomOf(target, room_labels);
        return room > 0 && room <= room_count && short_room[static_cast<std::size_t>(room)] != 0 ?
                   1.0 :
                   0.0;
    };
    const auto weight = [this](TargetKind kind) {
        switch (kind)
        {
            case TargetKind::kFace:
                return params_.face_weight;
            case TargetKind::kSurface:
                return params_.surface_weight;
            default:
                return 1.0;
        }
    };
    const auto well_seen = static_cast<float>(coverage.params().well_seen);

    double     best_utility = 0.0;
    double     best_gain    = 0.0;
    const bool room_short   = current_room > 0 && current_room <= room_count &&
                            short_room[static_cast<std::size_t>(current_room)] != 0;
    std::vector<Viewpoint> options;  // Every candidate worth a visit, for the tour.
    const auto             choose = [&] {
        best_utility = 0.0;
        best_gain    = 0.0;
        plan         = Plan{};
        options.clear();
        for (const Candidate& candidate : candidates)
        {
            const int    cx     = candidate.index % geometry.width;
            const int    cy     = candidate.index / geometry.width;
            const double x      = geometry.centreX(cx);
            const double y      = geometry.centreY(cy);
            const double factor = blacklistFactor(x, y);
            if (!std::isfinite(factor))
            {
                continue;
            }
            if (!cast_all(x, y))
            {
                continue;
            }

            // Gain of heading j over targets no chosen heading covers yet.
            const auto heading_gain = [&](int heading, bool commit, std::vector<int>* seen) {
                ++heading_value_;
                double gain = 0.0;
                for (const CameraRays& view : views)
                {
                    for (int slot = 0; slot <= 2 * view.half_rays; ++slot)
                    {
                        const int turned =
                            (heading * per_heading) + view.yaw_rays + slot - view.half_rays;
                        const int ray =
                            ((turned % params_.ray_count) + params_.ray_count) % params_.ray_count;
                        const float horizontal =
                            std::abs(view.rho_h[static_cast<std::size_t>(slot)]);
                        for (int h = view.offsets[static_cast<std::size_t>(ray)];
                             h < view.offsets[static_cast<std::size_t>(ray) + 1];
                             ++h)
                        {
                            const Hit&  hit = view.hits[static_cast<std::size_t>(h)];
                            const float rho = std::max(horizontal, std::abs(hit.rho_v));
                            const float edge = 1.0F - (rho * rho * rho * rho);
                            if (hit.base * edge < well_seen)
                            {
                                continue;
                            }
                            const auto target = static_cast<std::size_t>(hit.target);
                            predicted_any[target] = 1;
                            if (chosen_mark_[target] == chosen_value_ ||
                                heading_mark_[target] == heading_value_)
                            {
                                continue;
                            }
                            heading_mark_[target] = heading_value_;
                            gain += weight(hit.kind) * counts(hit.target);
                            if (commit)
                            {
                                chosen_mark_[target] = chosen_value_;
                                if (seen != nullptr)
                                {
                                    seen->push_back(hit.target);
                                }
                            }
                        }
                    }
                }
                return gain;
            };

            ++chosen_value_;
            std::vector<double> headings;
            std::vector<int>    seen;
            double              gain = 0.0;
            std::vector<bool>   used(static_cast<std::size_t>(params_.heading_count), false);
            for (int pick = 0; pick < params_.max_headings; ++pick)
            {
                int    best_heading = -1;
                double best_added   = 0.0;
                for (int heading = 0; heading < params_.heading_count; ++heading)
                {
                    if (used[static_cast<std::size_t>(heading)])
                    {
                        continue;
                    }
                    const double added = heading_gain(heading, false, nullptr);
                    if (added > best_added)
                    {
                        best_added   = added;
                        best_heading = heading;
                    }
                }
                const double needed = pick == 0 ? 1.0 : params_.min_heading_gain;
                if (best_heading < 0 || best_added < needed)
                {
                    break;
                }
                used[static_cast<std::size_t>(best_heading)] = true;
                heading_gain(best_heading, true, &seen);
                headings.push_back(wrapAngle(best_heading * per_heading * ray_angle));
                gain += best_added;
            }
            if (headings.empty())
            {
                continue;
            }

            const double travel_time =
                travel_[static_cast<std::size_t>(candidate.index)] / params_.travel_speed;
            const double arrival = std::hypot(x - robot.x, y - robot.y) > 0.3 ?
                                                   std::atan2(y - robot.y, x - robot.x) :
                                                   robot.yaw;
            double       cost    = travel_time + turnTime(arrival, headings) +
                          (static_cast<double>(headings.size()) * params_.dwell_time) +
                          params_.goal_overhead;
            cost *= factor;
            const int room = room_at(cx, cy);
            if (room_has_work && room != current_room)
            {
                cost *= params_.room_switch_factor;
            }
            const double utility = gain / cost;
            best_gain            = std::max(best_gain, gain);
            Viewpoint here;
            here.x         = x;
            here.y         = y;
            here.headings  = headings;
            here.room      = room;
            here.gain      = gain;
            here.cost      = cost;
            here.predicted = seen;
            if (gain >= params_.min_viewpoint_gain)
            {
                options.push_back(here);
            }
            if (utility > best_utility)
            {
                best_utility   = utility;
                plan.status    = PlanStatus::kViewpoint;
                plan.viewpoint = std::move(here);
            }
        }
        // The room the robot stands in keeps it while short of its share, or while a viewpoint
        // there still pays the pass's rate: leaving either way meant walking back for it later
        // (run 31: 15 m for 629 targets; run 34: 13 m for 669 in a room already past its share).
        const auto in_room = [&](const Viewpoint& option) { return option.room == current_room; };
        const auto rate = [](const Viewpoint& option) { return option.gain / option.cost; };
        const bool stay =
            current_room > 0 && std::ranges::any_of(options, [&](const Viewpoint& option) {
                return in_room(option) &&
                       (room_short || rate(option) >= params_.min_viewpoint_rate);
            });
        if (stay)
        {
            std::erase_if(options, [&](const Viewpoint& option) { return !in_room(option); });
            plan.viewpoint = *std::ranges::max_element(options, {}, rate);
        }
    };
    choose();
    if (best_utility < params_.min_viewpoint_rate && any_short && !focusing_ &&
        best_gain >= params_.min_viewpoint_gain)
    {
        // Viewpoints no longer pay for themselves, but rooms are short of their share: from here
        // on only their targets count, until they have it or nothing more there can be seen.
        focusing_ = true;
        choose();
    }
    if (best_gain < params_.min_viewpoint_gain ||
        (best_utility < params_.min_viewpoint_rate && !any_short))
    {
        // Any heading can centre a ray, so only its vertical offset counts.
        for (const Candidate& candidate : rest)
        {
            cast_all(
                geometry.centreX(candidate.index % geometry.width),
                geometry.centreY(candidate.index / geometry.width));
            for (const CameraRays& view : views)
            {
                for (const Hit& hit : view.hits)
                {
                    const float rho = std::abs(hit.rho_v);
                    if (hit.base * (1.0F - (rho * rho * rho * rho)) >= well_seen)
                    {
                        predicted_any[static_cast<std::size_t>(hit.target)] = 1;
                    }
                }
            }
        }
        // Nothing reachable predicts a good view of these: stop counting them as missing.
        int written_off = 0;
        for (int index = 0; index < count; ++index)
        {
            if (coverage.pending(index) && predicted_any[static_cast<std::size_t>(index)] == 0)
            {
                coverage.markUnobservable(index);
                ++written_off;
            }
            if (coverage.surfacePending(index) &&
                predicted_any[static_cast<std::size_t>(count) + static_cast<std::size_t>(index)] ==
                    0)
            {
                coverage.markSurfaceUnobservable(index);
                ++written_off;
            }
        }
        plan        = Plan{};
        plan.status = PlanStatus::kDone;
        plan.reason = "no viewpoint adds enough; " + std::to_string(written_off) +
                      " targets written off as unobservable";
        return plan;
    }

    // Greedy by gain per second zigzags across a room: of the few viewpoints that together see
    // the most, go to the first on the shortest walk through them.
    if (auto first = tourStart(std::move(options), geometry, [&](int target) {
            const double kind =
                target >= count ? params_.surface_weight : weight(coverage.kind(target));
            return kind * counts(target);
        }))
    {
        plan.viewpoint = std::move(*first);
    }
    coverage.setPlanned(predicted_any);
    plan.viewpoint.id = next_id_++;
    issued_.push_back(plan.viewpoint);
    // Only the recent past matters for reports; keep the list short.
    if (issued_.size() > 16)
    {
        issued_.erase(issued_.begin());
    }
    return plan;
}

std::optional<Viewpoint> ViewpointPlanner::tourStart(
    std::vector<Viewpoint> options, const GridGeometry& geometry,
    const std::function<double(int)>& worth) const
{
    // Greedy cover: each pick adds the most of what the ones before it leave unseen.
    std::vector<std::uint8_t> covered(2 * geometry.cellCount(), 0);
    std::vector<Viewpoint>    picks;
    while (static_cast<int>(picks.size()) < params_.tour_size && !options.empty())
    {
        double      best_added = 0.0;
        std::size_t best       = 0;
        for (std::size_t i = 0; i < options.size(); ++i)
        {
            double added = 0.0;
            for (const int target : options[i].predicted)
            {
                if (covered[static_cast<std::size_t>(target)] == 0)
                {
                    added += worth(target);
                }
            }
            if (added > best_added)
            {
                best_added = added;
                best       = i;
            }
        }
        if (best_added < params_.min_viewpoint_gain)
        {
            break;
        }
        for (const int target : options[best].predicted)
        {
            covered[static_cast<std::size_t>(target)] = 1;
        }
        picks.push_back(std::move(options[best]));
        options.erase(options.begin() + static_cast<std::ptrdiff_t>(best));
    }
    if (picks.size() < 2)
    {
        return std::nullopt;
    }

    // Walking distances: from the robot, already known, and between the picks.
    const std::size_t count = picks.size();
    const auto        cell  = [&](const Viewpoint& v) {
        return static_cast<std::size_t>(geometry.index(geometry.toCell(v.x, v.y)));
    };
    std::vector<double> start(count);
    std::vector<double> between(count * count);
    std::vector<float>  field;
    for (std::size_t i = 0; i < count; ++i)
    {
        start[i] = travel_[cell(picks[i])];
        travelFrom(geometry, picks[i].x, picks[i].y, field);
        for (std::size_t j = 0; j < count; ++j)
        {
            between[(i * count) + j] = field[cell(picks[j])];
        }
    }
    const auto walk = [&](const std::vector<std::size_t>& order) {
        double total = start[order.front()];
        for (std::size_t k = 1; k < order.size(); ++k)
        {
            total += between[(order[k - 1] * count) + order[k]];
        }
        return total;
    };
    // Nearest first, then untangle crossings (2-opt on the open walk).
    std::vector<std::size_t> order;
    std::vector<bool>        placed(count, false);
    for (std::size_t k = 0; k < count; ++k)
    {
        std::size_t next    = count;
        double      nearest = std::numeric_limits<double>::infinity();
        for (std::size_t j = 0; j < count; ++j)
        {
            const double d = order.empty() ? start[j] : between[(order.back() * count) + j];
            if (!placed[j] && (next == count || d < nearest))
            {
                next    = j;
                nearest = d;
            }
        }
        placed[next] = true;
        order.push_back(next);
    }
    for (bool improved = true; improved;)
    {
        improved = false;
        for (std::size_t i = 0; i + 1 < count; ++i)
        {
            for (std::size_t j = i + 1; j < count; ++j)
            {
                std::vector<std::size_t> swapped = order;
                std::reverse(
                    swapped.begin() + static_cast<std::ptrdiff_t>(i),
                    swapped.begin() + static_cast<std::ptrdiff_t>(j) + 1);
                if (walk(swapped) < walk(order) - 1e-6)
                {
                    order    = std::move(swapped);
                    improved = true;
                }
            }
        }
    }
    return std::move(picks[order.front()]);
}

void ViewpointPlanner::recordPose(const Pose2D& robot)
{
    pending_trail_.emplace_back(robot.x, robot.y);
}

void ViewpointPlanner::mapUpdated()
{
    // SLAM folds scans into the map it publishes every few seconds; until then what the robot
    // saw from a pose is not in the map, and the frontiers there only look unexplored.
    trail_.insert(trail_.end(), pending_trail_.begin(), pending_trail_.end());
    pending_trail_.clear();
}

bool ViewpointPlanner::stuck(const Pose2D& robot, Plan& plan)
{
    if (failures_in_a_row_ < params_.max_failures_in_a_row)
    {
        return false;
    }
    if (!stuck_at_)
    {
        // The failures say nothing about the rooms they were in: none of them count there.
        const auto recent =
            std::min(outcomes_.size(), static_cast<std::size_t>(failures_in_a_row_));
        outcomes_.resize(outcomes_.size() - recent);
        stuck_at_ = robot;
    }
    if (std::hypot(robot.x - stuck_at_->x, robot.y - stuck_at_->y) >= params_.blacklist_radius)
    {
        failures_in_a_row_ = 0;
        stuck_at_.reset();
        return false;
    }
    plan.status = PlanStatus::kUnavailable;
    plan.reason = "the last " + std::to_string(failures_in_a_row_) +
                  " viewpoints were not reached; the robot seems stuck";
    return true;
}

void ViewpointPlanner::report(CoverageMap& coverage, std::uint32_t id, bool reached)
{
    failures_in_a_row_ = reached ? 0 : failures_in_a_row_ + 1;
    if (reached)
    {
        stuck_at_.reset();
    }
    const auto frontier = std::find_if(
        issued_frontiers_.begin(),
        issued_frontiers_.end(),
        [id](const IssuedFrontier& entry) { return entry.id == id; });
    if (frontier != issued_frontiers_.end() && reached)
    {
        // Twice reached and still a frontier: glass, a mirror, or space the LiDAR cannot see.
        auto entry = std::find_if(
            exhausted_frontiers_.begin(),
            exhausted_frontiers_.end(),
            [&](const Blacklisted& e) {
                return std::hypot(e.x - frontier->target_x, e.y - frontier->target_y) < 0.75;
            });
        if (entry == exhausted_frontiers_.end())
        {
            exhausted_frontiers_.push_back({ frontier->target_x, frontier->target_y, 1 });
        }
        else
        {
            ++entry->failures;
        }
    }
    if (frontier != issued_frontiers_.end())
    {
        // A walk navigation could not finish says nothing about what is left to find: it is not
        // one of the visits the stall rule counts.
        if (!reached && !frontier_known_.empty() && frontier->id == issued_frontiers_.back().id)
        {
            frontier_known_.pop_back();
        }
        issued_frontiers_.erase(frontier);
    }

    const auto issued = std::find_if(issued_.begin(), issued_.end(), [id](const Viewpoint& v) {
        return v.id == id;
    });
    if (issued == issued_.end())
    {
        return;
    }
    outcomes_.push_back({ issued->x, issued->y, reached });
    if (reached)
    {
        recordPose({ issued->x, issued->y, 0.0 });
    }
    if (outcomes_.size() > 256)
    {
        outcomes_.erase(outcomes_.begin());
    }
    if (!reached)
    {
        auto entry = std::find_if(blacklist_.begin(), blacklist_.end(), [&](const Blacklisted& e) {
            return std::hypot(e.x - issued->x, e.y - issued->y) < params_.blacklist_radius;
        });
        if (entry == blacklist_.end())
        {
            blacklist_.push_back({ issued->x, issued->y, 1 });
        }
        else
        {
            ++entry->failures;
        }
    }
    else
    {
        const int count = static_cast<int>(coverage.geometry().cellCount());
        for (const int target : issued->predicted)
        {
            if (target < count)
            {
                if (coverage.pending(target) &&
                    coverage.countAttempt(target) >= params_.max_attempts)
                {
                    coverage.markUnobservable(target);
                }
            }
            else if (
                coverage.surfacePending(target - count) &&
                coverage.countAttempt(target - count) >= params_.max_attempts)
            {
                coverage.markSurfaceUnobservable(target - count);
            }
        }
    }
    issued_.erase(issued);
}

}  // namespace g1_world_model
