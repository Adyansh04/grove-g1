/**
 * @file viewpoint_planner.cpp
 * @brief Frontier and coverage viewpoints: travel field, ray prediction, heading selection.
 */

#include "g1_world_model/viewpoint_planner.hpp"

#include <algorithm>
#include <cmath>
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

}  // namespace

ViewpointPlanner::ViewpointPlanner(PlannerParams params, CameraModel camera)
  : params_(params)
  , camera_(camera)
{}

void ViewpointPlanner::computeTraversable(const cv::Mat& cells, const GridGeometry& geometry)
{
    const cv::Mat free_mask = cells == kFree;
    cv::distanceTransform(free_mask, clearance_, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    clearance_ *= geometry.resolution;
    // Nav2 inflates every cell within robot_radius to lethal, centre to centre; the margin keeps
    // a doorway exactly twice that wide from being a path here and a wall there.
    traversable_ = clearance_ >= params_.robot_radius + params_.travel_margin;
}

void ViewpointPlanner::computeTravel(const GridGeometry& geometry, const Pose2D& robot)
{
    travel_.assign(geometry.cellCount(), kUnreachable);
    const CellIndex start = geometry.toCell(robot.x, robot.y);
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
            if (distance < travel_[static_cast<std::size_t>(index)])
            {
                travel_[static_cast<std::size_t>(index)] = distance;
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
        if (cost > travel_[static_cast<std::size_t>(index)])
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
                if (cost + step < travel_[static_cast<std::size_t>(next)])
                {
                    travel_[static_cast<std::size_t>(next)] = cost + step;
                    queue.emplace(cost + step, next);
                }
            }
        }
    }
}

void ViewpointPlanner::castRays(
    const CoverageMap& coverage, double x, double y, std::vector<Hit>& hits,
    std::vector<int>& offsets) const
{
    const GridGeometry&   geometry = coverage.geometry();
    const CoverageParams& measure  = coverage.params();
    const cv::Mat&        cells    = coverage.cells();
    const int             count    = static_cast<int>(geometry.cellCount());

    const double height   = camera_.height;
    const double tan_low  = std::tan(camera_.pitch + (0.5 * camera_.vertical_fov));
    const double tan_high = std::tan(camera_.pitch - (0.5 * camera_.vertical_fov));
    const double tan_half = std::tan(0.5 * camera_.vertical_fov);
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
            const double rho_v = std::tan(below - camera_.pitch) / tan_half;
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
    computeTraversable(cells, geometry);
    computeTravel(geometry, robot);

    // Free cells beside unknown ones.
    cv::Mat frontier = cv::Mat::zeros(cells.size(), CV_8UC1);
    for (int y = 1; y + 1 < cells.rows; ++y)
    {
        for (int x = 1; x + 1 < cells.cols; ++x)
        {
            if (cells.at<std::uint8_t>(y, x) != kFree)
            {
                continue;
            }
            if (cells.at<std::uint8_t>(y, x + 1) == kUnknown ||
                cells.at<std::uint8_t>(y, x - 1) == kUnknown ||
                cells.at<std::uint8_t>(y + 1, x) == kUnknown ||
                cells.at<std::uint8_t>(y - 1, x) == kUnknown)
            {
                frontier.at<std::uint8_t>(y, x) = 255;
            }
        }
    }
    cv::Mat   labels;
    cv::Mat   stats;
    cv::Mat   centroids;
    const int found = cv::connectedComponentsWithStats(frontier, labels, stats, centroids, 8);

    const int    min_cells     = geometry.cellsFor(params_.min_frontier_size);
    const int    search        = geometry.cellsFor(1.5);
    const double standable     = params_.robot_radius + params_.clearance_margin;
    double       best          = 0.0;
    double       best_target_x = 0.0;
    double       best_target_y = 0.0;
    for (int component = 1; component < found; ++component)
    {
        const int size = stats.at<int>(component, cv::CC_STAT_AREA);
        if (size < min_cells)
        {
            continue;
        }
        const double target_x =
            geometry.origin_x + ((centroids.at<double>(component, 0) + 0.5) * geometry.resolution);
        const double target_y =
            geometry.origin_y + ((centroids.at<double>(component, 1) + 0.5) * geometry.resolution);
        const bool exhausted = std::any_of(
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

        // The reachable standing cell nearest the frontier's middle.
        const CellIndex centre     = geometry.toCell(target_x, target_y);
        int             goal       = -1;
        double          goal_error = 0.0;
        for (int dy = -search; dy <= search; ++dy)
        {
            for (int dx = -search; dx <= search; ++dx)
            {
                const int x = centre.x + dx;
                const int y = centre.y + dy;
                if (!geometry.contains(x, y) || clearance_.at<float>(y, x) < standable)
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

    // How far a pending target can be from a pose that might see it.
    const double reach = std::sqrt(std::max(
        0.0,
        (coverage.params().max_range * coverage.params().max_range) -
            (camera_.height * camera_.height)));
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
            if (clearance_.at<float>(y, x) < standable || to_pending.at<float>(y, x) > reach)
            {
                continue;
            }
            const bool reachable =
                std::isfinite(travel_[static_cast<std::size_t>(index)]) && !given_up(room_at(x, y));
            (reachable ? candidates : rest).push_back({ index, 0.0 });
        }
    }
    if (static_cast<int>(candidates.size()) > params_.max_candidates)
    {
        // Rank by how much pending work lies within sight, a cheap stand-in for the gain.
        cv::Mat integral;
        cv::integral(pending / 255, integral, CV_32S);
        const int window = geometry.cellsFor(reach);
        for (Candidate& candidate : candidates)
        {
            const int x     = candidate.index % geometry.width;
            const int y     = candidate.index / geometry.width;
            const int x0    = std::max(0, x - window);
            const int y0    = std::max(0, y - window);
            const int x1    = std::min(geometry.width, x + window + 1);
            const int y1    = std::min(geometry.height, y + window + 1);
            candidate.score = integral.at<int>(y1, x1) - integral.at<int>(y0, x1) -
                              integral.at<int>(y1, x0) + integral.at<int>(y0, x0);
        }
        std::nth_element(
            candidates.begin(),
            candidates.begin() + params_.max_candidates,
            candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.score > b.score; });
        rest.insert(rest.end(), candidates.begin() + params_.max_candidates, candidates.end());
        candidates.resize(static_cast<std::size_t>(params_.max_candidates));
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

    chosen_mark_.resize(static_cast<std::size_t>(count) * 2, 0);
    heading_mark_.resize(static_cast<std::size_t>(count) * 2, 0);
    std::vector<std::uint8_t> predicted_any(static_cast<std::size_t>(count) * 2, 0);

    const int    per_heading = std::max(1, params_.ray_count / std::max(1, params_.heading_count));
    const double ray_angle   = kTwoPi / params_.ray_count;
    const int    half_rays = static_cast<int>(std::floor(0.5 * camera_.horizontal_fov / ray_angle));
    const double tan_half_h = std::tan(0.5 * camera_.horizontal_fov);
    // Horizontal image offset of each ray relative to the heading, shared by all headings.
    std::vector<float> rho_h(static_cast<std::size_t>((2 * half_rays) + 1));
    for (int offset = -half_rays; offset <= half_rays; ++offset)
    {
        const int slot = offset + half_rays;
        rho_h[static_cast<std::size_t>(slot)] =
            static_cast<float>(std::tan(offset * ray_angle) / tan_half_h);
    }
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

    std::vector<Hit> hits;
    std::vector<int> offsets;
    double           best_utility = 0.0;
    double           best_gain    = 0.0;
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
        hits.clear();
        castRays(coverage, x, y, hits, offsets);
        if (hits.empty())
        {
            continue;
        }

        // Gain of heading j over targets no chosen heading covers yet.
        const auto heading_gain = [&](int heading, bool commit, std::vector<int>* seen) {
            ++heading_value_;
            double gain = 0.0;
            for (int offset = -half_rays; offset <= half_rays; ++offset)
            {
                const int ray =
                    ((heading * per_heading) + offset + params_.ray_count) % params_.ray_count;
                const float horizontal = std::abs(
                    rho_h[static_cast<std::size_t>(offset) + static_cast<std::size_t>(half_rays)]);
                for (int h = offsets[static_cast<std::size_t>(ray)];
                     h < offsets[static_cast<std::size_t>(ray) + 1];
                     ++h)
                {
                    const Hit&  hit  = hits[static_cast<std::size_t>(h)];
                    const float rho  = std::max(horizontal, std::abs(hit.rho_v));
                    const float edge = 1.0F - (rho * rho * rho * rho);
                    if (hit.base * edge < well_seen)
                    {
                        continue;
                    }
                    const auto target     = static_cast<std::size_t>(hit.target);
                    predicted_any[target] = 1;
                    if (chosen_mark_[target] == chosen_value_ ||
                        heading_mark_[target] == heading_value_)
                    {
                        continue;
                    }
                    heading_mark_[target] = heading_value_;
                    gain += weight(hit.kind);
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
        if (utility > best_utility)
        {
            best_utility             = utility;
            plan.status              = PlanStatus::kViewpoint;
            plan.viewpoint.x         = x;
            plan.viewpoint.y         = y;
            plan.viewpoint.headings  = headings;
            plan.viewpoint.room      = room;
            plan.viewpoint.gain      = gain;
            plan.viewpoint.cost      = cost;
            plan.viewpoint.predicted = seen;
        }
    }

    if (best_gain < params_.min_viewpoint_gain)
    {
        // Any heading can centre a ray, so only its vertical offset counts.
        for (const Candidate& candidate : rest)
        {
            hits.clear();
            castRays(
                coverage,
                geometry.centreX(candidate.index % geometry.width),
                geometry.centreY(candidate.index / geometry.width),
                hits,
                offsets);
            for (const Hit& hit : hits)
            {
                const float rho = std::abs(hit.rho_v);
                if (hit.base * (1.0F - (rho * rho * rho * rho)) >= well_seen)
                {
                    predicted_any[static_cast<std::size_t>(hit.target)] = 1;
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

    plan.viewpoint.id = next_id_++;
    issued_.push_back(plan.viewpoint);
    // Only the recent past matters for reports; keep the list short.
    if (issued_.size() > 16)
    {
        issued_.erase(issued_.begin());
    }
    return plan;
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
