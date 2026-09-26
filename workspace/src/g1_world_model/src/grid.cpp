/**
 * @file grid.cpp
 * @brief Occupancy classification and grid fingerprints.
 */

#include "g1_world_model/grid.hpp"

#include <cmath>
#include <cstdint>
#include <numbers>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

namespace g1_world_model
{

namespace
{

/// Cells from its edge to its middle, at most, for a pocket to be a crack: a beam gap in a wall.
constexpr float kCrackDepth = 2.0F;

/// How sharply occupied cells stack along the axes turned by @p yaw: sum of squared counts.
double profileSharpness(const std::vector<cv::Point2f>& points, double yaw, int span)
{
    const auto       c = static_cast<float>(std::cos(yaw));
    const auto       s = static_cast<float>(std::sin(yaw));
    std::vector<int> along(static_cast<std::size_t>(2 * span) + 1, 0);
    std::vector<int> across(along.size(), 0);
    for (const cv::Point2f& p : points)
    {
        const auto u = static_cast<int>(std::lround((p.x * c) + (p.y * s))) + span;
        const auto v = static_cast<int>(std::lround((p.y * c) - (p.x * s))) + span;
        ++along[static_cast<std::size_t>(u)];
        ++across[static_cast<std::size_t>(v)];
    }
    double sharpness = 0.0;
    for (std::size_t i = 0; i < along.size(); ++i)
    {
        sharpness += (static_cast<double>(along[i]) * along[i]) +
                     (static_cast<double>(across[i]) * across[i]);
    }
    return sharpness;
}

}  // namespace

double dominantAxis(const cv::Mat& cells)
{
    // Walls project onto a few sharp peaks only along their own directions: the deskew search.
    std::vector<cv::Point2f> points;
    for (int y = 0; y < cells.rows; ++y)
    {
        const auto* row = cells.ptr<std::uint8_t>(y);
        for (int x = 0; x < cells.cols; ++x)
        {
            if (row[x] == kOccupied)
            {
                points.emplace_back(static_cast<float>(x), static_cast<float>(y));
            }
        }
    }
    if (points.empty())
    {
        return 0.0;
    }
    const int    span   = cells.rows + cells.cols;
    const double degree = std::numbers::pi / 180.0;
    const auto   search = [&](double from, double step, int count) {
        double best_yaw   = from;
        double best_score = -1.0;
        for (int i = 0; i < count; ++i)
        {
            const double yaw   = from + (i * step);
            const double score = profileSharpness(points, yaw, span);
            if (score > best_score)
            {
                best_score = score;
                best_yaw   = yaw;
            }
        }
        return best_yaw;
    };
    const double coarse  = search(0.0, degree, 90);
    const double fine    = search(coarse - degree, 0.1 * degree, 21);
    const double quarter = std::numbers::pi / 2.0;
    return std::fmod(fine + quarter, quarter);
}

cv::Mat settleEnclosedUnknown(const cv::Mat& cells, const cv::Mat& furniture)
{
    cv::Mat   labels;
    cv::Mat   stats;
    cv::Mat   centroids;
    const int count =
        cv::connectedComponentsWithStats(cells == kUnknown, labels, stats, centroids, 4, CV_32S);
    // Per pocket: what it borders, and how far its middle lies from its edge.
    std::vector<std::uint8_t> solid(static_cast<std::size_t>(count), 0);
    std::vector<std::uint8_t> floor(static_cast<std::size_t>(count), 0);
    std::vector<float>        depth(static_cast<std::size_t>(count), 0.0F);
    cv::Mat                   inset;
    cv::distanceTransform(cells == kUnknown, inset, cv::DIST_L1, 3);
    for (int y = 0; y < cells.rows; ++y)
    {
        const int*   row  = labels.ptr<int>(y);
        const float* away = inset.ptr<float>(y);
        for (int x = 0; x < cells.cols; ++x)
        {
            const auto label = static_cast<std::size_t>(row[x]);
            if (label == 0)
            {
                continue;
            }
            depth[label] = std::max(depth[label], away[x]);
            for (const auto& [nx, ny] :
                 { std::pair{ x + 1, y }, { x - 1, y }, { x, y + 1 }, { x, y - 1 } })
            {
                if (nx >= 0 && ny >= 0 && nx < cells.cols && ny < cells.rows)
                {
                    const std::uint8_t next = cells.at<std::uint8_t>(ny, nx);
                    solid[label] |= static_cast<std::uint8_t>(next == kOccupied);
                    floor[label] |= static_cast<std::uint8_t>(next == kFree);
                }
            }
        }
    }
    const bool hinted  = !furniture.empty() && furniture.size() == cells.size();
    cv::Mat    settled = cells.clone();
    for (int y = 0; y < cells.rows; ++y)
    {
        const int* row  = labels.ptr<int>(y);
        auto*      cell = settled.ptr<std::uint8_t>(y);
        for (int x = 0; x < cells.cols; ++x)
        {
            const auto label = static_cast<std::size_t>(row[x]);
            if (label == 0)
            {
                continue;
            }
            const int* box     = stats.ptr<int>(static_cast<int>(label));
            const bool outside = box[cv::CC_STAT_LEFT] == 0 || box[cv::CC_STAT_TOP] == 0 ||
                                 box[cv::CC_STAT_LEFT] + box[cv::CC_STAT_WIDTH] == cells.cols ||
                                 box[cv::CC_STAT_TOP] + box[cv::CC_STAT_HEIGHT] == cells.rows;
            if (outside)
            {
                continue;
            }
            // A wide pocket open to the floor is the floor no beam reached behind a wardrobe, or
            // the underside of an armchair: solid only where an object stands.
            const bool solid_here =
                solid[label] != 0 && (floor[label] == 0 || depth[label] <= kCrackDepth ||
                                      (hinted && furniture.at<std::uint8_t>(y, x) != 0));
            cell[x] = solid_here ? kOccupied : kFree;
        }
    }
    return settled;
}

cv::Mat completeMap(
    const cv::Mat& cells, const cv::Mat& walls, double yaw, const PlanGaps& limits,
    const cv::Mat& furniture)
{
    cv::Mat closed = cells.clone();
    closed.setTo(kOccupied, (walls != 0) & (cells == kUnknown));

    // In the walls' own frame each wall is a row or a column of the grid. The canvas holds the
    // whole turned grid: one its own size cuts the corners off, and at 80-odd degrees both ends.
    const cv::Point2f centre(
        0.5F * static_cast<float>(cells.cols),
        0.5F * static_cast<float>(cells.rows));
    const double     degrees  = yaw * 180.0 / std::numbers::pi;
    cv::Mat          to_walls = cv::getRotationMatrix2D(centre, degrees, 1.0);
    const cv::Rect2f bounds =
        cv::RotatedRect(centre, cv::Size2f(cells.size()), static_cast<float>(degrees))
            .boundingRect2f();
    to_walls.at<double>(0, 2) += (0.5 * bounds.width) - centre.x;
    to_walls.at<double>(1, 2) += (0.5 * bounds.height) - centre.y;
    cv::Mat turned;
    cv::warpAffine(
        closed,
        turned,
        to_walls,
        cv::Size(
            static_cast<int>(std::ceil(bounds.width)),
            static_cast<int>(std::ceil(bounds.height))),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar(kUnknown));
    cv::Mat gaps(turned.size(), CV_8UC1, cv::Scalar(0));
    // A gap flanked by a straight run of solid on both sides is a wall carried on behind
    // furniture; any other closes only across one piece of furniture's depth.
    const auto close_line = [&](int count, const auto& at, const auto& mark) {
        int last   = -1;
        int behind = 0;
        int run    = 0;
        for (int i = 0; i < count; ++i)
        {
            const std::uint8_t cell = at(i);
            if (cell != kOccupied)
            {
                last = cell == kFree ? -1 : last;
                run  = 0;
                continue;
            }
            ++run;
            if (last >= 0 && i - last > 1)
            {
                int ahead = 0;
                while (i + ahead < count && at(i + ahead) == kOccupied)
                {
                    ++ahead;
                }
                const bool wall = behind >= limits.wall_run && ahead >= limits.wall_run;
                if (i - last - 1 <= (wall ? limits.wall : limits.furniture))
                {
                    for (int j = last + 1; j < i; ++j)
                    {
                        mark(j);
                    }
                }
            }
            last   = i;
            behind = run;
        }
    };
    for (int y = 0; y < turned.rows; ++y)
    {
        close_line(
            turned.cols,
            [&](int x) { return turned.at<std::uint8_t>(y, x); },
            [&](int x) { gaps.at<std::uint8_t>(y, x) = 255; });
    }
    for (int x = 0; x < turned.cols; ++x)
    {
        close_line(
            turned.rows,
            [&](int y) { return turned.at<std::uint8_t>(y, x); },
            [&](int y) { gaps.at<std::uint8_t>(y, x) = 255; });
    }
    cv::Mat back;
    cv::warpAffine(
        gaps,
        back,
        to_walls,
        closed.size(),
        cv::INTER_NEAREST | cv::WARP_INVERSE_MAP,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));
    closed.setTo(kOccupied, (back != 0) & (closed == kUnknown));
    return settleEnclosedUnknown(closed, furniture);
}

cv::Mat classifyOccupancy(
    const std::int8_t* data, const GridGeometry& geometry, int free_below, int occupied_above)
{
    cv::Mat           cells(geometry.height, geometry.width, CV_8UC1);
    const std::size_t count = geometry.cellCount();
    auto*             out   = cells.ptr<std::uint8_t>(0);
    for (std::size_t i = 0; i < count; ++i)
    {
        // Signed by definition: -1 is unknown.
        const auto value = static_cast<int>(data[i]);  // NOLINT(bugprone-signed-char-misuse)
        if (value < 0)
        {
            out[i] = kUnknown;
        }
        else if (value >= occupied_above)
        {
            out[i] = kOccupied;
        }
        else
        {
            out[i] = value <= free_below ? kFree : kUnknown;
        }
    }
    return cells;
}

}  // namespace g1_world_model
