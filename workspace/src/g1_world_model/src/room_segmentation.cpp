/**
 * @file room_segmentation.cpp
 * @brief Persistence of clearance peaks, region merging, outlines and contacts.
 */

#include "g1_world_model/room_segmentation.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace g1_world_model
{

namespace
{

constexpr std::array<std::array<int, 2>, 4> kNeighbours4{
    { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } }
};

/// Distance from each free cell to the nearest cell that is not free, m. Unknown counts as an
/// obstacle: a room is only as big as the part of it that has been seen.
cv::Mat clearanceOf(const cv::Mat& cells, double resolution)
{
    cv::Mat free_mask = (cells == kFree);
    cv::Mat distance;
    cv::distanceTransform(free_mask, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    distance *= resolution;
    return distance;
}

/**
 * Labels each free cell with the persistent clearance peak it climbs to. Components and their
 * persistence come from union-find; ownership follows steepest ascent, so a cell by a wall
 * belongs to the room it is in even when that room merged with a neighbour at a higher level.
 * Returns the number of rooms.
 */
int persistenceLabels(
    const cv::Mat& cells, const cv::Mat& clearance, const RoomSegmentationParams& params,
    cv::Mat& labels)
{
    const int         cols      = cells.cols;
    const int         rows      = cells.rows;
    const std::size_t total     = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
    const auto*       occupancy = cells.ptr<std::uint8_t>(0);
    const auto*       height    = clearance.ptr<float>(0);

    std::vector<int> order;
    order.reserve(total);
    for (int index = 0; index < rows * cols; ++index)
    {
        if (occupancy[index] == kFree)
        {
            order.push_back(index);
        }
    }
    // Ties by index keep the result independent of the sort's stability.
    std::sort(order.begin(), order.end(), [height](int a, int b) {
        return height[a] != height[b] ? height[a] > height[b] : a < b;
    });

    struct Peak
    {
        float birth;
        float death       = 0.0F;
        int   absorbed_by = -1;
    };
    std::vector<Peak> peaks;
    std::vector<int>  parent(total, -1);   // Union-find over added cells; -1 not yet added.
    std::vector<int>  peak_at(total, -1);  // The peak a root cell stands for.
    std::vector<int>  owner(total, -1);    // The peak a cell climbs to.
    const auto        find = [&parent](int cell) {
        while (parent[static_cast<std::size_t>(cell)] != cell)
        {
            auto& up = parent[static_cast<std::size_t>(cell)];
            up       = parent[static_cast<std::size_t>(up)];
            cell     = up;
        }
        return cell;
    };

    for (const int cell : order)
    {
        const auto slot = static_cast<std::size_t>(cell);
        const int  x    = cell % cols;
        const int  y    = cell / cols;

        std::array<int, 4> roots{};
        std::size_t        root_count = 0;
        int                uphill     = -1;
        std::array<int, 4> neighbours{};
        std::array<int, 4> neighbour_roots{};
        std::size_t        neighbour_count = 0;
        for (const auto& [dx, dy] : kNeighbours4)
        {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= cols || ny >= rows ||
                parent
                        [(static_cast<std::size_t>(ny) * static_cast<std::size_t>(cols)) +
                         static_cast<std::size_t>(nx)] < 0)
            {
                continue;
            }
            const int neighbour = (ny * cols) + nx;
            if (uphill < 0 || height[neighbour] > height[uphill] ||
                (height[neighbour] == height[uphill] && neighbour < uphill))
            {
                uphill = neighbour;
            }
            const int root                     = find(neighbour);
            neighbours[neighbour_count]        = neighbour;
            neighbour_roots[neighbour_count++] = root;
            if (std::find(
                    roots.begin(),
                    roots.begin() + static_cast<std::ptrdiff_t>(root_count),
                    root) == roots.begin() + static_cast<std::ptrdiff_t>(root_count))
            {
                roots[root_count++] = root;
            }
        }

        if (root_count == 0)
        {
            parent[slot]  = cell;
            peak_at[slot] = static_cast<int>(peaks.size());
            owner[slot]   = peak_at[slot];
            peaks.push_back({ height[cell] });
            continue;
        }

        // The elder rule: the component with the higher peak survives the meeting.
        int elder = roots[0];
        for (std::size_t i = 1; i < root_count; ++i)
        {
            if (peaks[static_cast<std::size_t>(peak_at[static_cast<std::size_t>(roots[i])])].birth >
                peaks[static_cast<std::size_t>(peak_at[static_cast<std::size_t>(elder)])].birth)
            {
                elder = roots[i];
            }
        }
        parent[slot] = elder;
        owner[slot]  = owner[static_cast<std::size_t>(uphill)];
        for (std::size_t i = 0; i < root_count; ++i)
        {
            if (roots[i] == elder)
            {
                continue;
            }
            // A dropped peak joins the region it touches here, not whichever component its
            // neighbour had merged into higher up: that could be across a doorway.
            int across = -1;
            for (std::size_t n = 0; n < neighbour_count; ++n)
            {
                if (neighbour_roots[n] != roots[i] &&
                    (across < 0 || height[neighbours[n]] > height[across]))
                {
                    across = neighbours[n];
                }
            }
            Peak& younger =
                peaks[static_cast<std::size_t>(peak_at[static_cast<std::size_t>(roots[i])])];
            younger.death                              = height[cell];
            younger.absorbed_by                        = owner[static_cast<std::size_t>(across)];
            parent[static_cast<std::size_t>(roots[i])] = elder;
        }
    }

    std::vector<int> label_of(peaks.size(), 0);
    int              count = 0;
    for (std::size_t peak = 0; peak < peaks.size(); ++peak)
    {
        const Peak& candidate = peaks[peak];
        const bool  doorway   = candidate.death <= 0.5 * params.max_doorway_width ||
                             candidate.death <= params.max_pinch_ratio * candidate.birth;
        if (candidate.birth >= params.min_peak &&
            candidate.birth - candidate.death >= params.min_persistence && doorway)
        {
            label_of[peak] = ++count;
        }
    }
    // A dropped peak joins the first kept peak along its chain. Chains point at components that
    // were already alive, so they end; the step cap only guards against a logic error.
    for (std::size_t peak = 0; peak < peaks.size(); ++peak)
    {
        int         next  = static_cast<int>(peak);
        std::size_t steps = 0;
        while (next >= 0 && label_of[static_cast<std::size_t>(next)] == 0 && steps++ < peaks.size())
        {
            next = peaks[static_cast<std::size_t>(next)].absorbed_by;
        }
        if (next >= 0)
        {
            label_of[peak] = label_of[static_cast<std::size_t>(next)];
        }
    }

    labels   = cv::Mat::zeros(rows, cols, CV_32S);
    int* out = labels.ptr<int>(0);
    for (const int cell : order)
    {
        out[cell] = label_of[static_cast<std::size_t>(owner[static_cast<std::size_t>(cell)])];
    }
    return count;
}

/// Merges regions under @p min_cells into the neighbour they share most border with.
void mergeSmallRegions(cv::Mat& labels, int count, int min_cells)
{
    const int rows = labels.rows;
    const int cols = labels.cols;
    for (;;)
    {
        std::vector<int>                      area(static_cast<std::size_t>(count) + 1, 0);
        std::unordered_map<std::int64_t, int> border;
        const auto                            key = [count](int a, int b) {
            return (static_cast<std::int64_t>(a) * (count + 1)) + b;
        };
        for (int y = 0; y < rows; ++y)
        {
            const int* row = labels.ptr<int>(y);
            const int* up  = y + 1 < rows ? labels.ptr<int>(y + 1) : nullptr;
            for (int x = 0; x < cols; ++x)
            {
                const int a = row[x];
                ++area[static_cast<std::size_t>(a)];
                if (a == 0)
                {
                    continue;
                }
                if (x + 1 < cols && row[x + 1] > 0 && row[x + 1] != a)
                {
                    ++border[key(a, row[x + 1])];
                    ++border[key(row[x + 1], a)];
                }
                if (up != nullptr && up[x] > 0 && up[x] != a)
                {
                    ++border[key(a, up[x])];
                    ++border[key(up[x], a)];
                }
            }
        }

        int smallest = 0;
        for (int label = 1; label <= count; ++label)
        {
            const auto label_area = area[static_cast<std::size_t>(label)];
            if (label_area > 0 && label_area < min_cells &&
                (smallest == 0 || label_area < area[static_cast<std::size_t>(smallest)]))
            {
                smallest = label;
            }
        }
        if (smallest == 0)
        {
            return;
        }
        int into        = 0;
        int into_border = 0;
        for (int other = 1; other <= count; ++other)
        {
            const auto found = border.find(key(smallest, other));
            if (found != border.end() && found->second > into_border)
            {
                into        = other;
                into_border = found->second;
            }
        }
        // A pocket that touches no other region is clutter, not a room.
        labels.setTo(into, labels == smallest);
    }
}

/// Splits off the corridors: stretches of free space no disc of the corridor width fits in,
/// when they are long and thin. Over all free space at once, since a hallway can have been
/// claimed piecewise by the rooms along it. Returns the new label count.
int splitCorridors(
    cv::Mat& labels, int count, const cv::Mat& clearance, const GridGeometry& geometry,
    const RoomSegmentationParams& params)
{
    const double  radius = 0.5 * params.corridor_width;
    const int     reach  = std::max(1, geometry.cellsFor(radius));
    const cv::Mat disc =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size((2 * reach) + 1, (2 * reach) + 1));
    const double cell_area = geometry.resolution * geometry.resolution;

    const cv::Mat free_space = labels > 0;
    const cv::Mat centres    = free_space & (clearance >= radius);
    cv::Mat       wide;
    cv::dilate(centres, wide, disc);
    const cv::Mat narrow = free_space & ~wide;

    cv::Mat   parts;
    cv::Mat   stats;
    cv::Mat   centroids;
    const int found = cv::connectedComponentsWithStats(narrow, parts, stats, centroids, 4, CV_32S);
    int       next  = count + 1;
    for (int part = 1; part < found; ++part)
    {
        if (stats.at<int>(part, cv::CC_STAT_AREA) * cell_area < params.min_corridor_area)
        {
            continue;
        }
        const cv::Mat          mask = parts == part;
        std::vector<cv::Point> points;
        cv::findNonZero(mask, points);
        const cv::RotatedRect box    = cv::minAreaRect(points);
        const double          length = std::max(box.size.width, box.size.height);
        const double          width  = std::max(1.0F, std::min(box.size.width, box.size.height));
        if (length / width < params.corridor_aspect)
        {
            continue;
        }
        // A region the corridor takes most of was the corridor already: it goes with it whole.
        std::vector<int> taken(static_cast<std::size_t>(next), 0);
        std::vector<int> area(static_cast<std::size_t>(next), 0);
        for (int y = 0; y < labels.rows; ++y)
        {
            const auto* row      = labels.ptr<int>(y);
            const auto* part_row = mask.ptr<std::uint8_t>(y);
            for (int x = 0; x < labels.cols; ++x)
            {
                if (row[x] > 0 && row[x] < next)
                {
                    ++area[static_cast<std::size_t>(row[x])];
                    taken[static_cast<std::size_t>(row[x])] += static_cast<int>(part_row[x] != 0);
                }
            }
        }
        const int corridor = next++;
        labels.setTo(corridor, mask);
        for (int label = 1; label < corridor; ++label)
        {
            const auto slot = static_cast<std::size_t>(label);
            if (area[slot] > 0 && taken[slot] * 10 >= area[slot] * 7)
            {
                labels.setTo(corridor, labels == label);
            }
        }
    }
    return next - 1;
}

/// Renumbers labels 1..n in scan order, dropping empty ones. Returns n.
int compactLabels(cv::Mat& labels, int count)
{
    std::vector<int> relabel(static_cast<std::size_t>(count) + 1, 0);
    int              next = 0;
    for (int y = 0; y < labels.rows; ++y)
    {
        int* row = labels.ptr<int>(y);
        for (int x = 0; x < labels.cols; ++x)
        {
            const int value = row[x];
            if (value == 0)
            {
                continue;
            }
            int& mapped = relabel[static_cast<std::size_t>(value)];
            if (mapped == 0)
            {
                mapped = ++next;
            }
            row[x] = mapped;
        }
    }
    return next;
}

std::vector<cv::Point2d>
outlineOf(const cv::Mat& labels, int label, const GridGeometry& geometry, double tolerance)
{
    const cv::Mat                       mask = labels == label;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
    {
        return {};
    }
    const auto largest =
        std::max_element(contours.begin(), contours.end(), [](const auto& a, const auto& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });
    std::vector<cv::Point> simplified;
    cv::approxPolyDP(*largest, simplified, tolerance / geometry.resolution, true);

    std::vector<cv::Point2d> outline;
    outline.reserve(simplified.size());
    for (const cv::Point& point : simplified)
    {
        outline.emplace_back(geometry.centreX(point.x), geometry.centreY(point.y));
    }
    double twice_area = 0.0;
    for (std::size_t i = 0; i < outline.size(); ++i)
    {
        const cv::Point2d& a = outline[i];
        const cv::Point2d& b = outline[(i + 1) % outline.size()];
        twice_area += (a.x * b.y) - (b.x * a.y);
    }
    if (twice_area < 0.0)
    {
        std::reverse(outline.begin(), outline.end());
    }
    return outline;
}

/// Contacts between regions: each connected run of cells where two labels touch.
void findContacts(
    const cv::Mat& labels, const cv::Mat& clearance, const GridGeometry& geometry, double min_width,
    std::vector<Region>& regions)
{
    const int rows = labels.rows;
    const int cols = labels.cols;

    // Boundary cells per ordered pair (a < b), both sides included.
    std::unordered_map<std::int64_t, std::vector<int>> boundaries;
    const auto                                         key = [](int a, int b) {
        return (static_cast<std::int64_t>(std::min(a, b)) << 32) | std::max(a, b);
    };
    for (int y = 0; y < rows; ++y)
    {
        const int* row = labels.ptr<int>(y);
        for (int x = 0; x < cols; ++x)
        {
            const int a = row[x];
            if (a == 0)
            {
                continue;
            }
            for (const auto& [dx, dy] : kNeighbours4)
            {
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= cols || ny >= rows)
                {
                    continue;
                }
                const int b = labels.at<int>(ny, nx);
                if (b > 0 && b != a)
                {
                    boundaries[key(a, b)].push_back((y * cols) + x);
                }
            }
        }
    }

    for (auto& [pair, cells] : boundaries)
    {
        const int a = static_cast<int>(pair >> 32);
        const int b = static_cast<int>(pair & 0xFFFFFFFF);
        std::sort(cells.begin(), cells.end());
        cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
        std::unordered_set<int> pending(cells.begin(), cells.end());

        while (!pending.empty())
        {
            // Single linkage over a few cells: a jagged boundary through one doorway is one run.
            constexpr int    kLink = 3;
            std::vector<int> run{ *pending.begin() };
            pending.erase(pending.begin());
            for (std::size_t head = 0; head < run.size(); ++head)
            {
                const int x = run[head] % cols;
                const int y = run[head] / cols;
                for (int dy = -kLink; dy <= kLink; ++dy)
                {
                    for (int dx = -kLink; dx <= kLink; ++dx)
                    {
                        if (x + dx < 0 || x + dx >= cols || y + dy < 0 || y + dy >= rows)
                        {
                            continue;
                        }
                        const auto found = pending.find(((y + dy) * cols) + x + dx);
                        if (found != pending.end())
                        {
                            run.push_back(*found);
                            pending.erase(found);
                        }
                    }
                }
            }

            double sum_x         = 0.0;
            double sum_y         = 0.0;
            double max_clearance = 0.0;
            for (const int index : run)
            {
                sum_x += geometry.centreX(index % cols);
                sum_y += geometry.centreY(index / cols);
                max_clearance =
                    std::max(max_clearance, static_cast<double>(clearance.ptr<float>(0)[index]));
            }
            // The clearance at the middle of an opening is half its width.
            const double width = 2.0 * max_clearance;
            if (width < min_width)
            {
                continue;
            }
            const auto count = static_cast<double>(run.size());
            regions[static_cast<std::size_t>(a - 1)].contacts.push_back(
                { b, sum_x / count, sum_y / count, width });
            regions[static_cast<std::size_t>(b - 1)].contacts.push_back(
                { a, sum_x / count, sum_y / count, width });
        }
    }
    for (Region& region : regions)
    {
        std::sort(region.contacts.begin(), region.contacts.end(), [](const auto& l, const auto& r) {
            return l.to_label != r.to_label ? l.to_label < r.to_label : l.x < r.x;
        });
    }
}

}  // namespace

Segmentation segmentRooms(
    const cv::Mat& cells, const GridGeometry& geometry, const RoomSegmentationParams& params)
{
    Segmentation result;
    const double cell_area      = geometry.resolution * geometry.resolution;
    const int    min_room_cells = std::max(1, static_cast<int>(params.min_room_area / cell_area));

    const cv::Mat clearance = clearanceOf(cells, geometry.resolution);
    int           count     = persistenceLabels(cells, clearance, params, result.labels);
    mergeSmallRegions(result.labels, count, min_room_cells);
    count = splitCorridors(result.labels, count, clearance, geometry, params);
    count = compactLabels(result.labels, count);

    result.regions.resize(static_cast<std::size_t>(count));
    std::vector<double> sum_x(result.regions.size(), 0.0);
    std::vector<double> sum_y(result.regions.size(), 0.0);
    std::vector<double> sum_clearance(result.regions.size(), 0.0);
    std::vector<int>    cell_count(result.regions.size(), 0);
    for (int y = 0; y < cells.rows; ++y)
    {
        const int*  row      = result.labels.ptr<int>(y);
        const auto* distance = clearance.ptr<float>(y);
        for (int x = 0; x < cells.cols; ++x)
        {
            if (row[x] == 0)
            {
                continue;
            }
            const auto slot = static_cast<std::size_t>(row[x] - 1);
            sum_x[slot] += geometry.centreX(x);
            sum_y[slot] += geometry.centreY(y);
            sum_clearance[slot] += distance[x];
            ++cell_count[slot];
        }
    }
    for (std::size_t slot = 0; slot < result.regions.size(); ++slot)
    {
        Region&    region     = result.regions[slot];
        const auto cells_in   = static_cast<double>(std::max(cell_count[slot], 1));
        region.label          = static_cast<int>(slot) + 1;
        region.area           = cell_count[slot] * cell_area;
        region.centroid_x     = sum_x[slot] / cells_in;
        region.centroid_y     = sum_y[slot] / cells_in;
        region.mean_clearance = sum_clearance[slot] / cells_in;
        region.outline = outlineOf(result.labels, region.label, geometry, params.outline_tolerance);

        std::vector<cv::Point2f> points(region.outline.begin(), region.outline.end());
        if (points.size() >= 3)
        {
            const cv::RotatedRect box = cv::minAreaRect(points);
            region.length             = std::max(box.size.width, box.size.height);
            region.width              = std::min(box.size.width, box.size.height);
        }
    }
    findContacts(result.labels, clearance, geometry, params.min_doorway_width, result.regions);
    return result;
}

std::vector<int> matchRegions(
    const cv::Mat& previous, int previous_count, const cv::Mat& current, int current_count,
    double min_iou)
{
    std::vector<int> match(static_cast<std::size_t>(current_count), 0);
    if (previous.empty() || previous.size() != current.size() || previous_count == 0)
    {
        return match;
    }

    const auto       stride = static_cast<std::size_t>(current_count) + 1;
    std::vector<int> overlap((static_cast<std::size_t>(previous_count) + 1) * stride, 0);
    std::vector<int> previous_area(static_cast<std::size_t>(previous_count) + 1, 0);
    std::vector<int> current_area(stride, 0);
    for (int y = 0; y < current.rows; ++y)
    {
        const int* before = previous.ptr<int>(y);
        const int* after  = current.ptr<int>(y);
        for (int x = 0; x < current.cols; ++x)
        {
            const auto p = static_cast<std::size_t>(std::clamp(before[x], 0, previous_count));
            const auto c = static_cast<std::size_t>(std::clamp(after[x], 0, current_count));
            ++previous_area[p];
            ++current_area[c];
            ++overlap[(p * stride) + c];
        }
    }

    struct Pair
    {
        double iou;
        int    previous_label;
        int    current_label;
    };
    std::vector<Pair> pairs;
    for (int p = 1; p <= previous_count; ++p)
    {
        for (int c = 1; c <= current_count; ++c)
        {
            const int shared =
                overlap[(static_cast<std::size_t>(p) * stride) + static_cast<std::size_t>(c)];
            if (shared == 0)
            {
                continue;
            }
            const int unite = previous_area[static_cast<std::size_t>(p)] +
                              current_area[static_cast<std::size_t>(c)] - shared;
            const double iou = static_cast<double>(shared) / unite;
            if (iou >= min_iou)
            {
                pairs.push_back({ iou, p, c });
            }
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
        return a.iou > b.iou;
    });
    std::vector<bool> taken(static_cast<std::size_t>(previous_count) + 1, false);
    for (const Pair& pair : pairs)
    {
        auto& slot = match[static_cast<std::size_t>(pair.current_label - 1)];
        if (slot != 0 || taken[static_cast<std::size_t>(pair.previous_label)])
        {
            continue;
        }
        slot                                                 = pair.previous_label;
        taken[static_cast<std::size_t>(pair.previous_label)] = true;
    }
    return match;
}

cv::Mat resampleLabels(const cv::Mat& labels, const GridGeometry& from, const GridGeometry& to)
{
    cv::Mat out = cv::Mat::zeros(to.height, to.width, CV_32S);
    if (labels.empty())
    {
        return out;
    }
    for (int y = 0; y < to.height; ++y)
    {
        int* row = out.ptr<int>(y);
        for (int x = 0; x < to.width; ++x)
        {
            const CellIndex source = from.toCell(to.centreX(x), to.centreY(y));
            if (from.contains(source))
            {
                row[x] = labels.at<int>(source.y, source.x);
            }
        }
    }
    return out;
}

}  // namespace g1_world_model
