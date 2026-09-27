/**
 * @file coverage_map.cpp
 * @brief Target extraction, depth integration and per-room tallies.
 */

#include "canopy/coverage_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <opencv2/imgproc.hpp>

namespace canopy
{

namespace
{

float incidenceLimitCos(double limit) { return static_cast<float>(std::cos(limit)); }

constexpr float kSameFacing = 0.7F;  // Cosine between two faces' normals: within 45 degrees.

/// Sector 0..7 of an azimuth, counter-clockwise from +x.
std::uint8_t sectorOf(double dx, double dy)
{
    const double angle = std::atan2(dy, dx) + std::numbers::pi;  // 0..2 pi
    const int    sector =
        static_cast<int>(std::floor(angle / (std::numbers::pi / 4.0))) % 8;  // NOLINT
    return static_cast<std::uint8_t>(1U << static_cast<unsigned>(sector));
}

}  // namespace

CoverageMap::CoverageMap(CoverageParams params)
  : params_(params)
{}

void CoverageMap::setMap(const cv::Mat& cells, const GridGeometry& geometry)
{
    const bool        same_grid = geometry == geometry_ && !cells_.empty();
    const std::size_t count     = geometry.cellCount();
    // The faces before this map, on its grid, for a redrawn wall to take its credit from.
    std::vector<std::uint8_t> old_kind   = kind_;
    std::vector<cv::Vec2f>    old_normal = normal_;
    if (!same_grid)
    {
        // SLAM grows and shifts its map every few seconds: what was seen stays seen.
        const GridGeometry from = cells_.empty() ? GridGeometry{} : geometry_;
        old_kind                = remapLayer<std::uint8_t>(kind_, from, geometry, 0);
        old_normal       = remapLayer<cv::Vec2f>(normal_, from, geometry, cv::Vec2f(0.0F, 0.0F));
        quality_         = remapLayer<std::uint8_t>(quality_, from, geometry, 0);
        surface_quality_ = remapLayer<std::uint8_t>(surface_quality_, from, geometry, 0);
        flags_           = remapLayer<std::uint8_t>(flags_, from, geometry, 0);
        directions_      = remapLayer<std::uint8_t>(directions_, from, geometry, 0);
        views_           = remapLayer<std::uint16_t>(views_, from, geometry, 0);
        last_frame_      = remapLayer<std::uint32_t>(last_frame_, from, geometry, 0);
        surface_at_.assign(count, -1);
        surface_height_.clear();
    }
    geometry_ = geometry;
    cells_    = cells.clone();
    kind_.assign(count, static_cast<std::uint8_t>(TargetKind::kNone));
    normal_.assign(count, cv::Vec2f(0.0F, 0.0F));

    const int cols = cells.cols;
    const int rows = cells.rows;
    for (int y = 0; y < rows; ++y)
    {
        const std::uint8_t* row = cells_.ptr<std::uint8_t>(y);
        for (int x = 0; x < cols; ++x)
        {
            const auto index = static_cast<std::size_t>(geometry.index(x, y));
            if (row[x] == kFree)
            {
                kind_[index] = static_cast<std::uint8_t>(TargetKind::kFloor);
                continue;
            }
            if (row[x] != kOccupied)
            {
                continue;
            }
            // A face is an obstacle cell with free space beside it; its normal points there.
            float nx          = 0.0F;
            float ny          = 0.0F;
            bool  beside_free = false;
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if ((dx == 0 && dy == 0) || !geometry.contains(x + dx, y + dy) ||
                        cells_.at<std::uint8_t>(y + dy, x + dx) != kFree)
                    {
                        continue;
                    }
                    const float length = std::hypot(static_cast<float>(dx), static_cast<float>(dy));
                    nx += static_cast<float>(dx) / length;
                    ny += static_cast<float>(dy) / length;
                    beside_free = beside_free || dx == 0 || dy == 0;
                }
            }
            const float norm = std::hypot(nx, ny);
            if (beside_free && norm > 1e-3F)
            {
                kind_[index]   = static_cast<std::uint8_t>(TargetKind::kFace);
                normal_[index] = cv::Vec2f(nx / norm, ny / norm);
            }
        }
    }
    rebuildNearestFaces();

    // After a loop closure SLAM redraws walls a cell or two off, and a wall drawn further back
    // puts its faces on cells that were inside it and never seen: each takes the credit of an old
    // face beside it facing the same way. Run 45 lost a fifth of every finished room's faces in
    // one redraw, and walked back to each.
    const int reach = std::max(1, geometry.cellsFor(params_.face_redraw_reach));
    const std::vector<std::uint8_t> before = quality_;
    const auto                      face   = static_cast<std::uint8_t>(TargetKind::kFace);
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            const auto index = static_cast<std::size_t>(geometry.index(x, y));
            if (kind_[index] != face || old_kind[index] == face)
            {
                continue;
            }
            for (int dy = -reach; dy <= reach; ++dy)
            {
                for (int dx = -reach; dx <= reach; ++dx)
                {
                    if (!geometry.contains(x + dx, y + dy))
                    {
                        continue;
                    }
                    const auto other = static_cast<std::size_t>(geometry.index(x + dx, y + dy));
                    if (old_kind[other] == face &&
                        old_normal[other].dot(normal_[index]) >= kSameFacing)
                    {
                        quality_[index] = std::max(quality_[index], before[other]);
                    }
                }
            }
        }
    }
    // A cell that changed kind keeps nothing it earned as the other kind.
    if (same_grid)
    {
        for (std::size_t index = 0; index < count; ++index)
        {
            if (kind_[index] == static_cast<std::uint8_t>(TargetKind::kNone))
            {
                quality_[index] = 0;
            }
        }
    }
}

void CoverageMap::setSurfaces(const std::vector<Surface>& surfaces)
{
    std::fill(surface_at_.begin(), surface_at_.end(), -1);
    surface_height_.clear();
    if (cells_.empty())
    {
        return;
    }
    cv::Mat mask(geometry_.height, geometry_.width, CV_8UC1);
    for (const Surface& surface : surfaces)
    {
        if (surface.footprint.size() < 3)
        {
            continue;
        }
        std::vector<cv::Point> polygon;
        polygon.reserve(surface.footprint.size());
        for (const cv::Point2d& corner : surface.footprint)
        {
            const CellIndex cell = geometry_.toCell(corner.x, corner.y);
            polygon.emplace_back(cell.x, cell.y);
        }
        mask.setTo(0);
        cv::fillConvexPoly(mask, polygon, cv::Scalar(255));
        const int slot = static_cast<int>(surface_height_.size());
        surface_height_.push_back(surface.height);
        // A top found higher than it was credited at was never seen: only views that stopped below
        // it were, and those credited the front face's upper edge.
        const auto [top, first] = surface_top_.try_emplace(surface.id, surface.height);
        const bool rose         = !first && surface.height > top->second + params_.surface_band;
        if (rose)
        {
            top->second = surface.height;
        }
        for (int y = 0; y < mask.rows; ++y)
        {
            const std::uint8_t* row = mask.ptr<std::uint8_t>(y);
            for (int x = 0; x < mask.cols; ++x)
            {
                if (row[x] != 0)
                {
                    const auto index   = static_cast<std::size_t>(geometry_.index(x, y));
                    surface_at_[index] = slot;
                    if (rose)
                    {
                        surface_quality_[index] = 0;
                    }
                }
            }
        }
    }
    for (std::size_t index = 0; index < surface_at_.size(); ++index)
    {
        if (surface_at_[index] < 0)
        {
            surface_quality_[index] = 0;
        }
    }
}

float CoverageMap::edgeWeight(double normalised_u, double normalised_v)
{
    const double rho    = std::min(1.0, std::max(std::abs(normalised_u), std::abs(normalised_v)));
    const double square = rho * rho;
    return static_cast<float>(1.0 - (square * square));
}

float CoverageMap::viewQuality(
    double range, double cos_incidence, double edge_weight, TargetKind kind) const
{
    if (range < params_.min_range || range >= params_.max_range || edge_weight <= 0.0)
    {
        return 0.0F;
    }
    double limit = params_.face_incidence_limit;
    if (kind == TargetKind::kFloor)
    {
        limit = params_.floor_incidence_limit;
    }
    else if (kind == TargetKind::kSurface)
    {
        limit = params_.surface_incidence_limit;
    }
    const double cos_limit = incidenceLimitCos(limit);
    if (cos_incidence <= cos_limit)
    {
        return 0.0F;
    }
    const double angle_weight = (cos_incidence - cos_limit) / (1.0 - cos_limit);
    const double range_weight =
        range <= params_.good_range ?
            1.0 :
            (params_.max_range - range) / (params_.max_range - params_.good_range);
    return static_cast<float>(range_weight * angle_weight * edge_weight);
}

void CoverageMap::rebuildRays(const Intrinsics& intrinsics)
{
    rays_.clear();
    const int stride = std::max(1, params_.pixel_stride);
    // Offset by half a stride so the samples sit symmetrically about the centre.
    for (int v = stride / 2; v < intrinsics.height; v += stride)
    {
        for (int u = stride / 2; u < intrinsics.width; u += stride)
        {
            PixelRay ray{};
            ray.x       = static_cast<float>((u - intrinsics.cx) / intrinsics.fx);
            ray.y       = static_cast<float>((v - intrinsics.cy) / intrinsics.fy);
            ray.stretch = std::sqrt((ray.x * ray.x) + (ray.y * ray.y) + 1.0F);
            ray.edge    = edgeWeight(
                (u - intrinsics.cx) / (0.5 * intrinsics.width),
                (v - intrinsics.cy) / (0.5 * intrinsics.height));
            ray.u = u;
            ray.v = v;
            rays_.push_back(ray);
        }
    }
    rays_for_    = intrinsics;
    rays_stride_ = stride;
}

void CoverageMap::credit(std::size_t index, float quality, const Eigen::Vector3d& towards_camera)
{
    if (last_frame_[index] != frame_)
    {
        last_frame_[index] = frame_;
        views_[index]      = static_cast<std::uint16_t>(std::min<int>(views_[index] + 1, 0xFFFF));
    }
    directions_[index] |= sectorOf(towards_camera.x(), towards_camera.y());
    const auto scaled =
        static_cast<std::uint8_t>(std::lround(std::clamp(quality, 0.0F, 1.0F) * 255.0F));
    quality_[index] = std::max(quality_[index], scaled);
}

void CoverageMap::rebuildNearestFaces()
{
    nearest_face_.assign(geometry_.cellCount(), -1);
    // Faces are the zeros: with DIST_LABEL_PIXEL every zero gets its own label, numbered in
    // raster order, and every cell is labelled with its nearest zero.
    cv::Mat          not_face(geometry_.height, geometry_.width, CV_8UC1, cv::Scalar(255));
    std::vector<int> face_of_label{ -1 };
    for (int y = 0; y < geometry_.height; ++y)
    {
        for (int x = 0; x < geometry_.width; ++x)
        {
            const int index = geometry_.index(x, y);
            if (kind(index) == TargetKind::kFace)
            {
                not_face.at<std::uint8_t>(y, x) = 0;
                face_of_label.push_back(index);
            }
        }
    }
    if (face_of_label.size() == 1)
    {
        return;
    }
    cv::Mat distance;
    cv::Mat labels;
    cv::distanceTransform(
        not_face,
        distance,
        labels,
        cv::DIST_L2,
        cv::DIST_MASK_5,
        cv::DIST_LABEL_PIXEL);
    const auto reach = static_cast<float>(params_.face_reach / geometry_.resolution);
    for (int y = 0; y < geometry_.height; ++y)
    {
        const auto* away  = distance.ptr<float>(y);
        const auto* label = labels.ptr<int>(y);
        for (int x = 0; x < geometry_.width; ++x)
        {
            if (away[x] <= reach && label[x] > 0 &&
                label[x] < static_cast<int>(face_of_label.size()))
            {
                nearest_face_[static_cast<std::size_t>(geometry_.index(x, y))] =
                    face_of_label[static_cast<std::size_t>(label[x])];
            }
        }
    }
}

int CoverageMap::faceFacing(
    const Eigen::Vector3d& point, const Eigen::Vector3d& towards, std::size_t index) const
{
    const auto facing = [this, &towards](int face) {
        const cv::Vec2f normal = normal_[static_cast<std::size_t>(face)];
        return (towards.x() * normal[0]) + (towards.y() * normal[1]) > 0.1;
    };
    const int nearest = nearest_face_[index];
    if (nearest < 0 || facing(nearest))
    {
        return nearest;
    }
    // A thin wall and a localisation error can put the sample past the wall's middle, nearest the
    // face on its far side. Walk back towards the camera to the face that looks at it.
    const double length = std::hypot(towards.x(), towards.y());
    if (length < 1e-6)
    {
        return -1;
    }
    const int steps = std::max(1, geometry_.cellsFor(params_.face_reach));
    for (int step = 1; step <= steps; ++step)
    {
        const double    back = step * geometry_.resolution / length;
        const CellIndex cell =
            geometry_.toCell(point.x() + (back * towards.x()), point.y() + (back * towards.y()));
        if (!geometry_.contains(cell))
        {
            break;
        }
        const int face = nearest_face_[static_cast<std::size_t>(geometry_.index(cell))];
        if (face >= 0 && facing(face))
        {
            return face;
        }
    }
    return -1;
}

std::size_t CoverageMap::integrate(
    const DepthImage& depth, const Intrinsics& intrinsics, const Eigen::Isometry3d& map_from_camera)
{
    if (cells_.empty() || depth.data == nullptr || depth.width != intrinsics.width ||
        depth.height != intrinsics.height)
    {
        return 0;
    }
    if (!(intrinsics == rays_for_) || rays_stride_ != std::max(1, params_.pixel_stride))
    {
        rebuildRays(intrinsics);
    }
    ++frame_;

    const Eigen::Matrix3d rotation = map_from_camera.linear();
    const Eigen::Vector3d origin   = map_from_camera.translation();
    std::size_t           improved = 0;
    const auto            before   = [this](std::size_t index, bool surface) {
        return surface ? surface_quality_[index] : quality_[index];
    };

    for (const PixelRay& ray : rays_)
    {
        float z = 0.0F;
        std::memcpy(
            &z,
            depth.data + (static_cast<std::size_t>(ray.v) * depth.row_stride) + ray.u,
            sizeof(float));
        if (!std::isfinite(z) || z <= 0.0F)
        {
            continue;
        }
        const double range = static_cast<double>(z) * ray.stretch;
        if (range < params_.min_range || range >= params_.max_range)
        {
            continue;
        }
        const Eigen::Vector3d in_camera(
            static_cast<double>(ray.x) * z,
            static_cast<double>(ray.y) * z,
            z);
        const Eigen::Vector3d point   = (rotation * in_camera) + origin;
        const Eigen::Vector3d towards = (origin - point) / range;
        const CellIndex       cell    = geometry_.toCell(point.x(), point.y());
        if (!geometry_.contains(cell))
        {
            continue;
        }
        const auto index = static_cast<std::size_t>(geometry_.index(cell));

        if (std::abs(point.z()) <= params_.floor_band)
        {
            if (kind_[index] == static_cast<std::uint8_t>(TargetKind::kFloor))
            {
                const std::uint8_t old = before(index, false);
                const float quality = viewQuality(range, towards.z(), ray.edge, TargetKind::kFloor);
                credit(index, quality, towards);
                improved += static_cast<std::size_t>(quality_[index] > old);
            }
            continue;
        }

        const int slot = surface_at_[index];
        if (slot >= 0 && std::abs(point.z() - surface_height_[static_cast<std::size_t>(slot)]) <=
                             params_.surface_band)
        {
            const std::uint8_t old = before(index, true);
            const auto quality = viewQuality(range, towards.z(), ray.edge, TargetKind::kSurface);
            const auto scaled =
                static_cast<std::uint8_t>(std::lround(std::clamp(quality, 0.0F, 1.0F) * 255.0F));
            surface_quality_[index] = std::max(surface_quality_[index], scaled);
            improved += static_cast<std::size_t>(surface_quality_[index] > old);
            continue;
        }

        if (point.z() < params_.face_min_z || point.z() > params_.face_max_z)
        {
            continue;
        }
        const int face = faceFacing(point, towards, index);
        if (face < 0)
        {
            continue;
        }
        const auto      face_index = static_cast<std::size_t>(face);
        const cv::Vec2f normal     = normal_[face_index];
        // Against the face's horizontal normal, or its top: a table or a bed is mostly seen from
        // above, and its top is as good a view of it as its side.
        const double cos_incidence =
            std::max((towards.x() * normal[0]) + (towards.y() * normal[1]), towards.z());
        const std::uint8_t old = before(face_index, false);
        credit(face_index, viewQuality(range, cos_incidence, ray.edge, TargetKind::kFace), towards);
        improved += static_cast<std::size_t>(quality_[face_index] > old);
    }
    return improved;
}

int CoverageMap::countAttempt(int index)
{
    auto&     flags    = flags_[static_cast<std::size_t>(index)];
    const int attempts = std::min(7, ((flags & kAttemptMask) >> kAttemptShift) + 1);
    flags              = static_cast<std::uint8_t>(
        (flags & ~kAttemptMask) | (static_cast<unsigned>(attempts) << kAttemptShift));
    return attempts;
}

std::vector<CoverageTally> CoverageMap::tally(const cv::Mat& labels, int count) const
{
    std::vector<CoverageTally> tallies(static_cast<std::size_t>(std::max(count, 0)) + 1);
    if (labels.empty() || labels.rows != geometry_.height || labels.cols != geometry_.width)
    {
        return tallies;
    }
    for (int y = 0; y < geometry_.height; ++y)
    {
        for (int x = 0; x < geometry_.width; ++x)
        {
            const int      index = geometry_.index(x, y);
            const int      owner = std::clamp(roomOf(index, labels), 0, count);
            CoverageTally& t     = tallies[static_cast<std::size_t>(owner)];
            // Shares are of what can be seen: a written-off target counts only if seen after all.
            const bool seen = wellSeen(index);
            if (seen || !unobservable(index))
            {
                switch (kind(index))
                {
                    case TargetKind::kFloor:
                        ++t.floor;
                        t.floor_seen += static_cast<int>(seen);
                        break;
                    case TargetKind::kFace:
                        ++t.face;
                        t.face_seen += static_cast<int>(seen);
                        break;
                    default:
                        break;
                }
            }
            const bool surface_seen = surfaceWellSeen(index);
            if (surfaceAt(index) >= 0 && (surface_seen || !surfaceUnobservable(index)))
            {
                ++t.surface;
                t.surface_seen += static_cast<int>(surface_seen);
            }
            t.unobservable += static_cast<int>(unobservable(index) || surfaceUnobservable(index));
        }
    }
    return tallies;
}

int CoverageMap::roomOf(int index, const cv::Mat& labels) const
{
    const int count = static_cast<int>(geometry_.cellCount());
    const int cell  = index >= count ? index - count : index;
    const int x     = cell % geometry_.width;
    const int y     = cell / geometry_.width;
    const int label = labels.at<int>(y, x);
    if (label != 0 || index >= count || kind(cell) != TargetKind::kFace)
    {
        return label;
    }
    const cv::Vec2f normal = normal_[static_cast<std::size_t>(cell)];
    const int       fx     = x + static_cast<int>(std::lround(normal[0]));
    const int       fy     = y + static_cast<int>(std::lround(normal[1]));
    return geometry_.contains(fx, fy) ? labels.at<int>(fy, fx) : 0;
}

std::vector<std::int8_t> CoverageMap::statusGrid() const
{
    constexpr std::int8_t    kToSee      = 90;
    constexpr std::int8_t    kWrittenOff = 99;
    std::vector<std::int8_t> grid(geometry_.cellCount(), 0);
    const auto               count   = grid.size();
    const bool               planned = planned_.size() == 2 * count;
    for (std::size_t index = 0; index < count; ++index)
    {
        const int  cell_index = static_cast<int>(index);
        const bool to_see     = pending(cell_index) && (!planned || planned_[index] != 0);
        const bool top_to_see =
            surfacePending(cell_index) && (!planned || planned_[count + index] != 0);
        if (to_see || top_to_see)
        {
            grid[index] = kToSee;
        }
        else if (
            pending(cell_index) || surfacePending(cell_index) || unobservable(cell_index) ||
            surfaceUnobservable(cell_index))
        {
            grid[index] = kWrittenOff;
        }
    }
    return grid;
}

bool CoverageMap::restoreLayers(
    const std::vector<std::uint8_t>& quality, const std::vector<std::uint8_t>& surface_quality,
    const std::vector<std::uint8_t>& flags, const std::vector<std::uint8_t>& directions)
{
    const std::size_t count = geometry_.cellCount();
    if (quality.size() != count || surface_quality.size() != count || flags.size() != count ||
        directions.size() != count)
    {
        return false;
    }
    quality_         = quality;
    surface_quality_ = surface_quality;
    flags_           = flags;
    directions_      = directions;
    return true;
}

}  // namespace canopy
