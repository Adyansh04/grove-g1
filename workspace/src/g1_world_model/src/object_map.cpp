/**
 * @file object_map.cpp
 * @brief Lifting, association, merging and absence checks for the object layer.
 */

#include "g1_world_model/object_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <opencv2/imgproc.hpp>

namespace g1_world_model
{

namespace
{

constexpr std::int64_t  kKeyOffset = std::int64_t{ 1 } << 20;
constexpr std::uint64_t kKeyMask   = (std::uint64_t{ 1 } << 21) - 1;

float depthAt(const DepthImage& depth, int u, int v)
{
    float value = 0.0F;
    std::memcpy(
        &value,
        depth.data + (static_cast<std::size_t>(v) * depth.row_stride) + u,
        sizeof(float));
    return value;
}

/// Every n-th element, keeping at most @p limit.
template <typename T>
std::vector<T> thinned(const std::vector<T>& values, std::size_t limit)
{
    if (values.size() <= limit)
    {
        return values;
    }
    const std::size_t step = (values.size() + limit - 1) / limit;
    std::vector<T>    out;
    out.reserve(limit);
    for (std::size_t i = 0; i < values.size(); i += step)
    {
        out.push_back(values[i]);
    }
    return out;
}

/// Whether two footprints, grown by @p gap between them, overlap and their heights meet.
bool touching(const MappedObject& a, const MappedObject& b, double gap)
{
    if (a.z_min > b.z_max + gap || b.z_min > a.z_max + gap)
    {
        return false;
    }
    const std::array<Eigen::Vector2d, 2> along_a{
        Eigen::Vector2d(std::cos(a.box_yaw), std::sin(a.box_yaw)),
        Eigen::Vector2d(-std::sin(a.box_yaw), std::cos(a.box_yaw))
    };
    const std::array<Eigen::Vector2d, 2> along_b{
        Eigen::Vector2d(std::cos(b.box_yaw), std::sin(b.box_yaw)),
        Eigen::Vector2d(-std::sin(b.box_yaw), std::cos(b.box_yaw))
    };
    const Eigen::Vector2d offset = b.box_centre - a.box_centre;
    const auto            meet   = [&](const Eigen::Vector2d& axis) {
        const double reach_a = 0.5 * ((a.box_size.x() * std::abs(axis.dot(along_a[0]))) +
                                      (a.box_size.y() * std::abs(axis.dot(along_a[1]))));
        const double reach_b = 0.5 * ((b.box_size.x() * std::abs(axis.dot(along_b[0]))) +
                                      (b.box_size.y() * std::abs(axis.dot(along_b[1]))));
        return std::abs(axis.dot(offset)) <= reach_a + reach_b + gap;
    };
    // Separating axes: two rectangles are apart iff one of their four edge directions parts them.
    return std::ranges::all_of(std::array{ along_a[0], along_a[1], along_b[0], along_b[1] }, meet);
}

/// Longest side of the axis-aligned box around both footprints.
double jointExtent(const MappedObject& a, const MappedObject& b)
{
    Eigen::Vector2d low  = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector2d high = -low;
    for (const MappedObject* object : { &a, &b })
    {
        const Eigen::Rotation2Dd rotation(object->box_yaw);
        for (const double sx : { -0.5, 0.5 })
        {
            for (const double sy : { -0.5, 0.5 })
            {
                const Eigen::Vector2d corner =
                    object->box_centre +
                    (rotation *
                     Eigen::Vector2d(sx * object->box_size.x(), sy * object->box_size.y()));
                low  = low.cwiseMin(corner);
                high = high.cwiseMax(corner);
            }
        }
    }
    return (high - low).maxCoeff();
}

}  // namespace

std::string MappedObject::label() const
{
    const auto best =
        std::max_element(votes.begin(), votes.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });
    return best == votes.end() ? std::string{} : best->first;
}

float MappedObject::confidence() const
{
    float total = 0.0F;
    float best  = 0.0F;
    for (const auto& [label, weight] : votes)
    {
        total += weight;
        best = std::max(best, weight);
    }
    return total > 0.0F ? best / total : 0.0F;
}

ObjectMap::ObjectMap(ObjectMapParams params)
  : params_(std::move(params))
{}

std::uint64_t ObjectMap::keyOf(const Eigen::Vector3d& point) const
{
    const auto axis = [this](double value) {
        const auto cell = static_cast<std::int64_t>(std::floor(value / params_.voxel)) + kKeyOffset;
        return static_cast<std::uint64_t>(std::clamp<std::int64_t>(cell, 0, kKeyMask));
    };
    return (axis(point.x()) << 42U) | (axis(point.y()) << 21U) | axis(point.z());
}

Eigen::Vector3d ObjectMap::centreOf(std::uint64_t key) const
{
    const auto axis = [this](std::uint64_t bits) {
        return (static_cast<double>(static_cast<std::int64_t>(bits & kKeyMask) - kKeyOffset) + 0.5) *
               params_.voxel;
    };
    return { axis(key >> 42U), axis(key >> 21U), axis(key) };
}

std::optional<ObjectMap::Lifted>
ObjectMap::lift(const MaskInput& mask, const FrameInput& frame) const
{
    const Intrinsics& k = frame.intrinsics;
    if (mask.mask == nullptr || mask.width <= 0 || mask.height <= 0 || frame.depth.data == nullptr)
    {
        return std::nullopt;
    }

    // Mask pixels two pixels in from its edge: the rim straddles depth edges and drags in the
    // background. Small masks keep their rim; they have little else.
    const bool erode  = mask.width * mask.height > 400;
    const auto inside = [&mask](int u, int v) {
        return u >= 0 && v >= 0 && u < mask.width && v < mask.height &&
               mask.mask[(static_cast<std::size_t>(v) * mask.width) + u] != 0;
    };
    std::vector<std::pair<int, int>> pixels;
    for (int v = 0; v < mask.height; ++v)
    {
        for (int u = 0; u < mask.width; ++u)
        {
            if (!inside(u, v))
            {
                continue;
            }
            if (erode &&
                !(inside(u - 2, v) && inside(u + 2, v) && inside(u, v - 2) && inside(u, v + 2)))
            {
                continue;
            }
            pixels.emplace_back(u + mask.x, v + mask.y);
        }
    }
    pixels = thinned(pixels, static_cast<std::size_t>(params_.max_points));

    std::vector<std::uint64_t> keys;
    keys.reserve(pixels.size());
    for (const auto& [u, v] : pixels)
    {
        if (u < 0 || v < 0 || u >= frame.depth.width || v >= frame.depth.height)
        {
            continue;
        }
        const float z = depthAt(frame.depth, u, v);
        if (!std::isfinite(z) || z < params_.min_depth || z > params_.max_depth)
        {
            continue;
        }
        const Eigen::Vector3d in_camera((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z);
        const Eigen::Vector3d point = frame.map_from_camera * in_camera;
        if (point.z() < params_.floor_cut)
        {
            continue;
        }
        keys.push_back(keyOf(point));
    }
    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
    if (static_cast<int>(keys.size()) < params_.min_voxels)
    {
        return std::nullopt;
    }

    // Largest 26-connected cluster: whatever the mask caught behind the object falls away.
    std::vector<int> parent(keys.size());
    std::iota(parent.begin(), parent.end(), 0);
    const auto find = [&parent](int node) {
        while (parent[static_cast<std::size_t>(node)] != node)
        {
            auto& up = parent[static_cast<std::size_t>(node)];
            up       = parent[static_cast<std::size_t>(up)];
            node     = up;
        }
        return node;
    };
    const auto shifted = [](std::uint64_t key, int dx, int dy, int dz) {
        const auto step = [](std::uint64_t bits, int delta) {
            return static_cast<std::uint64_t>(static_cast<std::int64_t>(bits & kKeyMask) + delta) &
                   kKeyMask;
        };
        return (step(key >> 42U, dx) << 42U) | (step(key >> 21U, dy) << 21U) | step(key, dz);
    };
    for (std::size_t i = 0; i < keys.size(); ++i)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    const std::uint64_t neighbour = shifted(keys[i], dx, dy, dz);
                    if (neighbour <= keys[i])
                    {
                        continue;  // Each pair once, from its lower key.
                    }
                    const auto found = std::lower_bound(keys.begin(), keys.end(), neighbour);
                    if (found != keys.end() && *found == neighbour)
                    {
                        const int a = find(static_cast<int>(i));
                        const int b = find(static_cast<int>(found - keys.begin()));
                        parent[static_cast<std::size_t>(std::max(a, b))] = std::min(a, b);
                    }
                }
            }
        }
    }
    std::vector<int> size(keys.size(), 0);
    int              largest = 0;
    for (std::size_t i = 0; i < keys.size(); ++i)
    {
        const int root = find(static_cast<int>(i));
        if (++size[static_cast<std::size_t>(root)] > size[static_cast<std::size_t>(largest)])
        {
            largest = root;
        }
    }

    Lifted lifted;
    for (std::size_t i = 0; i < keys.size(); ++i)
    {
        if (find(static_cast<int>(i)) == largest)
        {
            lifted.voxels.push_back(keys[i]);
            lifted.centroid += centreOf(keys[i]);
        }
    }
    if (static_cast<int>(lifted.voxels.size()) < params_.min_voxels)
    {
        return std::nullopt;
    }
    lifted.centroid /= static_cast<double>(lifted.voxels.size());
    return lifted;
}

double ObjectMap::overlap(
    const std::vector<std::uint64_t>& from, const std::vector<std::uint64_t>& into) const
{
    if (from.empty() || into.empty())
    {
        return 0.0;
    }
    // A sample is plenty to measure a fraction, and keeps big furniture cheap.
    const std::vector<std::uint64_t> sample = thinned(from, 600);
    int                              near   = 0;
    for (const std::uint64_t key : sample)
    {
        bool found = false;
        for (int dx = -1; dx <= 1 && !found; ++dx)
        {
            for (int dy = -1; dy <= 1 && !found; ++dy)
            {
                for (int dz = -1; dz <= 1 && !found; ++dz)
                {
                    const Eigen::Vector3d centre =
                        centreOf(key) + (params_.voxel * Eigen::Vector3d(dx, dy, dz));
                    found = std::binary_search(into.begin(), into.end(), keyOf(centre));
                }
            }
        }
        near += static_cast<int>(found);
    }
    return static_cast<double>(near) / static_cast<double>(sample.size());
}

double ObjectMap::proximity(const Lifted& detection, const MappedObject& object) const
{
    const Eigen::Vector2d offset   = detection.centroid.head<2>() - object.box_centre;
    const double          c        = std::cos(object.box_yaw);
    const double          s        = std::sin(object.box_yaw);
    const double          along    = (c * offset.x()) + (s * offset.y());
    const double          across   = (-s * offset.x()) + (c * offset.y());
    const double          dx       = std::max(0.0, std::abs(along) - (0.5 * object.box_size.x()));
    const double          dy       = std::max(0.0, std::abs(across) - (0.5 * object.box_size.y()));
    const double          dz       = detection.centroid.z() < object.z_min ?
                                         object.z_min - detection.centroid.z() :
                                         std::max(0.0, detection.centroid.z() - object.z_max);
    const double          distance = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
    return std::max(0.0, 1.0 - (distance / params_.position_tolerance));
}

double ObjectMap::similarity(const std::vector<float>& a, const std::vector<float>& b)
{
    if (a.empty() || a.size() != b.size())
    {
        return -1.0;
    }
    double dot = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
    {
        dot += static_cast<double>(a[i]) * b[i];
    }
    return dot;
}

void ObjectMap::refreshShape(MappedObject& object) const
{
    if (object.voxels.empty())
    {
        return;
    }
    const std::vector<std::uint64_t> sample = thinned(object.voxels, 5000);
    std::vector<cv::Point2f>         footprint;
    footprint.reserve(sample.size());
    Eigen::Vector3d sum   = Eigen::Vector3d::Zero();
    double          z_min = std::numeric_limits<double>::max();
    double          z_max = std::numeric_limits<double>::lowest();
    for (const std::uint64_t key : sample)
    {
        const Eigen::Vector3d centre = centreOf(key);
        sum += centre;
        z_min = std::min(z_min, centre.z());
        z_max = std::max(z_max, centre.z());
        footprint.emplace_back(static_cast<float>(centre.x()), static_cast<float>(centre.y()));
    }
    object.centroid = sum / static_cast<double>(sample.size());
    object.z_min    = z_min - (0.5 * params_.voxel);
    object.z_max    = z_max + (0.5 * params_.voxel);

    const cv::RotatedRect box = cv::minAreaRect(footprint);
    object.box_centre         = { box.center.x, box.center.y };
    object.box_size           = { box.size.width + params_.voxel, box.size.height + params_.voxel };
    object.box_yaw            = box.angle * std::numbers::pi / 180.0;
}

void ObjectMap::absorb(
    MappedObject& object, const Lifted& detection, const MaskInput& mask, const FrameInput& frame,
    MaskOutcome& outcome)
{
    object.votes[mask.label] += mask.score;
    if (!mask.embedding.empty())
    {
        if (object.embedding.size() != mask.embedding.size())
        {
            object.embedding       = mask.embedding;
            object.embedding_count = 1;
        }
        else
        {
            const auto count = static_cast<float>(object.embedding_count);
            double     norm  = 0.0;
            for (std::size_t i = 0; i < object.embedding.size(); ++i)
            {
                object.embedding[i] =
                    ((object.embedding[i] * count) + mask.embedding[i]) / (count + 1.0F);
                norm += static_cast<double>(object.embedding[i]) * object.embedding[i];
            }
            const auto scale = static_cast<float>(1.0 / std::max(std::sqrt(norm), 1e-9));
            for (float& value : object.embedding)
            {
                value *= scale;
            }
            ++object.embedding_count;
        }
    }

    std::vector<std::uint64_t> merged;
    merged.reserve(object.voxels.size() + detection.voxels.size());
    std::set_union(
        object.voxels.begin(),
        object.voxels.end(),
        detection.voxels.begin(),
        detection.voxels.end(),
        std::back_inserter(merged));
    object.voxels = thinned(merged, static_cast<std::size_t>(params_.max_voxels));

    if (object.observations == 0)
    {
        object.first_seen = frame.stamp;
    }
    ++object.observations;
    object.last_seen = std::max(object.last_seen, frame.stamp);
    object.misses    = 0;
    object.state     = ObjectState::kActive;

    // Big, central and whole beats small, peripheral or cut off by the frame edge.
    const Intrinsics& k          = frame.intrinsics;
    const double      centre_u   = mask.x + (0.5 * mask.width);
    const double      centre_v   = mask.y + (0.5 * mask.height);
    const double      centrality = CoverageMap::edgeWeight(
        (centre_u - k.cx) / (0.5 * k.width),
        (centre_v - k.cy) / (0.5 * k.height));
    const bool clipped = mask.x <= 1 || mask.y <= 1 || mask.x + mask.width >= k.width - 1 ||
                         mask.y + mask.height >= k.height - 1;
    const double view_score = static_cast<double>(mask.width) * mask.height * centrality *
                              (clipped ? 0.4 : 1.0) * mask.score;
    if (view_score > object.best_view.score)
    {
        object.best_view  = { frame.stamp, mask.x, mask.y, mask.width, mask.height, view_score };
        outcome.best_view = true;
    }
    refreshShape(object);
}

std::vector<MaskOutcome>
ObjectMap::integrate(const std::vector<MaskInput>& masks, const FrameInput& frame)
{
    ++frames_;
    std::vector<std::optional<Lifted>> lifted;
    lifted.reserve(masks.size());
    for (const MaskInput& mask : masks)
    {
        lifted.push_back(lift(mask, frame));
    }

    struct Pair
    {
        double      score;
        std::size_t mask;
        std::size_t object;
    };
    std::vector<Pair> pairs;
    for (std::size_t m = 0; m < masks.size(); ++m)
    {
        if (!lifted[m])
        {
            continue;
        }
        const Lifted& detection = *lifted[m];
        for (std::size_t o = 0; o < objects_.size(); ++o)
        {
            const MappedObject& object = objects_[o];
            if (object.state == ObjectState::kRemoved)
            {
                continue;
            }
            const double reach = params_.match_radius + (0.5 * object.box_size.maxCoeff());
            if ((object.centroid - detection.centroid).head<2>().norm() > reach)
            {
                continue;
            }
            const double shared = overlap(detection.voxels, object.voxels);
            float        total  = 0.0F;
            for (const auto& [label, weight] : object.votes)
            {
                total += weight;
            }
            const auto   vote        = object.votes.find(masks[m].label);
            const double label_share = vote == object.votes.end() || total <= 0.0F ?
                                           0.0 :
                                           static_cast<double>(vote->second) / total;
            const double geometric =
                std::max(shared, label_share > 0.0 ? proximity(detection, object) : 0.0);
            const double cosine = similarity(masks[m].embedding, object.embedding);
            const double semantic =
                cosine >= 0.0 ? std::clamp((cosine - 0.5) / 0.4, 0.0, 1.0) : label_share;
            const double score =
                (params_.geometry_weight * geometric) + (params_.semantic_weight * semantic);
            // Overlap alone decides only between comparable sizes: a mug's few voxels all sit
            // within a voxel of the table top it stands on.
            const bool strong =
                shared >= params_.strong_overlap &&
                static_cast<double>(detection.voxels.size()) >=
                    params_.strong_share * static_cast<double>(object.voxels.size());
            // Nothing says they are the same kind of thing: overlap alone would put a book on a
            // shelf into the shelf, whose mask took in the book as well.
            const bool kin = label_share > 0.0 || cosine >= 0.0;
            if ((kin && score >= params_.min_match && geometric >= 0.15) || strong)
            {
                pairs.push_back({ score + (strong ? 1.0 : 0.0), m, o });
            }
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
        return a.score > b.score;
    });

    std::vector<MaskOutcome> outcomes(masks.size());
    std::vector<bool>        object_taken(objects_.size(), false);
    std::vector<bool>        mask_taken(masks.size(), false);
    for (const Pair& pair : pairs)
    {
        if (mask_taken[pair.mask] || object_taken[pair.object])
        {
            continue;
        }
        mask_taken[pair.mask]      = true;
        object_taken[pair.object]  = true;
        MappedObject& object       = objects_[pair.object];
        outcomes[pair.mask].object = object.id;
        absorb(object, *lifted[pair.mask], masks[pair.mask], frame, outcomes[pair.mask]);
    }
    for (std::size_t m = 0; m < masks.size(); ++m)
    {
        if (mask_taken[m] || !lifted[m] || masks[m].score < params_.min_score)
        {
            continue;
        }
        MappedObject object;
        object.id           = next_id_++;
        outcomes[m].object  = object.id;
        outcomes[m].created = true;
        absorb(object, *lifted[m], masks[m], frame, outcomes[m]);
        objects_.push_back(std::move(object));
    }

    std::vector<int> matched;
    for (const MaskOutcome& outcome : outcomes)
    {
        if (outcome.object != 0)
        {
            matched.push_back(outcome.object);
        }
    }
    checkAbsence(frame, matched);
    if (params_.merge_every > 0 && frames_ % params_.merge_every == 0)
    {
        pruneUnconfirmed(frame.stamp);
        mergeDuplicates();
    }
    relateSupports();
    return outcomes;
}

void ObjectMap::checkAbsence(const FrameInput& frame, const std::vector<int>& matched)
{
    if (frame.depth.data == nullptr)
    {
        return;
    }
    const Intrinsics&       k               = frame.intrinsics;
    const Eigen::Isometry3d camera_from_map = frame.map_from_camera.inverse();
    for (MappedObject& object : objects_)
    {
        if (object.state == ObjectState::kRemoved ||
            std::find(matched.begin(), matched.end(), object.id) != matched.end())
        {
            continue;
        }
        const Eigen::Vector3d centre = camera_from_map * object.centroid;
        if (centre.z() < params_.min_depth || centre.z() > 3.5)
        {
            continue;
        }
        // Small objects need a small margin: a mug gone from a table leaves the table top only a
        // few centimetres behind where the mug was. Its bottom layer is left out: that is where
        // it touched the table, and the table is still there.
        const double extent =
            std::max({ object.box_size.x(), object.box_size.y(), object.height() });
        const double margin = std::clamp(0.25 * extent, 0.75 * params_.voxel, params_.miss_margin);
        std::vector<std::uint64_t> above;
        for (const std::uint64_t key : object.voxels)
        {
            if (centreOf(key).z() > object.z_min + params_.voxel)
            {
                above.push_back(key);
            }
        }
        const std::vector<std::uint64_t> sample =
            thinned(above.empty() ? object.voxels : above, 24);
        // A mug has a handful of voxels; demand half of them, but never fewer than three.
        const int needed  = std::min(6, std::max(3, static_cast<int>(sample.size()) / 2));
        int       through = 0;
        int       present = 0;
        for (const std::uint64_t key : sample)
        {
            const Eigen::Vector3d point = camera_from_map * centreOf(key);
            if (point.z() < params_.min_depth)
            {
                continue;
            }
            const int u = static_cast<int>(std::lround((k.fx * point.x() / point.z()) + k.cx));
            const int v = static_cast<int>(std::lround((k.fy * point.y() / point.z()) + k.cy));
            // Keep off the border, where the detector itself cuts objects off.
            if (u < k.width / 20 || v < k.height / 20 || u >= k.width - (k.width / 20) ||
                v >= k.height - (k.height / 20))
            {
                continue;
            }
            const float measured = depthAt(frame.depth, u, v);
            if (!std::isfinite(measured) || measured <= 0.0F)
            {
                continue;
            }
            if (measured > point.z() + margin)
            {
                ++through;
            }
            else if (measured > point.z() - margin)
            {
                ++present;
            }
        }
        const int evidence = through + present;
        if (evidence < needed)
        {
            continue;
        }
        if (through >= (7 * evidence) / 10)
        {
            ++object.misses;
            if (object.misses >= params_.remove_after)
            {
                object.state = ObjectState::kRemoved;
            }
            else if (object.misses >= params_.stale_after)
            {
                object.state = ObjectState::kStale;
            }
        }
    }
}

void ObjectMap::mergeInto(MappedObject& keep, MappedObject& drop) const
{
    for (const auto& [label, weight] : drop.votes)
    {
        keep.votes[label] += weight;
    }
    if (!drop.embedding.empty() && drop.embedding.size() == keep.embedding.size())
    {
        const auto a    = static_cast<float>(keep.embedding_count);
        const auto b    = static_cast<float>(drop.embedding_count);
        double     norm = 0.0;
        for (std::size_t i = 0; i < keep.embedding.size(); ++i)
        {
            keep.embedding[i] = ((keep.embedding[i] * a) + (drop.embedding[i] * b)) / (a + b);
            norm += static_cast<double>(keep.embedding[i]) * keep.embedding[i];
        }
        const auto scale = static_cast<float>(1.0 / std::max(std::sqrt(norm), 1e-9));
        for (float& value : keep.embedding)
        {
            value *= scale;
        }
        keep.embedding_count += drop.embedding_count;
    }
    else if (keep.embedding.empty())
    {
        keep.embedding       = drop.embedding;
        keep.embedding_count = drop.embedding_count;
    }
    std::vector<std::uint64_t> merged;
    std::set_union(
        keep.voxels.begin(),
        keep.voxels.end(),
        drop.voxels.begin(),
        drop.voxels.end(),
        std::back_inserter(merged));
    keep.voxels = thinned(merged, static_cast<std::size_t>(params_.max_voxels));
    keep.observations += drop.observations;
    keep.first_seen = std::min(keep.first_seen, drop.first_seen);
    keep.last_seen  = std::max(keep.last_seen, drop.last_seen);
    if (drop.best_view.score > keep.best_view.score)
    {
        keep.best_view = drop.best_view;
    }
    if (keep.name.empty())
    {
        keep.name    = drop.name;
        keep.caption = drop.caption;
    }
    refreshShape(keep);
}

int ObjectMap::mergeDuplicates()
{
    int merged = 0;
    for (std::size_t i = 0; i < objects_.size(); ++i)
    {
        for (std::size_t j = i + 1; j < objects_.size(); ++j)
        {
            MappedObject& a = objects_[i];
            MappedObject& b = objects_[j];
            if (a.state == ObjectState::kRemoved || b.state == ObjectState::kRemoved)
            {
                continue;
            }
            const bool same = a.label() == b.label();
            if (!same && similarity(a.embedding, b.embedding) < params_.merge_similarity)
            {
                continue;
            }
            const bool overlapping =
                (a.centroid - b.centroid).norm() <= params_.merge_distance &&
                std::max(overlap(a.voxels, b.voxels), overlap(b.voxels, a.voxels)) >=
                    params_.merge_overlap;
            // Parts of one object share no voxels: two sides of a table touch, and a table's legs
            // and the ends either side of a chair stand apart, however far their centres.
            const bool parts =
                same && (touching(a, b, params_.merge_gap) ||
                         (isSupport(a.label()) && touching(a, b, params_.support_merge_gap) &&
                          jointExtent(a, b) <= params_.max_merged_extent));
            if (!overlapping && !parts)
            {
                continue;
            }
            // The better-established object keeps its id.
            const bool keep_a = a.observations >= b.observations;
            mergeInto(keep_a ? a : b, keep_a ? b : a);
            objects_.erase(objects_.begin() + static_cast<std::ptrdiff_t>(keep_a ? j : i));
            ++merged;
            j = i;  // Restart the inner scan: `a` changed, or `i` now holds another object.
        }
    }
    return merged;
}

int ObjectMap::pruneUnconfirmed(double now)
{
    const auto before = objects_.size();
    std::erase_if(objects_, [&](const MappedObject& object) {
        return object.observations < params_.min_observations &&
               now - object.first_seen > params_.confirm_s;
    });
    return static_cast<int>(before - objects_.size());
}

void ObjectMap::relateSupports()
{
    for (MappedObject& object : objects_)
    {
        object.support         = 0;
        double       best_top  = -1.0;
        const double footprint = object.box_size.x() * object.box_size.y();
        for (const MappedObject& under : objects_)
        {
            if (under.id == object.id || under.state == ObjectState::kRemoved ||
                !isSupport(under.label()) || under.box_size.x() * under.box_size.y() <= footprint)
            {
                continue;
            }
            if (object.z_min < under.z_max - 0.08 || object.z_min > under.z_max + 0.10)
            {
                continue;
            }
            const Eigen::Vector2d offset = object.box_centre - under.box_centre;
            const double          c      = std::cos(under.box_yaw);
            const double          s      = std::sin(under.box_yaw);
            if (std::abs((c * offset.x()) + (s * offset.y())) > (0.5 * under.box_size.x()) + 0.05 ||
                std::abs((-s * offset.x()) + (c * offset.y())) > (0.5 * under.box_size.y()) + 0.05)
            {
                continue;
            }
            if (under.z_max > best_top)
            {
                best_top       = under.z_max;
                object.support = under.id;
            }
        }
    }
}

bool ObjectMap::isSupport(const std::string& label) const
{
    return std::find(params_.support_labels.begin(), params_.support_labels.end(), label) !=
           params_.support_labels.end();
}

std::vector<Surface> ObjectMap::surfaces() const
{
    std::vector<Surface> out;
    for (const MappedObject& object : objects_)
    {
        if (object.state == ObjectState::kRemoved || !isSupport(object.label()) ||
            object.height() < 0.2)
        {
            continue;
        }
        Surface surface;
        surface.id     = object.id;
        surface.height = object.z_max;
        const double c = std::cos(object.box_yaw);
        const double s = std::sin(object.box_yaw);
        for (const auto& [sx, sy] : { std::pair{ 1, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 } })
        {
            const double along  = 0.5 * sx * object.box_size.x();
            const double across = 0.5 * sy * object.box_size.y();
            surface.footprint.emplace_back(
                object.box_centre.x() + (c * along) - (s * across),
                object.box_centre.y() + (s * along) + (c * across));
        }
        out.push_back(std::move(surface));
    }
    return out;
}

MappedObject* ObjectMap::find(int id)
{
    const auto found = std::find_if(objects_.begin(), objects_.end(), [id](const MappedObject& o) {
        return o.id == id;
    });
    return found == objects_.end() ? nullptr : &*found;
}

const MappedObject* ObjectMap::find(int id) const
{
    const auto found = std::find_if(objects_.begin(), objects_.end(), [id](const MappedObject& o) {
        return o.id == id;
    });
    return found == objects_.end() ? nullptr : &*found;
}

void ObjectMap::restore(std::vector<MappedObject> objects)
{
    objects_ = std::move(objects);
    next_id_ = 1;
    for (MappedObject& object : objects_)
    {
        next_id_ = std::max(next_id_, object.id + 1);
        std::sort(object.voxels.begin(), object.voxels.end());
        refreshShape(object);
    }
}

}  // namespace g1_world_model
