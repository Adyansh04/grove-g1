#ifndef G1_WORLD_MODEL__COVERAGE_MAP_HPP_
#define G1_WORLD_MODEL__COVERAGE_MAP_HPP_

/**
 * @file coverage_map.hpp
 * @brief What the head camera has seen of the map, and how well.
 *
 * Three kinds of target share the occupancy grid's cells: floor (free cells), faces (the sides
 * of walls and furniture, i.e. occupied cells next to free space) and surfaces (the tops of
 * tables and shelves, registered once their objects are known). Each depth frame is lifted into
 * the map and every sample credits the target it lands on with a view quality from its range,
 * its angle to the surface and its distance from the image centre. A target is seen well enough
 * once its best view clears CoverageParams::well_seen. Measured coverage comes only from depth,
 * which already knows about the chair legs and table edges a 2D grid cannot represent.
 */

#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

#include "g1_world_model/grid.hpp"

namespace g1_world_model
{

enum class TargetKind : std::uint8_t
{
    kNone    = 0,
    kFloor   = 1,
    kFace    = 2,
    kSurface = 3,
};

struct CoverageParams
{
    double min_range    = 0.2;   ///< Closer samples are the robot itself or noise, m.
    double good_range   = 2.0;   ///< Full quality up to here, m.
    double max_range    = 4.0;   ///< Quality falls to zero here, m.
    double floor_band   = 0.06;  ///< Samples this close to z = 0 are floor, m.
    double face_min_z   = 0.10;  ///< Samples in this band credit the face of their cell, m.
    double face_max_z   = 2.0;
    double surface_band = 0.05;  ///< Samples this close to a surface's height credit it, m.
    /// A wall sample credits the face cell this near, m: AMCL puts walls up to 0.2 m off the map's.
    double face_reach              = 0.25;
    double floor_incidence_limit   = 1.40;  ///< Beyond this from the normal a view is useless, rad.
    double face_incidence_limit    = 1.31;
    double surface_incidence_limit = 1.48;
    double well_seen               = 0.35;  ///< Quality at which a target counts as seen.
    int    pixel_stride            = 4;     ///< Every n-th pixel in each direction is sampled.
};

/// Pinhole intrinsics of the depth image, in pixels.
struct Intrinsics
{
    double fx     = 0.0;
    double fy     = 0.0;
    double cx     = 0.0;
    double cy     = 0.0;
    int    width  = 0;
    int    height = 0;

    friend bool operator==(const Intrinsics&, const Intrinsics&) = default;
};

/// A depth image in metres, row-major; `row_stride` is in floats. Non-finite or 0 is no return.
struct DepthImage
{
    const float* data       = nullptr;
    int          width      = 0;
    int          height     = 0;
    std::size_t  row_stride = 0;
};

/// The top of a support object, registered once the object map knows it.
struct Surface
{
    int                      id     = 0;    ///< The owner's id, for bookkeeping only.
    double                   height = 0.0;  ///< m above the floor.
    std::vector<cv::Point2d> footprint;     ///< Map frame polygon.
};

/// Per-room counts of targets, and of those seen well enough.
struct CoverageTally
{
    int floor        = 0;
    int floor_seen   = 0;
    int face         = 0;
    int face_seen    = 0;
    int surface      = 0;
    int surface_seen = 0;
    int unobservable = 0;
};

class CoverageMap
{
public:
    explicit CoverageMap(CoverageParams params = {});

    /**
     * @brief Rebuilds the targets for an occupancy grid.
     *
     * Qualities carry over cell by cell when the geometry is unchanged, so a map that only
     * gained a few obstacles keeps what the camera already saw.
     */
    void setMap(const cv::Mat& cells, const GridGeometry& geometry);

    /// Replaces the registered surfaces. Their cells keep their quality if they stay covered.
    void setSurfaces(const std::vector<Surface>& surfaces);

    /**
     * @brief Credits the targets a depth frame sees.
     *
     * @param depth           The frame, in metres.
     * @param intrinsics      Its pinhole model.
     * @param map_from_camera Pose of the optical frame (z forward, x right, y down) in the map.
     * @return Targets whose quality improved.
     */
    std::size_t integrate(
        const DepthImage& depth, const Intrinsics& intrinsics,
        const Eigen::Isometry3d& map_from_camera);

    /**
     * @brief The quality of one view of a target; the measure prediction and measurement share.
     *
     * @param range         Distance from the camera, m.
     * @param cos_incidence Cosine of the angle between the view ray and the surface normal.
     * @param edge_weight   1 at the image centre, falling to 0 at its border.
     * @param kind          What was seen; each kind has its own incidence limit.
     */
    [[nodiscard]] float
    viewQuality(double range, double cos_incidence, double edge_weight, TargetKind kind) const;

    /// 1 at the image centre, 0 at the border: 1 - rho^4 of the larger normalised offset.
    [[nodiscard]] static float edgeWeight(double normalised_u, double normalised_v);

    [[nodiscard]] TargetKind kind(int index) const
    {
        return static_cast<TargetKind>(kind_[static_cast<std::size_t>(index)]);
    }

    [[nodiscard]] float quality(int index) const
    {
        return static_cast<float>(quality_[static_cast<std::size_t>(index)]) / 255.0F;
    }

    [[nodiscard]] bool wellSeen(int index) const { return quality(index) >= params_.well_seen; }

    /// Surface id at a cell, or -1.
    [[nodiscard]] int surfaceAt(int index) const
    {
        return surface_at_[static_cast<std::size_t>(index)];
    }

    [[nodiscard]] float surfaceQuality(int index) const
    {
        return static_cast<float>(surface_quality_[static_cast<std::size_t>(index)]) / 255.0F;
    }

    [[nodiscard]] bool surfaceWellSeen(int index) const
    {
        return surfaceQuality(index) >= params_.well_seen;
    }

    /// Height of the surface with slot @p slot, as returned by surfaceAt().
    [[nodiscard]] double surfaceHeight(int slot) const
    {
        return surface_height_[static_cast<std::size_t>(slot)];
    }

    /// Unit normal of a face target, pointing into free space.
    [[nodiscard]] cv::Vec2f faceNormal(int index) const
    {
        return normal_[static_cast<std::size_t>(index)];
    }

    [[nodiscard]] bool unobservable(int index) const
    {
        return (flags_[static_cast<std::size_t>(index)] & kUnobservableFlag) != 0;
    }

    /// Marks a target no reachable standing pose can see, so the planner stops trying.
    void markUnobservable(int index)
    {
        flags_[static_cast<std::size_t>(index)] |= kUnobservableFlag;
    }

    /// Counts a visit that was predicted to cover the target; returns the new count.
    int countAttempt(int index);

    [[nodiscard]] std::uint16_t views(int index) const
    {
        return views_[static_cast<std::size_t>(index)];
    }

    /// Bit i set: seen from azimuth sector i of 8, measured from the target towards the camera.
    [[nodiscard]] std::uint8_t directions(int index) const
    {
        return directions_[static_cast<std::size_t>(index)];
    }

    /// Whether a target still needs a view: exists, not seen well enough, not written off.
    [[nodiscard]] bool pending(int index) const
    {
        const TargetKind k = kind(index);
        return k != TargetKind::kNone && !wellSeen(index) && !unobservable(index);
    }

    [[nodiscard]] bool surfaceUnobservable(int index) const
    {
        return (flags_[static_cast<std::size_t>(index)] & kSurfaceUnobservableFlag) != 0;
    }

    void markSurfaceUnobservable(int index)
    {
        flags_[static_cast<std::size_t>(index)] |= kSurfaceUnobservableFlag;
    }

    [[nodiscard]] bool surfacePending(int index) const
    {
        return surfaceAt(index) >= 0 && !surfaceWellSeen(index) && !surfaceUnobservable(index);
    }

    /**
     * @brief Counts per label of @p labels (CV_32S, same geometry); element 0 is unlabelled space.
     *
     * Targets written off unseen are left out of the floor, face and surface counts, which are
     * therefore of what can be seen, and counted in `unobservable` instead.
     */
    [[nodiscard]] std::vector<CoverageTally> tally(const cv::Mat& labels, int count) const;

    /// Quality as 0..100 per cell, -1 where there is no target: an OccupancyGrid payload.
    [[nodiscard]] std::vector<std::int8_t> qualityGrid() const;

    [[nodiscard]] const GridGeometry&   geometry() const { return geometry_; }
    [[nodiscard]] const cv::Mat&        cells() const { return cells_; }
    [[nodiscard]] const CoverageParams& params() const { return params_; }

    /// Raw per-cell layers, for persistence: quality, surface quality, flags, directions.
    [[nodiscard]] std::array<const std::vector<std::uint8_t>*, 4> layers() const
    {
        return { &quality_, &surface_quality_, &flags_, &directions_ };
    }

    /// Restores layers saved from the same geometry; returns false on a size mismatch.
    bool restoreLayers(
        const std::vector<std::uint8_t>& quality, const std::vector<std::uint8_t>& surface_quality,
        const std::vector<std::uint8_t>& flags, const std::vector<std::uint8_t>& directions);

private:
    static constexpr std::uint8_t kUnobservableFlag        = 0x01U;
    static constexpr int          kAttemptShift            = 1;
    static constexpr std::uint8_t kAttemptMask             = 0x0EU;
    static constexpr std::uint8_t kSurfaceUnobservableFlag = 0x10U;

    struct PixelRay
    {
        float x;        ///< (u - cx) / fx
        float y;        ///< (v - cy) / fy
        float stretch;  ///< Range per metre of depth: |(x, y, 1)|.
        float edge;     ///< edgeWeight() of the pixel.
        int   u;
        int   v;
    };

    void rebuildRays(const Intrinsics& intrinsics);
    void credit(std::size_t index, float quality, const Eigen::Vector3d& towards_camera);
    void rebuildNearestFaces();
    /// The face a wall sample belongs to: the nearest one that faces the camera, or -1.
    [[nodiscard]] int faceFacing(
        const Eigen::Vector3d& point, const Eigen::Vector3d& towards, std::size_t index) const;

    CoverageParams params_;
    GridGeometry   geometry_;
    cv::Mat        cells_;

    std::vector<std::uint8_t>  kind_;
    std::vector<std::uint8_t>  quality_;
    std::vector<std::uint8_t>  surface_quality_;
    std::vector<std::uint8_t>  flags_;
    std::vector<std::uint8_t>  directions_;
    std::vector<std::uint16_t> views_;
    std::vector<std::uint32_t> last_frame_;
    std::vector<cv::Vec2f>     normal_;
    std::vector<int>           nearest_face_;  // Per cell, the face within face_reach, or -1.
    std::vector<int>           surface_at_;
    std::vector<double>        surface_height_;
    std::uint32_t              frame_ = 0;

    Intrinsics            rays_for_;
    int                   rays_stride_ = 0;
    std::vector<PixelRay> rays_;
};

}  // namespace g1_world_model

#endif  // G1_WORLD_MODEL__COVERAGE_MAP_HPP_
