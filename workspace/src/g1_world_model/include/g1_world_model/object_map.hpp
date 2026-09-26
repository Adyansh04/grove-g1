#ifndef G1_WORLD_MODEL__OBJECT_MAP_HPP_
#define G1_WORLD_MODEL__OBJECT_MAP_HPP_

/**
 * @file object_map.hpp
 * @brief Objects fused from detector masks and depth, in the map frame.
 *
 * Each mask is lifted to 3D, voxelised and cut to its largest connected cluster, then matched
 * to the mapped objects near it on geometric overlap and on what it was called (label votes, or
 * embeddings when both sides have them), after ConceptGraphs. Three changes fit this robot:
 * a same-label detection a few decimetres off still matches, because AMCL's map drifts by that
 * much between visits; overlap is measured from the detection's side, so a mug on a table is not
 * swallowed by the table; and an object whose place is seen through, depth reaching well past
 * where it stood, collects misses until it is marked stale and then removed (DynaMem).
 */

#include <Eigen/Geometry>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "g1_world_model/coverage_map.hpp"

namespace g1_world_model
{

struct ObjectMapParams
{
    double voxel              = 0.04;  ///< Voxel edge, m.
    double min_depth          = 0.25;  ///< Mask pixels nearer than this are ignored, m.
    double max_depth          = 4.5;   ///< And farther than this: depth is too coarse there, m.
    double floor_cut          = 0.03;  ///< Points this low are floor bleeding into a mask, m.
    int    max_points         = 4000;  ///< Mask pixels lifted per detection, evenly subsampled.
    int    min_voxels         = 4;     ///< Smaller clusters are noise.
    double min_score          = 0.30;  ///< Weaker detections never start an object.
    double match_radius       = 1.5;   ///< Only objects this close are candidates, m.
    double min_match          = 0.55;  ///< Association score to join an object.
    double strong_overlap     = 0.60;  ///< Overlap that joins whatever the label says...
    double strong_share       = 0.20;  ///< ...if the mask is not a sliver of the object.
    double geometry_weight    = 0.6;
    double semantic_weight    = 0.4;
    double position_tolerance = 0.35;   ///< Same label, this near the box: localisation drift, m.
    double merge_distance     = 1.5;    ///< Merge pass: centres this close are compared, m.
    double merge_overlap      = 0.5;    ///< And merged above this overlap, either way round.
    double merge_similarity   = 0.8;    ///< Embedding cosine that stands in for a shared label.
    double merge_gap          = 0.10;   ///< Same label, boxes this close: sides of one object, m.
    double support_merge_gap  = 1.0;    ///< For tables and shelves, seen as legs and ends, m.
    double max_merged_extent  = 3.5;    ///< The support gap joins nothing longer, m.
    int    merge_every        = 10;     ///< Frames between merge passes.
    int    max_voxels         = 20000;  ///< Per object; big furniture is thinned beyond it.
    int    stale_after        = 4;      ///< Misses before an object is stale.
    int    remove_after       = 10;     ///< And removed.
    int    min_observations   = 2;      ///< Fewer, this long after the first: a fragment, dropped.
    double confirm_s          = 120.0;  ///< s.
    double box_trim           = 0.02;   ///< Share of footprint points left off each box side.
    double turned_area_ratio  = 0.7;    ///< A box leaves the walls only this much tighter.
    double miss_margin        = 0.30;   ///< Depth this far behind an object sees through it, m;
                                        ///< less for small objects, down to one voxel.

    /// Labels whose tops hold other objects: tables, shelves and the like.
    std::vector<std::string> support_labels{ "table",     "dining table", "coffee table",
                                             "desk",      "counter",      "shelf",
                                             "bookshelf", "nightstand",   "cabinet",
                                             "dresser",   "workbench",    "side table" };
};

/// One mask from one frame, before it is lifted.
struct MaskInput
{
    std::string         label;
    float               score  = 0.0F;
    int                 x      = 0;  ///< Region of interest in the image, px.
    int                 y      = 0;
    int                 width  = 0;
    int                 height = 0;
    const std::uint8_t* mask   = nullptr;  ///< width x height, 0 outside the instance.
    std::vector<float>  embedding;         ///< Optional, L2-normalised.
};

/// The frame the masks were cut from.
struct FrameInput
{
    double            stamp = 0.0;  ///< s.
    DepthImage        depth;
    Intrinsics        intrinsics;
    Eigen::Isometry3d map_from_camera = Eigen::Isometry3d::Identity();
};

enum class ObjectState : std::uint8_t
{
    kActive  = 0,
    kStale   = 1,
    kRemoved = 2,
};

/// The best view of an object so far, for crops and captions.
struct BestView
{
    double stamp  = 0.0;
    int    x      = 0;
    int    y      = 0;
    int    width  = 0;
    int    height = 0;
    double score  = 0.0;  ///< Mask area weighted towards the image centre.
};

struct MappedObject
{
    int                          id = 0;
    std::map<std::string, float> votes;
    std::string                  name;  ///< From the describer; empty until described.
    std::string                  caption;
    std::vector<float>           embedding;  ///< Running mean, re-normalised.
    int                          embedding_count = 0;

    std::vector<std::uint64_t> voxels;  ///< Sorted voxel keys.
    Eigen::Vector3d            centroid   = Eigen::Vector3d::Zero();
    Eigen::Vector2d            box_centre = Eigen::Vector2d::Zero();  ///< Footprint, map frame.
    Eigen::Vector2d            seen_from  = Eigen::Vector2d::Zero();  ///< First camera position.
    Eigen::Vector2d            box_size   = Eigen::Vector2d::Zero();  ///< Along the box axes, m.
    double                     box_yaw    = 0.0;
    double                     z_min      = 0.0;
    double                     z_max      = 0.0;
    /// A mask of it stopped short of its image's upper edge: z_max is its own top, not merely
    /// as high as a camera looked.
    bool top_seen = false;

    int         observations = 0;
    double      first_seen   = 0.0;
    double      last_seen    = 0.0;
    int         misses       = 0;
    ObjectState state        = ObjectState::kActive;
    int         support      = 0;  ///< Id of the object it rests on, 0 if none.
    BestView    best_view;

    /// The label with the most votes.
    [[nodiscard]] std::string label() const;

    /// Share of all votes the leading label holds, 0..1.
    [[nodiscard]] float confidence() const;

    [[nodiscard]] double height() const { return z_max - z_min; }
};

/// What happened to one mask of a frame.
struct MaskOutcome
{
    int  object    = 0;  ///< Id of the object it joined or started; 0 if dropped.
    bool created   = false;
    bool best_view = false;  ///< It became the object's best view: take a new crop.
};

class ObjectMap
{
public:
    explicit ObjectMap(ObjectMapParams params = {});

    /// The walls' yaw (dominantAxis), which boxes lie along; refits every object.
    void setFrame(double yaw);

    /**
     * @brief Fuses one frame's masks.
     *
     * @return One outcome per mask, in order.
     */
    std::vector<MaskOutcome> integrate(const std::vector<MaskInput>& masks, const FrameInput& frame);

    /// Objects whose place @p frame sees through collect a miss; the others are left alone.
    void checkAbsence(const FrameInput& frame, const std::vector<int>& matched);

    /// Merges duplicates: objects close together that overlap and agree on what they are.
    int mergeDuplicates();

    /// Drops objects seen fewer than min_observations times in the confirm_s after the first.
    int pruneUnconfirmed(double now);

    /// Seen often enough to be real rather than a glimpse still waiting to be confirmed.
    [[nodiscard]] bool confirmed(const MappedObject& object) const
    {
        return object.observations >= params_.min_observations;
    }

    /// Unconfirmed objects, less pieces of a confirmed one of their label (those merge into it):
    /// what a second look from where they were seen confirms or drops.
    [[nodiscard]] std::vector<const MappedObject*> glimpses() const;

    /// Labels of what pruneUnconfirmed() has dropped since the last call.
    [[nodiscard]] std::vector<std::string> takeDropped() { return std::exchange(dropped_, {}); }

    /// Recomputes which object rests on which.
    void relateSupports();

    [[nodiscard]] const std::vector<MappedObject>& objects() const { return objects_; }

    [[nodiscard]] MappedObject*       find(int id);
    [[nodiscard]] const MappedObject* find(int id) const;

    /// Replaces everything, e.g. from a saved world. Ids continue after the highest.
    void restore(std::vector<MappedObject> objects);

    /// Support-type objects as coverage surfaces, top faces only.
    [[nodiscard]] std::vector<Surface> surfaces() const;

    [[nodiscard]] bool isSupport(const std::string& label) const;

    [[nodiscard]] const ObjectMapParams& params() const { return params_; }

    /// Voxel key of a point, and the centre of a key; exposed for persistence and tests.
    [[nodiscard]] std::uint64_t   keyOf(const Eigen::Vector3d& point) const;
    [[nodiscard]] Eigen::Vector3d centreOf(std::uint64_t key) const;

private:
    struct Lifted
    {
        std::vector<std::uint64_t> voxels;
        Eigen::Vector3d            centroid = Eigen::Vector3d::Zero();
        bool                       top_seen = false;  // See MappedObject::top_seen.
    };

    [[nodiscard]] std::optional<Lifted> lift(const MaskInput& mask, const FrameInput& frame) const;
    [[nodiscard]] double
    overlap(const std::vector<std::uint64_t>& from, const std::vector<std::uint64_t>& into) const;
    [[nodiscard]] double proximity(const Lifted& detection, const MappedObject& object) const;
    [[nodiscard]] static double similarity(const std::vector<float>& a, const std::vector<float>& b);
    void absorb(
        MappedObject& object, const Lifted& detection, const MaskInput& mask,
        const FrameInput& frame, MaskOutcome& outcome);
    void refreshShape(MappedObject& object) const;
    void mergeInto(MappedObject& keep, MappedObject& drop) const;

    ObjectMapParams           params_;
    std::vector<MappedObject> objects_;
    std::vector<std::string>  dropped_;
    int                       next_id_   = 1;
    int                       frames_    = 0;
    double                    frame_yaw_ = 0.0;
};

struct MapFitParams
{
    double reach       = 0.15;  ///< A blob this near an object's box is under it, m.
    double max_growth  = 3.0;   ///< A share larger than this many boxes is something bigger.
    double min_share   = 0.25;  ///< And one smaller than this share is a leg, not the object.
    double max_stretch = 1.0;   ///< Cells further than this outside an object's box are not its, m.
    double edge_bleed  = 0.05;  ///< The scan's blur past a side that faces free floor, m.
    /// A box more than this share of it on floor the scan saw free has drifted: the map's
    /// outline under it replaces it, if at least min_adrift_share, m2; a plant's pot, alone in
    /// the scan under its leaves, is not the plant.
    double adrift_free      = 0.5;
    double min_adrift_share = 0.04;
    double wall_reach       = 0.2;  ///< Furthest a side reaches back to the wall behind it, m.
};

/**
 * @brief Boxes for floor objects from the furniture the map shows, sharper than fused voxels.
 *
 * Each furniture cell goes to the box it lies deepest in, counted in box widths, or outside them
 * all to the nearest for its size; an object keeps its share only when that is about its own size.
 *
 * @param furniture CV_8U on @p geometry, non-zero where the map is occupied but not by a wall.
 * @param plan      CV_8U Cell values of the floor plan, to tell a drifted box; may be empty.
 * @return Fitted boxes by object id, each along its object's own box axes.
 */
[[nodiscard]] std::map<int, Footprint> fitToMap(
    const std::vector<MappedObject>& objects, const cv::Mat& furniture,
    const GridGeometry& geometry, const MapFitParams& params = {}, const cv::Mat& plan = {});

/**
 * @brief Moves each side of fitted boxes to where the plan says the object ends.
 *
 * A side facing free floor comes in by edge_bleed, the scan's blur. A side facing solid that no
 * wall return marks, with the wall right behind it, goes out to the wall: the wall's band took
 * the back of a counter with it. A side facing anything else stays.
 *
 * @param plan  CV_8U Cell values of the floor plan the boxes were fitted on.
 * @param walls CV_8U, non-zero where the LiDAR hit wall height.
 */
void settleSides(
    std::map<int, Footprint>& fitted, const cv::Mat& plan, const cv::Mat& walls,
    const GridGeometry& geometry, const MapFitParams& params = {});

}  // namespace g1_world_model

#endif  // G1_WORLD_MODEL__OBJECT_MAP_HPP_
