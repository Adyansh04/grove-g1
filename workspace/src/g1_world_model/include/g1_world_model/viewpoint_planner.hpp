#ifndef G1_WORLD_MODEL__VIEWPOINT_PLANNER_HPP_
#define G1_WORLD_MODEL__VIEWPOINT_PLANNER_HPP_

/**
 * @file viewpoint_planner.hpp
 * @brief Where to stand and which way to look next.
 *
 * Two modes over the same machinery. Frontier mode closes the map: it walks to the edge of the
 * known space. Coverage mode makes the camera see what the map holds: it samples standing
 * poses, predicts which pending targets each heading would see well enough, keeps the few
 * headings that add the most, and ranks poses by gain per second of humanoid time. Travel is
 * slow and imprecise on this gait while turning in place is cheap and exact, so a viewpoint is
 * one position and a short list of headings rather than a single pose.
 *
 * Receding horizon: only the best next viewpoint is returned, and the next request plans again
 * from what the camera actually measured.
 */

#include <cstdint>
#include <functional>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

#include "g1_world_model/coverage_map.hpp"
#include "g1_world_model/grid.hpp"

namespace g1_world_model
{

/// A camera's mount, as far as prediction needs it. Read from TF and camera_info.
struct CameraModel
{
    double height         = 1.21;  ///< Optical centre above the floor, m.
    double pitch          = 0.83;  ///< Optical axis below the horizon, rad.
    double horizontal_fov = 1.55;  ///< rad.
    double vertical_fov   = 1.01;  ///< rad.
    double yaw            = 0.0;   ///< Optical axis from the base's heading, rad, left positive.
};

struct PlannerParams
{
    double robot_radius       = 0.45;  ///< Clearance a standing pose needs, m.
    double clearance_margin   = 0.25;  ///< Added to robot_radius for candidate poses, m.
    double frontier_clearance = 0.35;  ///< Added to robot_radius for frontier goals, m.
    double travel_margin      = 0.05;  ///< Added to robot_radius for paths, m.
    double candidate_spacing  = 0.40;  ///< Lattice step of candidate positions, m.
    int    ray_count          = 240;   ///< Prediction rays per full turn.
    int    heading_count      = 16;    ///< Headings tried per position.
    int    max_headings       = 3;     ///< Headings kept per viewpoint.
    double min_heading_gain   = 20.0;  ///< Weighted targets a further heading must add.
    double min_viewpoint_gain = 40.0;  ///< Below this nothing is worth the walk: done.
    double min_viewpoint_rate = 2.0;   ///< Weighted targets per second, likewise...
    double room_floor_goal    = 0.9;   ///< ...once every room has this share of floor seen
    double room_face_goal     = 0.8;   ///< and this of faces; until then its targets alone.
    double face_weight        = 2.0;   ///< Objects stand against faces: they count double.
    double surface_weight     = 3.0;   ///< And sit on surfaces.
    double travel_speed       = 0.35;  ///< Average over a Nav2 goal on this gait, m/s.
    double turn_speed         = 0.8;   ///< In-place yaw rate, rad/s.
    double dwell_time         = 2.5;   ///< Per heading: detections arrive about 1 s late, s.
    double goal_overhead      = 4.0;   ///< Per viewpoint: planning, starting, settling, s.
    double room_switch_factor = 3.0;   ///< Cost multiplier for leaving a room with work left.
    int    max_candidates     = 400;   ///< Positions evaluated per request.
    /// Viewpoints that together see the most, taken up to this many; the next is the first stop
    /// of the shortest walk through them. 1 picks by gain per second alone.
    int    tour_size             = 8;
    double min_frontier_size     = 0.75;  ///< Shorter frontiers are map noise, m.
    double frontier_stand_back   = 0.5;   ///< Frontier goals stand this far from its middle, m.
    double frontier_gap_close    = 0.15;  ///< Gaps between beams closed before frontiers, m.
    double min_frontier_unknown  = 4.0;   ///< Unknown area a frontier must open onto, m2.
    double frontier_seen_radius  = 1.5;  ///< Unknown seen this near and still unknown is shadow, m.
    int    frontier_stall_visits = 4;    ///< Frontier visits over which the map has to grow...
    double min_frontier_growth   = 12.0;  ///< ...by this much known area, m2, or the pass ends.
    double pocket_min_area       = 0.1;   ///< Unknown inside the building this big gets a look, m2.
    double pocket_look_range     = 3.0;   ///< Farthest a pocket is looked into from, m.
    double pocket_min_edge       = 0.4;   ///< Pocket edge a look sees that no other look does, m.
    int    max_attempts          = 2;  ///< Predicted-but-unseen visits before a target is dropped.
    double blacklist_radius      = 0.5;  ///< Around a viewpoint Nav2 could not reach, m.
    int    max_room_failures     = 3;  ///< Unreached viewpoints in a room never entered: given up.
    int    max_failures_in_a_row = 5;  ///< Anywhere: the robot is stuck, not the rooms unreachable.
};

struct Viewpoint
{
    std::uint32_t       id = 0;
    double              x  = 0.0;  ///< Map frame, m.
    double              y  = 0.0;
    std::vector<double> headings;    ///< Map-frame yaws, in the order to face them, rad.
    int                 room = 0;    ///< Room label at the position, 0 if none.
    double              gain = 0.0;  ///< Weighted targets predicted to be seen.
    double              cost = 0.0;  ///< Predicted time, s.
    std::vector<int>    predicted;   ///< Target cells it should see; surfaces offset by cells.
};

enum class PlanStatus : std::uint8_t
{
    kViewpoint,
    kDone,
    kUnavailable,
};

struct Plan
{
    PlanStatus  status = PlanStatus::kUnavailable;
    Viewpoint   viewpoint;
    std::string reason;
};

/// A robot pose on the map, m and rad.
struct Pose2D
{
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

class ViewpointPlanner
{
public:
    explicit ViewpointPlanner(PlannerParams params = {}, CameraModel camera = {});
    /// Cameras turn with the base: a heading is scored by what all of them see from it.
    ViewpointPlanner(PlannerParams params, std::vector<CameraModel> cameras);

    /// With none, coverage planning waits for one.
    void setCameras(std::vector<CameraModel> cameras) { cameras_ = std::move(cameras); }

    [[nodiscard]] const std::vector<CameraModel>& cameras() const { return cameras_; }

    [[nodiscard]] const PlannerParams& params() const { return params_; }

    /// The frontier to walk to next, or kDone once no reachable frontier is left.
    Plan nextFrontier(const cv::Mat& cells, const GridGeometry& geometry, const Pose2D& robot);

    /**
     * @brief Where to look into the next unknown pocket inside the building, or kDone once every
     *        look is taken. For after the camera pass: the frontier pass leaves corners such as
     *        the floor behind a wardrobe as too small to walk to, and the camera pass maps most.
     *
     * A pocket is unknown inside @p inside that borders free floor. The first call plans the
     * looks, each a spot that sees the most pocket edge no other look has, straight across free
     * floor; each call then walks the shortest way through those left, and drops any whose edge
     * is mapped.
     * Unknown found after the plan is left: chasing each look's crumbs cost run 41 nine minutes.
     *
     * @param inside CV_8U on @p cells' grid, non-zero inside the building (the floor plan's
     *               known cells).
     */
    Plan nextPocketLook(
        const cv::Mat& cells, const cv::Mat& inside, const GridGeometry& geometry,
        const Pose2D& robot);

    /**
     * @brief Remembers that the robot stood at @p robot and looked around; report() calls it.
     *
     * Unknown in plain view from there that the caught-up map (mapUpdated()) still lacks is
     * shadow the LiDAR cannot reach, and frontiers onto it are dropped.
     */
    void recordPose(const Pose2D& robot);

    /// The map now holds all the LiDAR saw from the poses recorded so far.
    void mapUpdated();

    /**
     * @brief The viewpoint with the most predicted coverage per second.
     *
     * Also writes off targets that no reachable pose can see, once nothing else is left; done
     * stays done, wherever the pocket looks after it take the robot.
     *
     * @param coverage    Targets and their qualities; updated only by write-offs.
     * @param room_labels CV_32S room labels on the coverage grid, or empty.
     * @param robot       Where the robot stands now.
     */
    Plan nextCoverage(CoverageMap& coverage, const cv::Mat& room_labels, const Pose2D& robot);

    /**
     * @brief Records how an issued viewpoint went.
     *
     * Reached: every target it was meant to cover and still has not gets an attempt, and is
     * written off after max_attempts. Not reached: the spot is avoided, after max_room_failures
     * in a room never entered the whole room, and after max_failures_in_a_row anywhere nothing is
     * planned until the robot has moved.
     */
    void report(CoverageMap& coverage, std::uint32_t id, bool reached);

    /// A viewpoint outside the coverage bookkeeping: stand at (@p x, @p y) and face @p heading.
    [[nodiscard]] Viewpoint glance(double x, double y, double heading)
    {
        Viewpoint viewpoint;
        viewpoint.id       = next_id_++;
        viewpoint.x        = x;
        viewpoint.y        = y;
        viewpoint.headings = { heading };
        return viewpoint;
    }

    /**
     * @brief Computes clearance and the travel field from @p robot without planning, for callers
     * that only need to know what is reachable (approach poses).
     */
    void prepare(const cv::Mat& cells, const GridGeometry& geometry, const Pose2D& robot)
    {
        computeTraversable(cells, geometry);
        computeTravel(geometry, robot);
    }

    /// Distance to the nearest non-free cell, m, CV_32F; from the last request or prepare().
    [[nodiscard]] const cv::Mat& clearance() const { return clearance_; }

    /// Path length from the robot per cell, m; infinity where unreachable.
    [[nodiscard]] const std::vector<float>& travel() const { return travel_; }

private:
    /// A pending target a prediction ray reaches, with everything but the heading folded in.
    struct Hit
    {
        int        target;  ///< Cell index; surfaces are offset by the cell count.
        float      base;    ///< Range and incidence weight: quality at the image centre.
        float      rho_v;   ///< Normalised vertical image offset of the part seen.
        TargetKind kind;
    };

    struct Blacklisted
    {
        double x;
        double y;
        int    failures;
    };

    struct Outcome
    {
        double x;
        double y;
        bool   reached;
    };

    struct IssuedFrontier
    {
        std::uint32_t id;
        double        target_x;
        double        target_y;
    };

    struct PocketLook
    {
        Viewpoint                view;
        std::vector<cv::Point2d> edge;  ///< The pocket edge it was planned for, map frame.
    };

    /// True, with @p plan set to unavailable, while the robot stands where it got stuck.
    bool stuck(const Pose2D& robot, Plan& plan);
    void computeTraversable(const cv::Mat& cells, const GridGeometry& geometry);
    void computeTravel(const GridGeometry& geometry, const Pose2D& robot);
    /// Path length from (@p x, @p y) to every cell over the traversable grid, into @p field.
    void
    travelFrom(const GridGeometry& geometry, double x, double y, std::vector<float>& field) const;
    /// The first stop of the shortest walk through the few @p options that together see the
    /// most, each target counted at @p worth; empty when fewer than two are worth a visit.
    [[nodiscard]] std::optional<Viewpoint> tourStart(
        std::vector<Viewpoint> options, const GridGeometry& geometry,
        const std::function<double(int)>& worth) const;
    /// Index of the first of @p stops (not empty) on the shortest walk from the robot through
    /// them all: nearest first, then 2-opt, over walking distances.
    [[nodiscard]] std::size_t
    shortestWalkStart(const std::vector<Viewpoint>& stops, const GridGeometry& geometry) const;

    void castRays(
        const CoverageMap& coverage, const CameraModel& camera, double x, double y,
        std::vector<Hit>& hits, std::vector<int>& offsets) const;
    [[nodiscard]] double blacklistFactor(double x, double y) const;
    [[nodiscard]] double turnTime(double from_yaw, std::vector<double>& headings) const;
    /// Greedy cover of the pockets' edge by spots; needs clearance_ and travel_.
    [[nodiscard]] std::vector<PocketLook>
    planPocketLooks(const cv::Mat& cells, const cv::Mat& inside, const GridGeometry& geometry) const;

    PlannerParams            params_;
    std::vector<CameraModel> cameras_;

    cv::Mat            clearance_;    // CV_32F, m
    cv::Mat            traversable_;  // CV_8U
    std::vector<float> travel_;       // Path length from the robot, m; infinity if unreachable.

    std::uint32_t               next_id_  = 1;
    bool                        focusing_ = false;  // Past the rate cut, on the short rooms.
    std::vector<Viewpoint>      issued_;
    std::vector<IssuedFrontier> issued_frontiers_;
    std::vector<Blacklisted>    blacklist_;
    std::vector<Blacklisted>    exhausted_frontiers_;
    std::vector<Outcome>        outcomes_;        // Of coverage viewpoints, oldest first.
    std::vector<cv::Point2d>    trail_;           // Where the robot looked from, map frame.
    std::vector<cv::Point2d>    pending_trail_;   // Not yet in the map.
    std::vector<double>         frontier_known_;  // Known area as each frontier was issued, m2.
    int                         failures_in_a_row_ = 0;
    std::optional<Pose2D>       stuck_at_;

    // Why the camera pass ended, once it has: seen from where a pocket look leaves the robot, a
    // leftover viewpoint can pay again, and run 47 walked back for seven of them.
    std::optional<std::string> coverage_done_;
    // Planned once, as the camera pass ends.
    std::optional<std::vector<PocketLook>> pocket_looks_;

    // Per-target marks, compared against a counter so they never need clearing.
    std::vector<std::uint32_t> chosen_mark_;
    std::vector<std::uint32_t> heading_mark_;
    std::uint32_t              chosen_value_  = 0;
    std::uint32_t              heading_value_ = 0;
};

}  // namespace g1_world_model

#endif  // G1_WORLD_MODEL__VIEWPOINT_PLANNER_HPP_
