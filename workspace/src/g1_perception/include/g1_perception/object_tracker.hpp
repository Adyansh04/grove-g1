#ifndef G1_PERCEPTION__OBJECT_TRACKER_HPP_
#define G1_PERCEPTION__OBJECT_TRACKER_HPP_

/**
 * @file object_tracker.hpp
 * @brief Keeps one object id pointing at one physical object across frames.
 *
 * A detector answers "red cube" with however many instances it sees, in whatever order it found
 * them. Skills address objects by id, so the id has to outlive the frame: `red_cube_0` must be
 * the same cube next second, or a place step re-aims at the wrong one.
 */

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "g1_perception/object_geometry.hpp"

namespace g1_perception
{

/// One detection to be named, positioned in a frame that does not move with the camera.
struct Observation
{
    std::string phrase;
    Point3      position;
};

class ObjectTracker
{
public:
    /**
     * @param match_radius_m How far an object may move between frames and still be itself. Too
     *                       large and two neighbours swap; too small and every frame invents a
     *                       new id.
     * @param timeout_s      How long an unseen track keeps its index reserved.
     */
    ObjectTracker(double match_radius_m, double timeout_s);

    /**
     * @brief Names every observation, creating and retiring tracks as needed.
     *
     * Greedy nearest neighbour: the closest pair inside the radius wins, then the next, which is
     * enough for a handful of objects on a table and has no failure mode worth a Hungarian solver.
     *
     * @param now_s Monotonic seconds; only differences matter.
     * @return One id per observation, in the order they were given.
     */
    std::vector<std::string> update(std::span<const Observation> observations, double now_s);

    /// True when exactly one live track answers to @p phrase, so the bare phrase is unambiguous.
    [[nodiscard]] bool isSoleTrackFor(std::string_view phrase) const;

    /// Id of a track, as `<slug>_<index>`.
    [[nodiscard]] static std::string idFor(std::string_view phrase, std::uint32_t index);

private:
    struct Track
    {
        std::string   phrase;
        std::uint32_t index{ 0 };
        Point3        position;
        double        last_seen_s{ 0.0 };
    };

    void          retireStale(double now_s);
    std::uint32_t lowestFreeIndex(std::string_view phrase) const;

    double             match_radius_m_;
    double             timeout_s_;
    std::vector<Track> tracks_;
};

}  // namespace g1_perception

#endif  // G1_PERCEPTION__OBJECT_TRACKER_HPP_
