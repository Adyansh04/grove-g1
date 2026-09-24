#ifndef G1_PERCEPTION__OBJECT_TRACKER_HPP_
#define G1_PERCEPTION__OBJECT_TRACKER_HPP_

/**
 * @file object_tracker.hpp
 * @brief Keeps one object id pointing at one physical object across frames.
 *
 * A detector answers a phrase with however many instances it sees, in any order. Skills address
 * objects by id, so `red_block_0` has to be the same object next second.
 */

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
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
     * @param match_radius_m How far an object may move between frames and still be itself, when
     *                       more than one object answers to its phrase.
     * @param timeout_s      How long an unseen track keeps its index reserved.
     */
    ObjectTracker(double match_radius_m, double timeout_s);

    /**
     * @brief Names every observation, creating and retiring tracks as needed.
     *
     * A phrase seen once with one track is that track at any distance, so a jittery pose cannot
     * split one object in two. Otherwise closest pairs within the radius match first.
     *
     * @param now_s Monotonic seconds; only differences matter.
     * @return One id per observation, in the order they were given.
     */
    std::vector<std::string> update(std::span<const Observation> observations, double now_s);

    /**
     * @brief The id the bare phrase stands for in the latest update, if any.
     *
     * The alias belongs to the first track that was seen alone under its phrase, until that track
     * retires, so it never jumps between objects. It is withheld from any frame where that track
     * is unseen or another track of the phrase is seen too.
     */
    [[nodiscard]] std::optional<std::string> aliasFor(std::string_view phrase) const;

    /// Id of a track, as `<slug>_<index>`.
    [[nodiscard]] static std::string idFor(std::string_view phrase, std::uint32_t index);

    /// The slug an id was made from: `red_block_3` gives `red_block`; anything else comes back whole.
    [[nodiscard]] static std::string_view phraseOf(std::string_view id);

private:
    struct Track
    {
        std::string   phrase;
        std::uint32_t index{ 0 };
        Point3        position;
        double        last_seen_s{ 0.0 };
        bool          seen_in_latest{ false };  ///< Matched or created by the latest update.
    };

    void          retireStale(double now_s);
    std::uint32_t lowestFreeIndex(std::string_view phrase) const;

    double             match_radius_m_;
    double             timeout_s_;
    std::vector<Track> tracks_;
    /// Phrase to the index of the track its bare alias belongs to.
    std::map<std::string, std::uint32_t, std::less<>> alias_;
};

}  // namespace g1_perception

#endif  // G1_PERCEPTION__OBJECT_TRACKER_HPP_
