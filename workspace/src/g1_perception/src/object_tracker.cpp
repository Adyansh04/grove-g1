#include "g1_perception/object_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace g1_perception
{
namespace
{

double distance(const Point3& a, const Point3& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

}  // namespace

ObjectTracker::ObjectTracker(double match_radius_m, double timeout_s)
    : match_radius_m_(match_radius_m), timeout_s_(timeout_s)
{
}

std::string ObjectTracker::idFor(std::string_view phrase, std::uint32_t index)
{
    return slugify(phrase) + "_" + std::to_string(index);
}

void ObjectTracker::retireStale(double now_s)
{
    std::erase_if(tracks_, [this, now_s](const Track& track) {
        return now_s - track.last_seen_s > timeout_s_;
    });
}

std::uint32_t ObjectTracker::lowestFreeIndex(std::string_view phrase) const
{
    for (std::uint32_t candidate = 0;; ++candidate)
    {
        const bool taken = std::any_of(tracks_.begin(), tracks_.end(),
                                       [&phrase, candidate](const Track& track) {
                                           return track.index == candidate && track.phrase == phrase;
                                       });
        if (!taken)
        {
            return candidate;
        }
    }
}

std::vector<std::string> ObjectTracker::update(
    std::span<const Observation> observations, double now_s)
{
    retireStale(now_s);

    std::vector<std::string> ids(observations.size());
    std::vector<bool>        named(observations.size(), false);
    std::vector<bool>        matched(tracks_.size(), false);

    // Closest pair first, so a confident match cannot be stolen by a worse one considered earlier.
    while (true)
    {
        double      best = match_radius_m_;
        std::size_t best_observation = observations.size();
        std::size_t best_track = tracks_.size();
        for (std::size_t o = 0; o < observations.size(); ++o)
        {
            if (named[o])
            {
                continue;
            }
            for (std::size_t t = 0; t < tracks_.size(); ++t)
            {
                if (matched[t] || tracks_[t].phrase != observations[o].phrase)
                {
                    continue;
                }
                const double gap = distance(observations[o].position, tracks_[t].position);
                if (gap <= best)
                {
                    best = gap;
                    best_observation = o;
                    best_track = t;
                }
            }
        }
        if (best_observation == observations.size())
        {
            break;
        }
        Track& track = tracks_[best_track];
        track.position = observations[best_observation].position;
        track.last_seen_s = now_s;
        ids[best_observation] = idFor(track.phrase, track.index);
        named[best_observation] = true;
        matched[best_track] = true;
    }

    for (std::size_t o = 0; o < observations.size(); ++o)
    {
        if (named[o])
        {
            continue;
        }
        Track track;
        track.phrase = observations[o].phrase;
        track.index = lowestFreeIndex(track.phrase);
        track.position = observations[o].position;
        track.last_seen_s = now_s;
        ids[o] = idFor(track.phrase, track.index);
        tracks_.push_back(std::move(track));
    }
    return ids;
}

bool ObjectTracker::isSoleTrackFor(std::string_view phrase) const
{
    return std::count_if(tracks_.begin(), tracks_.end(), [&phrase](const Track& track) {
               return track.phrase == phrase;
           }) == 1;
}

}  // namespace g1_perception
