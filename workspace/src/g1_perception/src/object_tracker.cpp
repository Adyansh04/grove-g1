/**
 * @file object_tracker.cpp
 * @brief Frame-to-frame association and the bare-phrase alias.
 */

#include "g1_perception/object_tracker.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iterator>

namespace g1_perception
{
namespace
{

double distance(const Point3& a, const Point3& b)
{
    const Point3 gap{ a.x - b.x, a.y - b.y, a.z - b.z };
    return std::sqrt(dot(gap, gap));
}

}  // namespace

ObjectTracker::ObjectTracker(double match_radius_m, double timeout_s)
  : match_radius_m_(match_radius_m)
  , timeout_s_(timeout_s)
{}

std::string ObjectTracker::idFor(std::string_view phrase, std::uint32_t index)
{
    return slugify(phrase) + "_" + std::to_string(index);
}

std::string_view ObjectTracker::phraseOf(std::string_view id)
{
    const std::size_t underscore = id.rfind('_');
    if (underscore == std::string_view::npos || underscore == 0 || underscore + 1 == id.size())
    {
        return id;
    }
    const std::string_view index   = id.substr(underscore + 1);
    const bool             numeric = std::all_of(index.begin(), index.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
    });
    return numeric ? id.substr(0, underscore) : id;
}

void ObjectTracker::retireStale(double now_s)
{
    std::erase_if(tracks_, [this, now_s](const Track& track) {
        return now_s - track.last_seen_s > timeout_s_;
    });
    std::erase_if(alias_, [this](const auto& holder) {
        return std::ranges::none_of(tracks_, [&holder](const Track& track) {
            return track.phrase == holder.first && track.index == holder.second;
        });
    });
}

std::uint32_t ObjectTracker::lowestFreeIndex(std::string_view phrase) const
{
    for (std::uint32_t candidate = 0;; ++candidate)
    {
        const bool taken =
            std::any_of(tracks_.begin(), tracks_.end(), [&phrase, candidate](const Track& track) {
                return track.index == candidate && track.phrase == phrase;
            });
        if (!taken)
        {
            return candidate;
        }
    }
}

std::vector<std::string>
ObjectTracker::update(std::span<const Observation> observations, double now_s)
{
    retireStale(now_s);
    for (Track& track : tracks_)
    {
        track.seen_in_latest = false;
    }

    std::vector<std::string> ids(observations.size());
    std::vector<bool>        named(observations.size(), false);
    std::vector<bool>        matched(tracks_.size(), false);
    const auto               claim = [&](std::size_t o, std::size_t t) {
        Track& track         = tracks_[t];
        track.position       = observations[o].position;
        track.last_seen_s    = now_s;
        track.seen_in_latest = true;
        ids[o]               = idFor(track.phrase, track.index);
        named[o]             = true;
        matched[t]           = true;
    };

    for (std::size_t o = 0; o < observations.size(); ++o)
    {
        const auto same = [&phrase = observations[o].phrase](const auto& item) {
            return item.phrase == phrase;
        };
        if (std::ranges::count_if(observations, same) == 1 &&
            std::ranges::count_if(tracks_, same) == 1)
        {
            claim(
                o,
                static_cast<std::size_t>(
                    std::distance(tracks_.begin(), std::ranges::find_if(tracks_, same))));
        }
    }

    // Closest pair first, so a confident match cannot be stolen by a worse one considered earlier.
    while (true)
    {
        double      best             = match_radius_m_;
        std::size_t best_observation = observations.size();
        std::size_t best_track       = tracks_.size();
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
                    best             = gap;
                    best_observation = o;
                    best_track       = t;
                }
            }
        }
        if (best_observation == observations.size())
        {
            break;
        }
        claim(best_observation, best_track);
    }

    for (std::size_t o = 0; o < observations.size(); ++o)
    {
        if (named[o])
        {
            continue;
        }
        Track track;
        track.phrase         = observations[o].phrase;
        track.index          = lowestFreeIndex(track.phrase);
        track.position       = observations[o].position;
        track.last_seen_s    = now_s;
        track.seen_in_latest = true;
        ids[o]               = idFor(track.phrase, track.index);
        tracks_.push_back(std::move(track));
    }

    for (const Track& track : tracks_)
    {
        const bool alone = std::ranges::count_if(tracks_, [&track](const Track& other) {
                               return other.seen_in_latest && other.phrase == track.phrase;
                           }) == 1;
        if (track.seen_in_latest && alone && !alias_.contains(track.phrase))
        {
            alias_.emplace(track.phrase, track.index);
        }
    }
    return ids;
}

std::optional<std::string> ObjectTracker::aliasFor(std::string_view phrase) const
{
    const auto holder = alias_.find(phrase);
    if (holder == alias_.end())
    {
        return std::nullopt;
    }
    std::optional<std::string> id;
    for (const Track& track : tracks_)
    {
        if (!track.seen_in_latest || track.phrase != phrase)
        {
            continue;
        }
        if (track.index != holder->second || id.has_value())
        {
            return std::nullopt;
        }
        id = idFor(track.phrase, track.index);
    }
    return id;
}

}  // namespace g1_perception
