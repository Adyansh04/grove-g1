#ifndef G1_WORLD_MODEL__WORLD_STORE_HPP_
#define G1_WORLD_MODEL__WORLD_STORE_HPP_

/**
 * @file world_store.hpp
 * @brief Saving and loading the world model next to the map it was built on.
 *
 * world.yaml holds what people read (rooms, object labels, names, poses), objects.bin the voxels
 * and embeddings, coverage.bin the camera coverage layers and the occupancy classes they were
 * built on. A world is only reloaded onto the grid it was built on (worldFits()), so a
 * re-mapped building starts clean instead of wearing someone else's objects.
 */

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "g1_world_model/object_map.hpp"

namespace g1_world_model
{

/// What survives about a room; its geometry is re-segmented from the map on load.
struct RoomRecord
{
    std::string id;    ///< "R3".
    std::string name;  ///< "room C", or an operator's name.
    std::string type;
    double      type_confidence = 0.0;
    std::string type_source;
    double      x = 0.0;  ///< A point inside the room, to find it again, m.
    double      y = 0.0;
};

struct WorldSnapshot
{
    GridGeometry              geometry;
    std::vector<std::uint8_t> cells;  ///< Cell classes of the map it was built on.
    int                       next_room = 1;
    std::vector<RoomRecord>   rooms;
    std::vector<MappedObject> objects;
    std::vector<std::uint8_t> quality;
    std::vector<std::uint8_t> surface_quality;
    std::vector<std::uint8_t> flags;
    std::vector<std::uint8_t> directions;
};

/**
 * @brief Writes @p snapshot into @p directory, creating it; each file is replaced atomically.
 * @return An empty string on success, else what failed.
 */
std::string saveWorld(const std::string& directory, const WorldSnapshot& snapshot);

/**
 * @brief Whether a saved world belongs on this map: same grid within a centimetre, and nearly
 * every cell classed alike. A map saved and served back rounds values; a new map does not fit.
 */
bool worldFits(
    const WorldSnapshot& snapshot, const cv::Mat& cells, const GridGeometry& geometry,
    double min_agreement = 0.97);

/**
 * @brief Reads a snapshot written by saveWorld().
 * @param error Set to what failed when nothing is returned.
 */
std::optional<WorldSnapshot> loadWorld(const std::string& directory, std::string& error);

}  // namespace g1_world_model

#endif  // G1_WORLD_MODEL__WORLD_STORE_HPP_
