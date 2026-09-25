#ifndef G1_WORLD_MODEL__ROOM_TYPING_HPP_
#define G1_WORLD_MODEL__ROOM_TYPING_HPP_

/**
 * @file room_typing.hpp
 * @brief What kind of room a region is, from the objects in it and its shape.
 *
 * Naive Bayes over the distinct labels found in the room, from a table of how often each label
 * appears in each room type. Scoring by co-occurring objects types distinctive rooms (kitchen,
 * bedroom, bathroom) well and generic ones poorly (Chen et al. 2022), so a type comes with a
 * probability, and a describer or an operator can overrule it. Long narrow regions with little
 * in them are hallways whatever the table says.
 */

#include <map>
#include <string>
#include <vector>

namespace g1_world_model
{

struct RoomTypeTable
{
    std::vector<std::string>                   types;
    std::vector<double>                        priors;
    std::map<std::string, std::vector<double>> likelihood;  ///< label -> P(present | type).
    double unlisted = 0.08;  ///< P(present | type) for a label the type does not list.

    /// Parses the `room_types` table of a YAML file; throws YAML::Exception on bad input.
    static RoomTypeTable fromYaml(const std::string& path);
};

struct RoomTyping
{
    std::string type;  ///< Empty when nothing in the room says anything.
    double      probability = 0.0;
};

/**
 * @brief Types a room.
 *
 * @param table   Likelihoods.
 * @param labels  Labels of the objects in the room; repeats count once.
 * @param length  Long side of the room's minimum-area rectangle, m.
 * @param width   Short side, m.
 */
RoomTyping classifyRoom(
    const RoomTypeTable& table, const std::vector<std::string>& labels, double length, double width);

}  // namespace g1_world_model

#endif  // G1_WORLD_MODEL__ROOM_TYPING_HPP_
