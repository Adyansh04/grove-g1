/**
 * @file room_typing.cpp
 * @brief Naive Bayes room typing and its YAML table.
 */

#include "g1_world_model/room_typing.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <set>

namespace g1_world_model
{

namespace
{

/// Narrower than this and several times longer, with little in it: a hallway.
constexpr double      kHallwayWidth       = 2.2;
constexpr double      kHallwayAspect      = 3.0;
constexpr std::size_t kHallwayMostObjects = 2;

}  // namespace

RoomTypeTable RoomTypeTable::fromYaml(const std::string& path)
{
    const YAML::Node root = YAML::LoadFile(path);
    RoomTypeTable    table;
    table.unlisted = root["unlisted"].as<double>(table.unlisted);

    const YAML::Node types = root["room_types"];
    for (const auto& entry : types)
    {
        table.types.push_back(entry.first.as<std::string>());
        table.priors.push_back(entry.second["prior"].as<double>(1.0));
    }
    for (std::size_t slot = 0; slot < table.types.size(); ++slot)
    {
        const YAML::Node objects = types[table.types[slot]]["objects"];
        for (const auto& object : objects)
        {
            auto& row = table.likelihood[object.first.as<std::string>()];
            row.resize(table.types.size(), table.unlisted);
            row[slot] = object.second.as<double>();
        }
    }
    return table;
}

RoomTyping classifyRoom(
    const RoomTypeTable& table, const std::vector<std::string>& labels, double length, double width)
{
    const std::set<std::string> distinct(labels.begin(), labels.end());
    if (width > 0.0 && width < kHallwayWidth && length > kHallwayAspect * width &&
        distinct.size() <= kHallwayMostObjects)
    {
        return { "hallway", 0.8 };
    }
    if (table.types.empty())
    {
        return {};
    }

    std::vector<double> score(table.types.size(), 0.0);
    for (std::size_t slot = 0; slot < table.types.size(); ++slot)
    {
        score[slot] = std::log(std::max(table.priors[slot], 1e-6));
    }
    bool evidence = false;
    for (const std::string& label : distinct)
    {
        const auto row = table.likelihood.find(label);
        if (row == table.likelihood.end())
        {
            continue;  // A label the table knows nothing about says nothing about the room.
        }
        evidence = true;
        for (std::size_t slot = 0; slot < table.types.size(); ++slot)
        {
            score[slot] += std::log(std::max(row->second[slot], 1e-6));
        }
    }
    if (!evidence)
    {
        return {};
    }

    const double top   = *std::max_element(score.begin(), score.end());
    double       total = 0.0;
    for (double& value : score)
    {
        value = std::exp(value - top);
        total += value;
    }
    const auto best =
        static_cast<std::size_t>(std::max_element(score.begin(), score.end()) - score.begin());
    return { table.types[best], score[best] / total };
}

}  // namespace g1_world_model
