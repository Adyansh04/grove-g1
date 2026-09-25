/**
 * @file world_store.cpp
 * @brief YAML and binary persistence of rooms, objects and coverage.
 */

#include "g1_world_model/world_store.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace g1_world_model
{

namespace
{

constexpr std::uint32_t kMagic   = 0x4D573147U;  // "G1WM"
constexpr std::uint32_t kVersion = 1;

const char* stateName(ObjectState state)
{
    switch (state)
    {
        case ObjectState::kStale:
            return "stale";
        case ObjectState::kRemoved:
            return "removed";
        default:
            return "active";
    }
}

ObjectState stateOf(const std::string& name)
{
    if (name == "stale")
    {
        return ObjectState::kStale;
    }
    return name == "removed" ? ObjectState::kRemoved : ObjectState::kActive;
}

template <typename T>
void put(std::ostream& out, const T& value)
{
    out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
bool get(std::istream& in, T& value)
{
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    return static_cast<bool>(in);
}

/// Writes through a temporary file and renames it over @p path, so a crash leaves the old file.
std::string writeAtomically(
    const std::filesystem::path& path, const std::string& contents, std::ios::openmode mode)
{
    const std::filesystem::path temporary = path.string() + ".tmp";
    {
        std::ofstream out(temporary, mode | std::ios::trunc);
        if (!out)
        {
            return "cannot write " + temporary.string();
        }
        out << contents;
        if (!out)
        {
            return "failed writing " + temporary.string();
        }
    }
    std::error_code error;
    std::filesystem::rename(temporary, path, error);
    return error ? "cannot replace " + path.string() + ": " + error.message() : std::string{};
}

}  // namespace

std::string saveWorld(const std::string& directory, const WorldSnapshot& snapshot)
{
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error)
    {
        return "cannot create " + directory + ": " + error.message();
    }
    const std::filesystem::path root(directory);

    YAML::Emitter yaml;
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "version" << YAML::Value << kVersion;
    yaml << YAML::Key << "grid" << YAML::Value << YAML::Flow << YAML::BeginMap;
    yaml << YAML::Key << "resolution" << YAML::Value << snapshot.geometry.resolution;
    yaml << YAML::Key << "origin_x" << YAML::Value << snapshot.geometry.origin_x;
    yaml << YAML::Key << "origin_y" << YAML::Value << snapshot.geometry.origin_y;
    yaml << YAML::Key << "width" << YAML::Value << snapshot.geometry.width;
    yaml << YAML::Key << "height" << YAML::Value << snapshot.geometry.height;
    yaml << YAML::EndMap;
    yaml << YAML::Key << "next_room" << YAML::Value << snapshot.next_room;
    yaml << YAML::Key << "rooms" << YAML::Value << YAML::BeginSeq;
    for (const RoomRecord& room : snapshot.rooms)
    {
        yaml << YAML::Flow << YAML::BeginMap;
        yaml << YAML::Key << "id" << YAML::Value << room.id;
        yaml << YAML::Key << "name" << YAML::Value << room.name;
        yaml << YAML::Key << "type" << YAML::Value << room.type;
        yaml << YAML::Key << "type_confidence" << YAML::Value << room.type_confidence;
        yaml << YAML::Key << "type_source" << YAML::Value << room.type_source;
        yaml << YAML::Key << "x" << YAML::Value << room.x;
        yaml << YAML::Key << "y" << YAML::Value << room.y;
        yaml << YAML::EndMap;
    }
    yaml << YAML::EndSeq;
    yaml << YAML::Key << "objects" << YAML::Value << YAML::BeginSeq;
    for (const MappedObject& object : snapshot.objects)
    {
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "id" << YAML::Value << object.id;
        yaml << YAML::Key << "label" << YAML::Value << object.label();
        yaml << YAML::Key << "votes" << YAML::Value << YAML::Flow << YAML::BeginMap;
        for (const auto& [label, weight] : object.votes)
        {
            yaml << YAML::Key << label << YAML::Value << weight;
        }
        yaml << YAML::EndMap;
        yaml << YAML::Key << "name" << YAML::Value << object.name;
        yaml << YAML::Key << "caption" << YAML::Value << object.caption;
        yaml << YAML::Key << "centre" << YAML::Value << YAML::Flow << YAML::BeginSeq
             << object.box_centre.x() << object.box_centre.y() << YAML::EndSeq;
        yaml << YAML::Key << "size" << YAML::Value << YAML::Flow << YAML::BeginSeq
             << object.box_size.x() << object.box_size.y() << object.height() << YAML::EndSeq;
        yaml << YAML::Key << "yaw" << YAML::Value << object.box_yaw;
        yaml << YAML::Key << "observations" << YAML::Value << object.observations;
        yaml << YAML::Key << "first_seen" << YAML::Value << object.first_seen;
        yaml << YAML::Key << "last_seen" << YAML::Value << object.last_seen;
        yaml << YAML::Key << "misses" << YAML::Value << object.misses;
        yaml << YAML::Key << "state" << YAML::Value << stateName(object.state);
        yaml << YAML::Key << "best_view" << YAML::Value << YAML::Flow << YAML::BeginSeq
             << object.best_view.stamp << object.best_view.x << object.best_view.y
             << object.best_view.width << object.best_view.height << object.best_view.score
             << YAML::EndSeq;
        yaml << YAML::EndMap;
    }
    yaml << YAML::EndSeq;
    yaml << YAML::EndMap;
    std::string failure =
        writeAtomically(root / "world.yaml", std::string(yaml.c_str()) + "\n", std::ios::out);
    if (!failure.empty())
    {
        return failure;
    }

    std::ostringstream objects(std::ios::binary);
    put(objects, kMagic);
    put(objects, kVersion);
    put(objects, static_cast<std::uint32_t>(snapshot.objects.size()));
    for (const MappedObject& object : snapshot.objects)
    {
        put(objects, static_cast<std::int32_t>(object.id));
        put(objects, static_cast<std::uint32_t>(object.voxels.size()));
        objects.write(
            reinterpret_cast<const char*>(object.voxels.data()),
            static_cast<std::streamsize>(object.voxels.size() * sizeof(std::uint64_t)));
        put(objects, static_cast<std::uint32_t>(object.embedding.size()));
        put(objects, static_cast<std::int32_t>(object.embedding_count));
        objects.write(
            reinterpret_cast<const char*>(object.embedding.data()),
            static_cast<std::streamsize>(object.embedding.size() * sizeof(float)));
    }
    failure =
        writeAtomically(root / "objects.bin", objects.str(), std::ios::out | std::ios::binary);
    if (!failure.empty())
    {
        return failure;
    }

    std::ostringstream coverage(std::ios::binary);
    put(coverage, kMagic);
    put(coverage, kVersion);
    put(coverage, static_cast<std::uint32_t>(snapshot.cells.size()));
    for (const auto* layer : { &snapshot.cells,
                               &snapshot.quality,
                               &snapshot.surface_quality,
                               &snapshot.flags,
                               &snapshot.directions })
    {
        if (layer->size() != snapshot.cells.size())
        {
            return "coverage layers differ in size";
        }
        coverage.write(
            reinterpret_cast<const char*>(layer->data()),
            static_cast<std::streamsize>(layer->size()));
    }
    return writeAtomically(root / "coverage.bin", coverage.str(), std::ios::out | std::ios::binary);
}

bool worldFits(
    const WorldSnapshot& snapshot, const cv::Mat& cells, const GridGeometry& geometry,
    double min_agreement)
{
    const GridGeometry& saved = snapshot.geometry;
    if (saved.width != geometry.width || saved.height != geometry.height ||
        std::abs(saved.resolution - geometry.resolution) > 1e-6 ||
        std::abs(saved.origin_x - geometry.origin_x) > 0.01 ||
        std::abs(saved.origin_y - geometry.origin_y) > 0.01 ||
        snapshot.cells.size() != geometry.cellCount() || cells.empty())
    {
        return false;
    }
    std::size_t agree = 0;
    const auto* now   = cells.ptr<std::uint8_t>(0);
    for (std::size_t index = 0; index < snapshot.cells.size(); ++index)
    {
        agree += static_cast<std::size_t>(snapshot.cells[index] == now[index]);
    }
    return static_cast<double>(agree) >= min_agreement * static_cast<double>(snapshot.cells.size());
}

std::optional<WorldSnapshot> loadWorld(const std::string& directory, std::string& error)
{
    const std::filesystem::path root(directory);
    WorldSnapshot               snapshot;
    std::map<int, std::size_t>  slot_of;
    try
    {
        const YAML::Node yaml = YAML::LoadFile((root / "world.yaml").string());
        if (yaml["version"].as<std::uint32_t>(0) != kVersion)
        {
            error = "world.yaml has an unknown version";
            return std::nullopt;
        }
        const YAML::Node grid = yaml["grid"];
        snapshot.geometry     = { grid["resolution"].as<double>(),
                                  grid["origin_x"].as<double>(),
                                  grid["origin_y"].as<double>(),
                                  grid["width"].as<int>(),
                                  grid["height"].as<int>() };
        snapshot.next_room    = yaml["next_room"].as<int>(1);
        for (const YAML::Node& room : yaml["rooms"])
        {
            snapshot.rooms.push_back({ room["id"].as<std::string>(),
                                       room["name"].as<std::string>(""),
                                       room["type"].as<std::string>(""),
                                       room["type_confidence"].as<double>(0.0),
                                       room["type_source"].as<std::string>(""),
                                       room["x"].as<double>(0.0),
                                       room["y"].as<double>(0.0) });
        }
        for (const YAML::Node& node : yaml["objects"])
        {
            MappedObject object;
            object.id = node["id"].as<int>();
            for (const auto& vote : node["votes"])
            {
                object.votes[vote.first.as<std::string>()] = vote.second.as<float>();
            }
            object.name         = node["name"].as<std::string>("");
            object.caption      = node["caption"].as<std::string>("");
            object.observations = node["observations"].as<int>(0);
            object.first_seen   = node["first_seen"].as<double>(0.0);
            object.last_seen    = node["last_seen"].as<double>(0.0);
            object.misses       = node["misses"].as<int>(0);
            object.state        = stateOf(node["state"].as<std::string>("active"));
            if (const YAML::Node view = node["best_view"]; view && view.size() == 6)
            {
                object.best_view = { view[0].as<double>(), view[1].as<int>(), view[2].as<int>(),
                                     view[3].as<int>(),    view[4].as<int>(), view[5].as<double>() };
            }
            slot_of[object.id] = snapshot.objects.size();
            snapshot.objects.push_back(std::move(object));
        }
    }
    catch (const YAML::Exception& exception)
    {
        error = std::string("world.yaml: ") + exception.what();
        return std::nullopt;
    }

    std::ifstream objects(root / "objects.bin", std::ios::binary);
    std::uint32_t magic   = 0;
    std::uint32_t version = 0;
    std::uint32_t count   = 0;
    if (!get(objects, magic) || !get(objects, version) || !get(objects, count) || magic != kMagic ||
        version != kVersion)
    {
        error = "objects.bin is missing or not a world file";
        return std::nullopt;
    }
    for (std::uint32_t i = 0; i < count; ++i)
    {
        std::int32_t  id         = 0;
        std::uint32_t voxels     = 0;
        std::uint32_t dimensions = 0;
        std::int32_t  embedded   = 0;
        if (!get(objects, id) || !get(objects, voxels))
        {
            error = "objects.bin is truncated";
            return std::nullopt;
        }
        std::vector<std::uint64_t> keys(voxels);
        objects.read(
            reinterpret_cast<char*>(keys.data()),
            static_cast<std::streamsize>(voxels * sizeof(std::uint64_t)));
        if (!get(objects, dimensions) || !get(objects, embedded))
        {
            error = "objects.bin is truncated";
            return std::nullopt;
        }
        std::vector<float> embedding(dimensions);
        objects.read(
            reinterpret_cast<char*>(embedding.data()),
            static_cast<std::streamsize>(dimensions * sizeof(float)));
        if (!objects)
        {
            error = "objects.bin is truncated";
            return std::nullopt;
        }
        const auto slot = slot_of.find(id);
        if (slot == slot_of.end())
        {
            continue;
        }
        MappedObject& object   = snapshot.objects[slot->second];
        object.voxels          = std::move(keys);
        object.embedding       = std::move(embedding);
        object.embedding_count = embedded;
    }

    std::ifstream coverage(root / "coverage.bin", std::ios::binary);
    std::uint32_t cells = 0;
    if (!get(coverage, magic) || !get(coverage, version) || !get(coverage, cells) ||
        magic != kMagic || version != kVersion)
    {
        error = "coverage.bin is missing or not a world file";
        return std::nullopt;
    }
    for (auto* layer : { &snapshot.cells,
                         &snapshot.quality,
                         &snapshot.surface_quality,
                         &snapshot.flags,
                         &snapshot.directions })
    {
        layer->resize(cells);
        coverage.read(reinterpret_cast<char*>(layer->data()), static_cast<std::streamsize>(cells));
    }
    if (!coverage)
    {
        error = "coverage.bin is truncated";
        return std::nullopt;
    }
    return snapshot;
}

}  // namespace g1_world_model
