/**
 * @file test_world_parts.cpp
 * @brief Room typing, approach poses and persistence.
 */

#include <gmock/gmock.h>

#include <filesystem>
#include <opencv2/imgproc.hpp>

#include "g1_world_model/approach_pose.hpp"
#include "g1_world_model/room_typing.hpp"
#include "g1_world_model/world_store.hpp"

namespace g1_world_model
{
namespace
{

RoomTypeTable table()
{
    return RoomTypeTable::fromYaml(std::string(G1_WORLD_MODEL_CONFIG_DIR) + "/room_types.yaml");
}

TEST(RoomTyping, NamesDistinctiveRoomsFromTheirObjects)
{
    const RoomTypeTable types = table();
    EXPECT_EQ(classifyRoom(types, { "refrigerator", "counter", "sink" }, 5.0, 4.0).type, "kitchen");
    EXPECT_EQ(classifyRoom(types, { "bed", "nightstand", "lamp" }, 4.0, 4.0).type, "bedroom");
    EXPECT_EQ(
        classifyRoom(types, { "sofa", "coffee table", "television" }, 6.0, 5.0).type,
        "living room");
    EXPECT_EQ(classifyRoom(types, { "desk", "monitor", "office chair" }, 5.0, 4.0).type, "office");
    const RoomTyping storage = classifyRoom(types, { "cardboard box", "crate", "shelf" }, 4.0, 3.0);
    EXPECT_EQ(storage.type, "storage room");
    EXPECT_GT(storage.probability, 0.5);
}

TEST(RoomTyping, ALongEmptyStripIsAHallway)
{
    EXPECT_EQ(classifyRoom(table(), { "potted plant" }, 12.0, 1.8).type, "hallway");
    EXPECT_EQ(classifyRoom(table(), {}, 4.0, 4.0).type, "");
}

TEST(ApproachPose, StandsClearOfATableAndFacesIt)
{
    // A 6 x 6 m room with a 1.2 x 0.8 m table in the middle.
    const GridGeometry geometry{ 0.05, 0.0, 0.0, 120, 120 };
    cv::Mat            cells(120, 120, CV_8UC1, cv::Scalar(kOccupied));
    cv::rectangle(cells, cv::Point(2, 2), cv::Point(117, 117), cv::Scalar(kFree), cv::FILLED);
    cv::rectangle(cells, cv::Point(48, 52), cv::Point(71, 67), cv::Scalar(kOccupied), cv::FILLED);

    ViewpointPlanner planner;
    planner.prepare(cells, geometry, { 1.0, 1.0, 0.0 });
    const Footprint table{ { 3.0, 3.0 }, { 1.2, 0.8 }, 0.0 };
    const auto pose = approachPose(planner.clearance(), planner.travel(), geometry, table, 0.0, {});
    ASSERT_TRUE(pose.has_value());
    const double gap = std::hypot(
        std::max(std::abs(pose->x - 3.0) - 0.6, 0.0),
        std::max(std::abs(pose->y - 3.0) - 0.4, 0.0));
    EXPECT_GE(gap, 0.55);
    EXPECT_LE(gap, 0.9);
    // It faces the table.
    EXPECT_NEAR(
        std::remainder(pose->yaw - std::atan2(3.0 - pose->y, 3.0 - pose->x), 2.0 * M_PI),
        0.0,
        1e-6);
    // And came from the side the robot starts on.
    EXPECT_LT(pose->x + pose->y, 6.0);
}

TEST(WorldStore, RoundTripsRoomsObjectsAndCoverage)
{
    WorldSnapshot snapshot;
    snapshot.geometry  = { 0.05, -2.0, 1.5, 3, 1 };
    snapshot.cells     = { kFree, kOccupied, kUnknown };
    snapshot.next_room = 4;
    snapshot.rooms.push_back({ "R2", "room B", "office", 0.7, "objects", 3.5, -1.25 });
    MappedObject object;
    object.id              = 12;
    object.votes           = { { "dustbin", 2.5F }, { "bucket", 0.4F } };
    object.name            = "metal bin";
    object.caption         = "A grey metal dustbin in the corner.";
    object.voxels          = { 5, 9, 1024 };
    object.embedding       = { 0.6F, 0.8F };
    object.embedding_count = 3;
    object.observations    = 7;
    object.state           = ObjectState::kStale;
    object.best_view       = { 12.5, 10, 20, 30, 40, 900.0 };
    snapshot.objects.push_back(object);
    snapshot.quality         = { 0, 128, 255 };
    snapshot.surface_quality = { 1, 2, 3 };
    snapshot.flags           = { 0, 1, 0 };
    snapshot.directions      = { 0, 0, 7 };

    const std::string directory =
        (std::filesystem::temp_directory_path() / "g1_world_store_test").string();
    std::filesystem::remove_all(directory);
    ASSERT_EQ(saveWorld(directory, snapshot), "");

    std::string error;
    const auto  loaded = loadWorld(directory, error);
    ASSERT_TRUE(loaded.has_value()) << error;
    EXPECT_EQ(loaded->geometry, snapshot.geometry);
    EXPECT_EQ(loaded->cells, snapshot.cells);
    const cv::Mat same  = (cv::Mat_<std::uint8_t>(1, 3) << kFree, kOccupied, kUnknown);
    const cv::Mat other = (cv::Mat_<std::uint8_t>(1, 3) << kOccupied, kOccupied, kFree);
    EXPECT_TRUE(worldFits(*loaded, same, snapshot.geometry));
    EXPECT_FALSE(worldFits(*loaded, other, snapshot.geometry));
    EXPECT_EQ(loaded->next_room, 4);
    ASSERT_EQ(loaded->rooms.size(), 1U);
    EXPECT_EQ(loaded->rooms[0].name, "room B");
    EXPECT_DOUBLE_EQ(loaded->rooms[0].y, -1.25);
    ASSERT_EQ(loaded->objects.size(), 1U);
    const MappedObject& back = loaded->objects[0];
    EXPECT_EQ(back.id, 12);
    EXPECT_EQ(back.label(), "dustbin");
    EXPECT_EQ(back.caption, object.caption);
    EXPECT_EQ(back.voxels, object.voxels);
    EXPECT_EQ(back.embedding, object.embedding);
    EXPECT_EQ(back.embedding_count, 3);
    EXPECT_EQ(back.state, ObjectState::kStale);
    EXPECT_EQ(back.best_view.width, 30);
    EXPECT_EQ(loaded->directions, snapshot.directions);
    std::filesystem::remove_all(directory);
}

TEST(WorldStore, ReportsAMissingWorld)
{
    std::string error;
    EXPECT_FALSE(loadWorld("/nonexistent/g1_world", error).has_value());
    EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace g1_world_model
