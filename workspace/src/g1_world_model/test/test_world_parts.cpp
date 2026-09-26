/**
 * @file test_world_parts.cpp
 * @brief Grid helpers, room typing, approach poses and persistence.
 */

#include <gmock/gmock.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <numbers>
#include <opencv2/imgproc.hpp>

#include "g1_world_model/approach_pose.hpp"
#include "g1_world_model/grid.hpp"
#include "g1_world_model/room_typing.hpp"
#include "g1_world_model/world_render.hpp"
#include "g1_world_model/world_store.hpp"

namespace g1_world_model
{
namespace
{

RoomTypeTable table()
{
    return RoomTypeTable::fromYaml(std::string(G1_WORLD_MODEL_CONFIG_DIR) + "/room_types.yaml");
}

/// Folded difference of two wall axes, rad: 89 deg and 1 deg are 2 deg apart.
double axisError(double a, double b)
{
    const double d = std::abs(std::remainder(a - b, std::numbers::pi / 2.0));
    return d;
}

/// A 6 x 4 m room's walls, 0.1 m thick, turned by @p yaw about the grid's middle.
cv::Mat turnedRoom(double yaw)
{
    cv::Mat           cells(200, 200, CV_8UC1, cv::Scalar(kFree));
    const cv::Point2f centre(100.0F, 100.0F);
    const auto        corners = [&](double half_x, double half_y) {
        std::vector<cv::Point> out;
        for (const auto& [sx, sy] : { std::pair{ -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } })
        {
            const double x = sx * half_x;
            const double y = sy * half_y;
            out.emplace_back(
                static_cast<int>(std::lround(centre.x + (x * std::cos(yaw)) - (y * std::sin(yaw)))),
                static_cast<int>(std::lround(centre.y + (x * std::sin(yaw)) + (y * std::cos(yaw)))));
        }
        return out;
    };
    cv::fillConvexPoly(cells, corners(62.0, 42.0), cv::Scalar(kOccupied));
    cv::fillConvexPoly(cells, corners(60.0, 40.0), cv::Scalar(kFree));
    return cells;
}

TEST(Grid, FindsTheWallsAxis)
{
    for (const double degrees : { 0.0, 12.0, 30.0, 45.0, 80.0 })
    {
        const double yaw = degrees * std::numbers::pi / 180.0;
        EXPECT_LT(axisError(dominantAxis(turnedRoom(yaw)), yaw), 2.0 * std::numbers::pi / 180.0)
            << degrees << " deg";
    }
}

TEST(Grid, CarriesALayerOntoAGrownAndShiftedGrid)
{
    const GridGeometry from{ 0.05, 0.0, 0.0, 4, 3 };
    std::vector<int>   layer(from.cellCount());
    for (std::size_t i = 0; i < layer.size(); ++i)
    {
        layer[i] = static_cast<int>(i) + 1;
    }
    // Two cells more on the left and one below: the old cell (0, 0) is now (2, 1).
    const GridGeometry     to{ 0.05, -0.10, -0.05, 7, 5 };
    const std::vector<int> out = remapLayer(layer, from, to, 0);
    EXPECT_EQ(out[static_cast<std::size_t>(to.index(2, 1))], layer[0]);
    EXPECT_EQ(
        out[static_cast<std::size_t>(to.index(5, 3))],
        layer[static_cast<std::size_t>(from.index(3, 2))]);
    EXPECT_EQ(out[static_cast<std::size_t>(to.index(0, 0))], 0);
    EXPECT_EQ(out[static_cast<std::size_t>(to.index(6, 4))], 0);
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

TEST(Grid, SettlesThePocketsTheMapEncloses)
{
    cv::Mat cells(40, 60, CV_8UC1, cv::Scalar(kUnknown));
    cv::rectangle(cells, cv::Point(5, 5), cv::Point(54, 34), cv::Scalar(kOccupied), cv::FILLED);
    cv::rectangle(cells, cv::Point(6, 6), cv::Point(53, 33), cv::Scalar(kFree), cv::FILLED);
    // A sofa whose inside no beam reached.
    cv::rectangle(cells, cv::Point(20, 15), cv::Point(30, 20), cv::Scalar(kOccupied), cv::FILLED);
    cv::rectangle(cells, cv::Point(21, 16), cv::Point(29, 19), cv::Scalar(kUnknown), cv::FILLED);
    // A partition with a stretch no beam hit, and a gap between beams out on the floor.
    cv::line(cells, cv::Point(40, 6), cv::Point(40, 33), cv::Scalar(kOccupied));
    cv::line(cells, cv::Point(40, 20), cv::Point(40, 22), cv::Scalar(kUnknown));
    cells.at<std::uint8_t>(10, 12) = kUnknown;

    const cv::Mat settled = settleEnclosedUnknown(cells);
    EXPECT_EQ(settled.at<std::uint8_t>(0, 0), kUnknown);     // The outside.
    EXPECT_EQ(settled.at<std::uint8_t>(17, 25), kOccupied);  // Inside the sofa.
    EXPECT_EQ(settled.at<std::uint8_t>(21, 40), kOccupied);  // The partition, closed.
    EXPECT_EQ(settled.at<std::uint8_t>(10, 12), kFree);      // The floor, filled.
}

TEST(Grid, ClosesTheWallBehindACounterButNotADoorway)
{
    // A room whose right wall the scan band only sees above a doorway's height, and a counter
    // standing on its top wall, its back and the wall behind it never seen.
    cv::Mat cells(40, 60, CV_8UC1, cv::Scalar(kUnknown));
    cv::rectangle(cells, cv::Point(5, 5), cv::Point(54, 34), cv::Scalar(kOccupied), cv::FILLED);
    cv::rectangle(cells, cv::Point(6, 6), cv::Point(53, 33), cv::Scalar(kFree), cv::FILLED);
    cv::rectangle(cells, cv::Point(20, 3), cv::Point(40, 8), cv::Scalar(kUnknown), cv::FILLED);
    cv::line(cells, cv::Point(20, 9), cv::Point(40, 9), cv::Scalar(kOccupied));
    cv::rectangle(cells, cv::Point(54, 15), cv::Point(59, 20), cv::Scalar(kFree), cv::FILLED);
    // Wall-height hits: the whole outline, the lintel over the doorway included.
    cv::Mat walls(cells.size(), CV_8UC1, cv::Scalar(0));
    cv::rectangle(walls, cv::Point(5, 5), cv::Point(54, 34), cv::Scalar(255));

    // A wardrobe on the bottom wall hides it at every height: no hits there either.
    cv::rectangle(cells, cv::Point(20, 30), cv::Point(35, 37), cv::Scalar(kUnknown), cv::FILLED);
    cv::line(cells, cv::Point(20, 30), cv::Point(35, 30), cv::Scalar(kOccupied));
    cv::line(walls, cv::Point(20, 34), cv::Point(35, 34), cv::Scalar(0));

    const cv::Mat plan = completeMap(cells, walls, 0.0, { 20, 12, 3 });
    EXPECT_EQ(plan.at<std::uint8_t>(5, 30), kOccupied);   // The wall behind the counter.
    EXPECT_EQ(plan.at<std::uint8_t>(7, 30), kOccupied);   // The counter, solid to the wall.
    EXPECT_EQ(plan.at<std::uint8_t>(34, 27), kOccupied);  // The wall behind the wardrobe.
    EXPECT_EQ(plan.at<std::uint8_t>(32, 27), kOccupied);  // The wardrobe, solid.
    EXPECT_EQ(plan.at<std::uint8_t>(17, 54), kFree);      // The doorway, lintel or not.
    EXPECT_EQ(plan.at<std::uint8_t>(3, 30), kUnknown);    // Outside, open to the edge.
    EXPECT_EQ(plan.at<std::uint8_t>(37, 27), kUnknown);   // And below the room.
}

TEST(Grid, ClosesFurnitureAtTheMapsEdgeWhateverTheWallsYaw)
{
    // A fridge in a room's corner, its back at the grid's edge: the wall behind it is off the map,
    // so only the run between the wall and the fridge's side closes it.
    cv::Mat cells(40, 60, CV_8UC1, cv::Scalar(kFree));
    cv::line(cells, cv::Point(1, 0), cv::Point(1, 39), cv::Scalar(kOccupied));
    cv::rectangle(cells, cv::Point(2, 0), cv::Point(9, 7), cv::Scalar(kUnknown), cv::FILLED);
    cv::line(cells, cv::Point(2, 8), cv::Point(10, 8), cv::Scalar(kOccupied));
    cv::line(cells, cv::Point(10, 0), cv::Point(10, 8), cv::Scalar(kOccupied));
    const cv::Mat walls(cells.size(), CV_8UC1, cv::Scalar(0));
    for (const double yaw : { 0.0, 1.45 })
    {
        EXPECT_EQ(completeMap(cells, walls, yaw, { 20, 12, 3 }).at<std::uint8_t>(3, 5), kOccupied)
            << yaw;
    }
}

TEST(Grid, KeepsTheFloorBehindAWardrobeFreeButFillsWhatTheCameraMapped)
{
    // A room with a wardrobe in its top right corner: the scan saw its west and south faces, and
    // past its corner some of the floor between it and the top wall; the rest of that floor never.
    cv::Mat cells(40, 60, CV_8UC1, cv::Scalar(kUnknown));
    cv::rectangle(cells, cv::Point(5, 5), cv::Point(54, 34), cv::Scalar(kOccupied), cv::FILLED);
    cv::rectangle(cells, cv::Point(6, 6), cv::Point(53, 33), cv::Scalar(kFree), cv::FILLED);
    cv::rectangle(cells, cv::Point(45, 6), cv::Point(53, 21), cv::Scalar(kUnknown), cv::FILLED);
    cv::line(cells, cv::Point(44, 12), cv::Point(44, 22), cv::Scalar(kOccupied));
    cv::line(cells, cv::Point(44, 22), cv::Point(53, 22), cv::Scalar(kOccupied));
    // An armchair out on the floor, seen from the north and east only: floor all round the rest.
    cv::rectangle(cells, cv::Point(20, 20), cv::Point(26, 25), cv::Scalar(kUnknown), cv::FILLED);
    cv::line(cells, cv::Point(20, 19), cv::Point(27, 19), cv::Scalar(kOccupied));
    cv::line(cells, cv::Point(27, 19), cv::Point(27, 25), cv::Scalar(kOccupied));
    cv::Mat walls(cells.size(), CV_8UC1, cv::Scalar(0));
    cv::rectangle(walls, cv::Point(5, 5), cv::Point(54, 34), cv::Scalar(255));
    // The camera mapped the armchair, and the wardrobe from its faces.
    cv::Mat mapped(cells.size(), CV_8UC1, cv::Scalar(0));
    cv::rectangle(mapped, cv::Point(19, 18), cv::Point(28, 26), cv::Scalar(255), cv::FILLED);
    cv::rectangle(mapped, cv::Point(43, 11), cv::Point(54, 23), cv::Scalar(255), cv::FILLED);

    const cv::Mat plan = completeMap(cells, walls, 0.0, { 20, 12, 3 }, mapped);
    EXPECT_EQ(plan.at<std::uint8_t>(17, 49), kOccupied);  // The wardrobe, solid to the wall.
    EXPECT_EQ(plan.at<std::uint8_t>(8, 49), kFree);       // The floor behind it: floor.
    EXPECT_EQ(plan.at<std::uint8_t>(22, 23), kOccupied);  // The armchair's underside.
    EXPECT_EQ(completeMap(cells, walls, 0.0, { 20, 12, 3 }).at<std::uint8_t>(22, 23), kFree);
}

TEST(WorldStore, SavesTheMapAsMapServerReadsIt)
{
    // 3 x 2: free, occupied, unknown on the bottom row; unknown, free, occupied on the top.
    const GridGeometry             geometry{ 0.05, -1.0, 2.0, 3, 2 };
    const std::vector<std::int8_t> data{ 0, 100, -1, -1, 0, 100 };
    const std::string directory = (std::filesystem::temp_directory_path() / "g1_map_save").string();
    ASSERT_EQ(saveOccupancy(directory, data, geometry), "");

    std::ifstream pgm(directory + "/map.pgm", std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(pgm)), std::istreambuf_iterator<char>());
    const std::string header = "P5\n3 2\n255\n";
    ASSERT_EQ(bytes.size(), header.size() + 6);
    EXPECT_EQ(bytes.substr(0, header.size()), header);
    // The file's first row is the map's top one.
    EXPECT_EQ(static_cast<unsigned char>(bytes[header.size() + 0]), 205);
    EXPECT_EQ(static_cast<unsigned char>(bytes[header.size() + 1]), 254);
    EXPECT_EQ(static_cast<unsigned char>(bytes[header.size() + 2]), 0);
    EXPECT_EQ(static_cast<unsigned char>(bytes[header.size() + 3]), 254);
    EXPECT_EQ(static_cast<unsigned char>(bytes[header.size() + 4]), 0);
    EXPECT_EQ(static_cast<unsigned char>(bytes[header.size() + 5]), 205);

    std::ifstream     yaml_file(directory + "/map.yaml");
    const std::string yaml(
        (std::istreambuf_iterator<char>(yaml_file)),
        std::istreambuf_iterator<char>());
    EXPECT_THAT(yaml, ::testing::HasSubstr("origin: [-1, 2, 0]"));
    EXPECT_THAT(yaml, ::testing::HasSubstr("free_thresh: 0.196"));
    std::filesystem::remove_all(directory);
}

TEST(WorldRender, KeepsLabelsApart)
{
    // A mug and a bowl on a table: all three labels would sit on the table's middle.
    const std::vector<cv::Rect>  boxes{ { 40, 40, 80, 40 }, { 70, 55, 8, 8 }, { 84, 55, 8, 8 } };
    const std::vector<cv::Size>  sizes{ { 50, 10 }, { 24, 10 }, { 26, 10 } };
    const std::vector<cv::Point> corners = placeLabels(boxes, sizes, {});
    ASSERT_EQ(corners.size(), 3U);
    EXPECT_EQ(corners[0], cv::Point(55, 55));  // The table keeps its middle.
    for (std::size_t i = 0; i < corners.size(); ++i)
    {
        for (std::size_t j = i + 1; j < corners.size(); ++j)
        {
            EXPECT_EQ((cv::Rect(corners[i], sizes[i]) & cv::Rect(corners[j], sizes[j])).area(), 0)
                << i << " and " << j;
        }
    }
}

TEST(WorldRender, TintsRoomsAndOutlinesObjects)
{
    const GridGeometry geometry{ 0.05, 0.0, 0.0, 40, 20 };
    cv::Mat            cells(geometry.height, geometry.width, CV_8UC1, cv::Scalar(kFree));
    cv::Mat            labels(geometry.height, geometry.width, CV_32S, cv::Scalar(1));
    labels(cv::Rect(20, 0, 20, 20)) = 2;
    const cv::Mat image             = renderWorld(
        cells,
        geometry,
        labels,
        { { 1, "R1 office", 0.5, 0.5 }, { 2, "R2 bedroom", 1.5, 0.5 } },
        { { { { 1.0, 0.5 }, { 0.6, 0.4 }, 0.0 }, "desk" } },
        4);
    ASSERT_EQ(image.cols, 160);
    ASSERT_EQ(image.rows, 80);
    // Two rooms, two tints; neither is the free-space white.
    const auto& left  = image.at<cv::Vec3b>(75, 5);
    const auto& right = image.at<cv::Vec3b>(75, 155);
    EXPECT_NE(left, right);
    EXPECT_NE(left, cv::Vec3b(255, 255, 255));
    // The desk's outline crosses its box's left edge, 0.7 m in: pixel column 56.
    bool outlined = false;
    for (int x = 54; x <= 58; ++x)
    {
        const auto& pixel = image.at<cv::Vec3b>(40, x);
        outlined          = outlined || (pixel[2] > pixel[0] + 60);
    }
    EXPECT_TRUE(outlined);
}

}  // namespace
}  // namespace g1_world_model
