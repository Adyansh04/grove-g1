/**
 * @file test_room_segmentation.cpp
 * @brief Room segmentation on hand-drawn floor plans.
 */

#include <gmock/gmock.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <set>

#include "g1_world_model/room_segmentation.hpp"

namespace g1_world_model
{
namespace
{

constexpr double kResolution = 0.05;

GridGeometry geometryOf(const cv::Mat& cells)
{
    return { kResolution, 0.0, 0.0, cells.cols, cells.rows };
}

int cellsOf(double metres) { return static_cast<int>(std::lround(metres / kResolution)); }

/// Fills a map-frame rectangle, in metres.
void fill(cv::Mat& cells, double x0, double y0, double x1, double y1, Cell value)
{
    cv::rectangle(
        cells,
        cv::Point(cellsOf(x0), cellsOf(y0)),
        cv::Point(cellsOf(x1) - 1, cellsOf(y1) - 1),
        cv::Scalar(value),
        cv::FILLED);
}

/// Two 5 x 4 m rooms side by side, joined by a door of @p door_width in a 0.1 m wall.
cv::Mat twoRooms(double door_width)
{
    cv::Mat cells(cellsOf(4.2), cellsOf(10.3), CV_8UC1, cv::Scalar(kOccupied));
    fill(cells, 0.1, 0.1, 5.1, 4.1, kFree);
    fill(cells, 5.2, 0.1, 10.2, 4.1, kFree);
    fill(cells, 5.1, 2.0 - (door_width / 2.0), 5.2, 2.0 + (door_width / 2.0), kFree);
    return cells;
}

TEST(RoomSegmentation, SplitsTwoRoomsAtTheirDoor)
{
    const cv::Mat      cells  = twoRooms(1.0);
    const Segmentation result = segmentRooms(cells, geometryOf(cells), {});

    ASSERT_EQ(result.regions.size(), 2U);
    for (const Region& region : result.regions)
    {
        EXPECT_NEAR(region.area, 20.0, 1.5);
        ASSERT_EQ(region.contacts.size(), 1U);
        EXPECT_NEAR(region.contacts[0].x, 5.15, 0.2);
        EXPECT_NEAR(region.contacts[0].y, 2.0, 0.2);
        EXPECT_NEAR(region.contacts[0].width, 1.0, 0.2);
    }
    // The door is the boundary: each room keeps its own side.
    const Region& left = result.regions[0].centroid_x < 5.0 ? result.regions[0] : result.regions[1];
    EXPECT_NEAR(left.centroid_x, 2.6, 0.2);
}

TEST(RoomSegmentation, KeepsACorridorBetweenRooms)
{
    // Two 5 x 5 m rooms above a 10 x 1.6 m corridor, each opening onto it through a 1.2 m door.
    cv::Mat cells(cellsOf(7.0), cellsOf(10.2), CV_8UC1, cv::Scalar(kOccupied));
    fill(cells, 0.1, 0.1, 10.1, 1.7, kFree);
    fill(cells, 0.1, 1.8, 5.0, 6.9, kFree);
    fill(cells, 5.1, 1.8, 10.1, 6.9, kFree);
    fill(cells, 1.9, 1.7, 3.1, 1.8, kFree);
    fill(cells, 6.9, 1.7, 8.1, 1.8, kFree);

    const Segmentation result = segmentRooms(cells, geometryOf(cells), {});

    ASSERT_EQ(result.regions.size(), 3U);
    const auto corridor =
        std::find_if(result.regions.begin(), result.regions.end(), [](const Region& region) {
            return region.centroid_y < 1.5;
        });
    ASSERT_NE(corridor, result.regions.end());
    EXPECT_GT(corridor->length / corridor->width, 4.0);
    EXPECT_EQ(corridor->contacts.size(), 2U);
}

TEST(RoomSegmentation, SplitsACorridorThatOpensAtItsFullWidth)
{
    // An 8 x 8 m room with a 12 m hallway leaving it at the hallway's own width: no pinch at all.
    cv::Mat cells(cellsOf(8.2), cellsOf(20.2), CV_8UC1, cv::Scalar(kOccupied));
    fill(cells, 0.1, 0.1, 8.1, 8.1, kFree);
    fill(cells, 8.1, 3.1, 20.1, 4.9, kFree);

    const Segmentation result = segmentRooms(cells, geometryOf(cells), {});
    ASSERT_EQ(result.regions.size(), 2U);
    const auto hallway =
        std::find_if(result.regions.begin(), result.regions.end(), [](const Region& region) {
            return region.centroid_x > 10.0;
        });
    ASSERT_NE(hallway, result.regions.end());
    EXPECT_GT(hallway->area, 18.0);
    ASSERT_EQ(hallway->contacts.size(), 1U);
    EXPECT_NEAR(hallway->contacts[0].x, 8.1, 1.3);
}

TEST(RoomSegmentation, FurnitureDoesNotSplitARoom)
{
    cv::Mat cells(cellsOf(6.2), cellsOf(8.2), CV_8UC1, cv::Scalar(kOccupied));
    fill(cells, 0.1, 0.1, 8.1, 6.1, kFree);
    // A table, a sofa against a wall and a shelf: all in the scan band, so all occupied.
    fill(cells, 3.0, 2.5, 4.6, 3.4, kOccupied);
    fill(cells, 0.1, 4.8, 2.3, 5.7, kOccupied);
    fill(cells, 7.6, 0.5, 8.1, 2.5, kOccupied);

    const Segmentation result = segmentRooms(cells, geometryOf(cells), {});
    EXPECT_EQ(result.regions.size(), 1U);
}

TEST(RoomSegmentation, UnknownSpaceIsNotARoom)
{
    cv::Mat cells(cellsOf(5.0), cellsOf(5.0), CV_8UC1, cv::Scalar(kUnknown));
    fill(cells, 0.5, 0.5, 4.5, 4.5, kFree);
    const Segmentation result = segmentRooms(cells, geometryOf(cells), {});
    ASSERT_EQ(result.regions.size(), 1U);
    EXPECT_NEAR(result.regions[0].area, 16.0, 0.5);
    EXPECT_EQ(result.labels.at<int>(0, 0), 0);
}

TEST(RoomSegmentation, MatchesRoomsAcrossResegmentation)
{
    const cv::Mat      before = twoRooms(1.0);
    const Segmentation first  = segmentRooms(before, geometryOf(before), {});

    // A chair appears in the right room: same rooms, possibly relabelled.
    cv::Mat after = before.clone();
    fill(after, 8.0, 1.0, 8.6, 1.6, kOccupied);
    const Segmentation second = segmentRooms(after, geometryOf(after), {});

    const std::vector<int> match = matchRegions(
        first.labels,
        static_cast<int>(first.regions.size()),
        second.labels,
        static_cast<int>(second.regions.size()),
        0.2);
    ASSERT_EQ(match.size(), 2U);
    for (std::size_t slot = 0; slot < match.size(); ++slot)
    {
        ASSERT_GT(match[slot], 0);
        const Region& now  = second.regions[slot];
        const Region& then = first.regions[static_cast<std::size_t>(match[slot] - 1)];
        EXPECT_NEAR(now.centroid_x, then.centroid_x, 0.3);
    }
}

/// A map_server trinary PGM as Cell values, rows flipped so row y grows with map y.
cv::Mat loadMap(const std::string& name)
{
    const cv::Mat image = cv::imread(std::string(G1_MAPS_DIR) + "/" + name, cv::IMREAD_GRAYSCALE);
    cv::Mat       cells(image.size(), CV_8UC1, cv::Scalar(kUnknown));
    cells.setTo(kFree, image >= 250);
    cells.setTo(kOccupied, image <= 50);
    cv::flip(cells, cells, 0);
    return cells;
}

TEST(RoomSegmentation, SplitsTheFacilityIntoItsRoomsAndHub)
{
    // Four rooms around a hub, each open onto it through a 2.6 m gap rather than a door.
    const cv::Mat cells = loadMap("facility.pgm");
    ASSERT_FALSE(cells.empty());
    const Segmentation result = segmentRooms(cells, geometryOf(cells), {});
    for (const Region& region : result.regions)
    {
        RecordProperty("room_" + std::to_string(region.label), std::to_string(region.area));
    }
    EXPECT_GE(result.regions.size(), 4U);
    EXPECT_LE(result.regions.size(), 6U);
}

TEST(RoomSegmentation, FindsTheApartmentsRooms)
{
    // Six rooms off a hallway, doors 1.0 to 1.8 m. This is the full map, furniture included; the
    // node segments a walls-only grid, where the pockets behind the sofa are gone too.
    const cv::Mat cells = loadMap("apartment.pgm");
    ASSERT_FALSE(cells.empty());
    GridGeometry geometry     = geometryOf(cells);
    geometry.origin_x         = -5.5;
    geometry.origin_y         = -6.5;
    const Segmentation result = segmentRooms(cells, geometry, {});

    // The middle of each room from worlds/apartment.truth.yaml, each in a region of its own.
    const std::vector<cv::Point2d> middles{ { -1.0, 0.0 }, { 9.0, 0.0 },  { 6.0, 3.5 },
                                            { 12.0, 3.5 }, { 6.5, -3.5 }, { 12.5, -3.5 } };
    std::set<int>                  labels;
    for (const cv::Point2d& middle : middles)
    {
        const CellIndex cell  = geometry.toCell(middle.x, middle.y);
        const int       label = result.labels.at<int>(cell.y, cell.x);
        EXPECT_GT(label, 0) << "no room at " << middle;
        labels.insert(label);
    }
    EXPECT_EQ(labels.size(), middles.size());
    EXPECT_LE(result.regions.size(), 8U);
}

TEST(RoomSegmentation, ResamplesOntoAGrownMap)
{
    const cv::Mat      cells  = twoRooms(1.0);
    const GridGeometry from   = geometryOf(cells);
    const Segmentation result = segmentRooms(cells, from, {});

    GridGeometry grown = from;
    grown.origin_x -= 1.0;
    grown.width += cellsOf(2.0);
    const cv::Mat moved = resampleLabels(result.labels, from, grown);
    EXPECT_EQ(moved.cols, grown.width);
    EXPECT_EQ(
        moved.at<int>(cellsOf(2.0), cellsOf(3.5)),
        result.labels.at<int>(cellsOf(2.0), cellsOf(2.5)));
}

}  // namespace
}  // namespace g1_world_model
