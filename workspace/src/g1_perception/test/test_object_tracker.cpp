/**
 * @file test_object_tracker.cpp
 * @brief Pins the naming rule: an id has to keep meaning the same object while the camera moves.
 */

#include <gmock/gmock.h>

#include <string>
#include <vector>

#include "g1_perception/object_tracker.hpp"

namespace
{

using g1_perception::ObjectTracker;
using g1_perception::Observation;

Observation seen(const std::string& phrase, double x, double y)
{
    return { phrase, { x, y, 0.83 } };
}

TEST(Tracker, KeepsAnIdAcrossFrames)
{
    ObjectTracker                  tracker(0.05, 2.0);
    const std::vector<Observation> first{ seen("red block", 0.30, -0.20) };
    const std::vector<Observation> jittered{ seen("red block", 0.316, -0.212) };

    EXPECT_THAT(tracker.update(first, 0.0), testing::ElementsAre("red_block_0"));
    EXPECT_THAT(tracker.update(jittered, 1.0), testing::ElementsAre("red_block_0"));
    EXPECT_THAT(tracker.update(first, 2.0), testing::ElementsAre("red_block_0"));
}

TEST(Tracker, NumbersASecondInstance)
{
    ObjectTracker                  tracker(0.05, 2.0);
    const std::vector<Observation> one{ seen("red block", 0.30, -0.20) };
    const std::vector<Observation> two{ seen("red block", 0.30, -0.20),
                                        seen("red block", 0.42, -0.20) };

    EXPECT_THAT(tracker.update(one, 0.0), testing::ElementsAre("red_block_0"));
    EXPECT_THAT(tracker.update(two, 0.5), testing::ElementsAre("red_block_0", "red_block_1"));
}

TEST(Tracker, DoesNotSwapTwoNearbyInstances)
{
    ObjectTracker                  tracker(0.05, 2.0);
    const std::vector<Observation> frame_one{ seen("red block", 0.30, -0.20),
                                              seen("red block", 0.38, -0.20) };
    // Same two objects, reported in the other order and each nudged toward the other.
    const std::vector<Observation> frame_two{ seen("red block", 0.363, -0.204),
                                              seen("red block", 0.315, -0.196) };

    const std::vector<std::string> first  = tracker.update(frame_one, 0.0);
    const std::vector<std::string> second = tracker.update(frame_two, 0.4);

    ASSERT_THAT(first, testing::ElementsAre("red_block_0", "red_block_1"));
    EXPECT_THAT(second, testing::ElementsAre("red_block_1", "red_block_0"))
        << "the nearer track keeps the object, whatever order the detector reports them in";
}

TEST(Tracker, FreesAnIndexAfterTheTimeout)
{
    ObjectTracker                  tracker(0.05, 1.0);
    const std::vector<Observation> here{ seen("red block", 0.30, -0.20) };
    const std::vector<Observation> elsewhere{ seen("red block", 0.90, 0.40) };

    EXPECT_THAT(tracker.update(here, 0.0), testing::ElementsAre("red_block_0"));
    EXPECT_THAT(tracker.update(elsewhere, 5.0), testing::ElementsAre("red_block_0"))
        << "the old track timed out, so its index is free again";
}

TEST(Tracker, KeepsIndicesApartForDifferentPhrases)
{
    ObjectTracker                  tracker(0.05, 2.0);
    const std::vector<Observation> mixed{ seen("red block", 0.30, -0.20),
                                          seen("blue sphere", 0.42, -0.20) };

    EXPECT_THAT(tracker.update(mixed, 0.0), testing::ElementsAre("red_block_0", "blue_sphere_0"));
}

TEST(Tracker, OnlyAliasesASoleInstance)
{
    ObjectTracker                  tracker(0.05, 2.0);
    const std::vector<Observation> one{ seen("red block", 0.30, -0.20) };
    const std::vector<Observation> two{ seen("red block", 0.30, -0.20),
                                        seen("red block", 0.42, -0.20) };

    tracker.update(one, 0.0);
    EXPECT_TRUE(tracker.isSoleTrackFor("red block"));
    tracker.update(two, 0.5);
    EXPECT_FALSE(tracker.isSoleTrackFor("red block"));
    EXPECT_FALSE(tracker.isSoleTrackFor("green cylinder"));
}

TEST(Tracker, RecoversThePhraseFromAnId)
{
    EXPECT_EQ(ObjectTracker::phraseOf(ObjectTracker::idFor("red block", 3)), "red_block");
    EXPECT_EQ(ObjectTracker::phraseOf("white_cup_12"), "white_cup");
    // Not an id: a bare phrase, a word that happens to contain an underscore, a raw label.
    EXPECT_EQ(ObjectTracker::phraseOf("red_block"), "red_block");
    EXPECT_EQ(ObjectTracker::phraseOf("cube_top"), "cube_top");
    EXPECT_EQ(ObjectTracker::phraseOf("green cylinder"), "green cylinder");
    EXPECT_EQ(ObjectTracker::phraseOf("_0"), "_0");
    EXPECT_EQ(ObjectTracker::phraseOf("cube_"), "cube_");
}

}  // namespace
