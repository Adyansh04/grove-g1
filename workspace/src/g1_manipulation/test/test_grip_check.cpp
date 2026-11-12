/**
 * @file test_grip_check.cpp
 * @brief Pins what counts as holding something, so a pick cannot report a grip it does not have.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <vector>

#include "g1_manipulation/grip_check.hpp"

namespace
{

using g1_manipulation::JointGrip;
using g1_manipulation::verifyGrip;

constexpr double kMinError  = 0.08;
constexpr double kMinEffort = 0.10;

/// The right hand's seven joints at their commanded `closed` posture, reached with no load.
/// thumb_0 rolls the thumb and is the same in `open` and `closed`, so the caller marks it.
std::vector<JointGrip> emptyHand()
{
    return {
        { "right_hand_thumb_0_joint", 0.00, 0.00, 0.0, false },
        { "right_hand_thumb_1_joint", 0.00, 0.00, 0.0 },
        { "right_hand_thumb_2_joint", -1.40, -1.40, 0.0 },
        { "right_hand_middle_0_joint", 1.20, 1.20, 0.0 },
        { "right_hand_middle_1_joint", 1.40, 1.40, 0.0 },
        { "right_hand_index_0_joint", 1.20, 1.20, 0.0 },
        { "right_hand_index_1_joint", 1.40, 1.40, 0.0 },
    };
}

/// Stalls one joint short of its target, pushing.
void load(std::vector<JointGrip>& hand, const std::string& joint, double shortfall, double effort)
{
    for (JointGrip& candidate : hand)
    {
        if (candidate.name.find(joint) != std::string::npos)
        {
            candidate.position = candidate.target - std::copysign(shortfall, candidate.target);
            candidate.effort   = effort;
        }
    }
}

TEST(GripCheck, AHandThatClosedOnNothingIsNotHolding)
{
    const auto verdict = verifyGrip(emptyHand(), kMinError, kMinEffort);

    EXPECT_FALSE(verdict.holding);
    EXPECT_THAT(verdict.why, testing::HasSubstr("unloaded"));
}

TEST(GripCheck, ThumbAgainstAFingerIsHolding)
{
    std::vector<JointGrip> hand = emptyHand();
    load(hand, "thumb_2", 0.34, 0.51);
    load(hand, "index_1", 0.57, 0.86);

    const auto verdict = verifyGrip(hand, kMinError, kMinEffort);

    EXPECT_TRUE(verdict.holding);
    EXPECT_TRUE(verdict.opposed);
    EXPECT_THAT(verdict.why, testing::HasSubstr("2 fingers are pressing"));
}

TEST(GripCheck, IndexAndMiddleAgainstThePalmAreHolding)
{
    // Two fingers pressing an object against the palm still hold it.
    std::vector<JointGrip> hand = emptyHand();
    load(hand, "index_1", 0.5, 0.8);
    load(hand, "middle_1", 0.5, 0.8);

    const auto verdict = verifyGrip(hand, kMinError, kMinEffort);
    EXPECT_TRUE(verdict.holding);
    // Not opposed, though: without the thumb, a close still creeping in keeps going.
    EXPECT_FALSE(verdict.opposed);
}

TEST(GripCheck, OneFingerAloneIsNotHolding)
{
    std::vector<JointGrip> hand = emptyHand();
    load(hand, "index_1", 0.5, 0.8);

    const auto verdict = verifyGrip(hand, kMinError, kMinEffort);

    EXPECT_FALSE(verdict.holding);
    EXPECT_THAT(verdict.why, testing::HasSubstr("nothing opposes it"));
}

TEST(GripCheck, ShortWithoutTorqueIsASlackDriveRatherThanAGrip)
{
    std::vector<JointGrip> hand = emptyHand();
    load(hand, "thumb_2", 0.4, 0.0);
    load(hand, "index_1", 0.4, 0.0);

    EXPECT_FALSE(verifyGrip(hand, kMinError, kMinEffort).holding);
}

TEST(GripCheck, TorqueAtTheTargetIsNotAGrip)
{
    // A finger holding its own weight at the end of the trajectory: pushing, but not blocked.
    std::vector<JointGrip> hand = emptyHand();
    load(hand, "thumb_2", 0.0, 1.2);
    load(hand, "index_1", 0.0, 1.2);

    const auto verdict = verifyGrip(hand, kMinError, kMinEffort);
    EXPECT_FALSE(verdict.holding);
    // Not "unloaded": the fingers push hard enough, they just are not blocked.
    EXPECT_THAT(verdict.why, testing::HasSubstr("no finger is blocked"));
}

TEST(GripCheck, ThumbRollDoesNotCountAsAFinger)
{
    // A joint the close does not move, such as thumb_0's roll, can stall on its own travel with
    // the hand empty.
    std::vector<JointGrip> hand = emptyHand();
    load(hand, "thumb_0", 0.5, 1.0);
    load(hand, "index_1", 0.5, 1.0);

    EXPECT_FALSE(verifyGrip(hand, kMinError, kMinEffort).holding);
}

TEST(GripCheck, TheLeftHandReadsTheSameWithItsSignsFlipped)
{
    std::vector<JointGrip> hand = {
        { "left_hand_thumb_2_joint", 1.40, 1.06, -0.51 },
        { "left_hand_index_1_joint", -1.40, -0.83, 0.86 },
    };

    EXPECT_TRUE(verifyGrip(hand, kMinError, kMinEffort).holding);
}

TEST(GripCheck, NonFiniteReadingsAreNotAGrip)
{
    std::vector<JointGrip> hand = emptyHand();
    load(hand, "thumb_2", 0.4, 0.6);
    load(hand, "index_1", 0.4, 0.6);
    hand.front().position = std::nan("");
    for (JointGrip& joint : hand)
    {
        if (joint.name.find("index_1") != std::string::npos)
        {
            joint.effort = std::nan("");
        }
    }

    EXPECT_FALSE(verifyGrip(hand, kMinError, kMinEffort).holding);
}

}  // namespace

/// An Inspire right hand at its `closed` posture, empty: four fingers and the thumb's bend close,
/// and the thumb's rotation is held at 1.3 rad by both postures.
std::vector<JointGrip> emptyInspireHand()
{
    return {
        { "R_pinky_proximal_joint", 0.80, 0.80, 0.0 },
        { "R_ring_proximal_joint", 0.80, 0.80, 0.0 },
        { "R_middle_proximal_joint", 0.80, 0.80, 0.0 },
        { "R_index_proximal_joint", 0.80, 0.80, 0.0 },
        { "R_thumb_proximal_pitch_joint", 0.22, 0.22, 0.0 },
        { "R_thumb_proximal_yaw_joint", 1.30, 1.30, 0.0, false },
    };
}

TEST(GripCheck, AnInspireThumbAgainstTheIndexIsAnOpposedGrip)
{
    std::vector<JointGrip> hand = emptyInspireHand();
    load(hand, "thumb_proximal_pitch", 0.10, 0.20);
    load(hand, "index_proximal", 0.30, 0.15);

    const auto verdict = verifyGrip(hand, kMinError, kMinEffort);
    EXPECT_TRUE(verdict.holding);
    EXPECT_TRUE(verdict.opposed);
}

TEST(GripCheck, RingAndPinkyAloneHoldButOpposeNothing)
{
    std::vector<JointGrip> hand = emptyInspireHand();
    load(hand, "ring_proximal", 0.30, 0.15);
    load(hand, "pinky_proximal", 0.30, 0.15);

    const auto verdict = verifyGrip(hand, kMinError, kMinEffort);
    EXPECT_TRUE(verdict.holding);
    EXPECT_FALSE(verdict.opposed);
}

TEST(GripCheck, AnInspireThumbRotationStalledOnItsTravelIsNotAFinger)
{
    std::vector<JointGrip> hand = emptyInspireHand();
    load(hand, "thumb_proximal_yaw", 0.40, 0.30);
    load(hand, "index_proximal", 0.30, 0.15);

    EXPECT_FALSE(verifyGrip(hand, kMinError, kMinEffort).holding);
}
