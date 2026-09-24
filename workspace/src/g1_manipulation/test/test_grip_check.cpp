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
std::vector<JointGrip> emptyHand()
{
    return {
        { "right_hand_thumb_0_joint", 0.00, 0.00, 0.0 },
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

    EXPECT_FALSE(verifyGrip(hand, kMinError, kMinEffort).holding);
}

TEST(GripCheck, ThumbRollDoesNotCountAsAFinger)
{
    // thumb_0 rolls the thumb and can stall on its own travel with the hand empty.
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
