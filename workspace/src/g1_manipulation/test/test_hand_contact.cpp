/**
 * @file test_hand_contact.cpp
 * @brief The allowed-collision-matrix arithmetic, without a planning scene.
 *
 * The matrix is square and index-addressed, so the failures worth pinning are a grown name list
 * with an ungrown row, and a half-written symmetric pair. Both read as an exemption that did
 * nothing.
 */

#include <gmock/gmock.h>

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "g1_manipulation/hand_contact.hpp"

using g1_manipulation::editHandContact;
using moveit_msgs::msg::AllowedCollisionEntry;
using moveit_msgs::msg::AllowedCollisionMatrix;

namespace
{

/// A square matrix over the given names with nothing allowed, the shape move_group answers with.
AllowedCollisionMatrix matrixOver(const std::vector<std::string>& names)
{
    AllowedCollisionMatrix acm;
    acm.entry_names = names;
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        AllowedCollisionEntry row;
        row.enabled.assign(names.size(), false);
        acm.entry_values.push_back(row);
    }
    return acm;
}

std::size_t indexOf(const AllowedCollisionMatrix& acm, const std::string& name)
{
    const auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
    return static_cast<std::size_t>(it - acm.entry_names.begin());
}

bool allowed(const AllowedCollisionMatrix& acm, const std::string& a, const std::string& b)
{
    return acm.entry_values[indexOf(acm, a)].enabled[indexOf(acm, b)];
}

/// Every row as long as the name list, which is what the index arithmetic assumes.
bool isSquare(const AllowedCollisionMatrix& acm)
{
    if (acm.entry_values.size() != acm.entry_names.size())
    {
        return false;
    }
    return std::all_of(
        acm.entry_values.begin(),
        acm.entry_values.end(),
        [&acm](const AllowedCollisionEntry& row) {
            return row.enabled.size() == acm.entry_names.size();
        });
}

const std::vector<std::string> kHandLinks = { "right_hand_palm_link", "right_wrist_roll_link" };

}  // namespace

TEST(HandContact, ExemptsHandLinksFromAnUnknownObject)
{
    AllowedCollisionMatrix acm = matrixOver(kHandLinks);

    editHandContact(acm, kHandLinks, { "<octomap>" }, true, true);

    EXPECT_TRUE(isSquare(acm));
    EXPECT_TRUE(allowed(acm, "right_hand_palm_link", "<octomap>"));
    EXPECT_TRUE(allowed(acm, "right_wrist_roll_link", "<octomap>"));
}

TEST(HandContact, WritesBothHalvesOfEveryPair)
{
    AllowedCollisionMatrix acm = matrixOver(kHandLinks);

    editHandContact(acm, kHandLinks, { "<octomap>", "red_cube" }, true, true);

    EXPECT_TRUE(allowed(acm, "<octomap>", "right_hand_palm_link"));
    EXPECT_TRUE(allowed(acm, "right_hand_palm_link", "<octomap>"));
    // The touchables against each other, not only against the hand.
    EXPECT_TRUE(allowed(acm, "red_cube", "<octomap>"));
    EXPECT_TRUE(allowed(acm, "<octomap>", "red_cube"));
}

TEST(HandContact, WithoutIncludeLinksLeavesTheHandChecked)
{
    AllowedCollisionMatrix acm = matrixOver(kHandLinks);

    editHandContact(acm, kHandLinks, { "<octomap>", "red_cube" }, true, false);

    EXPECT_TRUE(allowed(acm, "red_cube", "<octomap>"));
    EXPECT_FALSE(allowed(acm, "right_hand_palm_link", "<octomap>"));
    EXPECT_FALSE(allowed(acm, "right_wrist_roll_link", "red_cube"));
}

TEST(HandContact, RestoreClearsWhatTheApplySet)
{
    AllowedCollisionMatrix acm = matrixOver(kHandLinks);

    editHandContact(acm, kHandLinks, { "<octomap>" }, true, true);
    editHandContact(acm, kHandLinks, { "<octomap>" }, false, true);

    EXPECT_FALSE(allowed(acm, "right_hand_palm_link", "<octomap>"));
    EXPECT_FALSE(allowed(acm, "<octomap>", "right_wrist_roll_link"));
}

TEST(HandContact, KeepsRulesItDidNotTouch)
{
    AllowedCollisionMatrix acm = matrixOver({ "right_hand_palm_link", "pelvis", "waist_yaw_link" });
    // A self-collision rule of the kind the SRDF sets up.
    acm.entry_values[indexOf(acm, "pelvis")].enabled[indexOf(acm, "waist_yaw_link")] = true;
    acm.entry_values[indexOf(acm, "waist_yaw_link")].enabled[indexOf(acm, "pelvis")] = true;

    editHandContact(acm, kHandLinks, { "<octomap>" }, true, true);

    EXPECT_TRUE(allowed(acm, "pelvis", "waist_yaw_link"));
    EXPECT_TRUE(allowed(acm, "right_hand_palm_link", "<octomap>"));
}

TEST(HandContact, IgnoresLinksTheMatrixDoesNotHave)
{
    AllowedCollisionMatrix acm = matrixOver({ "right_hand_palm_link" });

    editHandContact(acm, kHandLinks, { "<octomap>" }, true, true);

    EXPECT_TRUE(isSquare(acm));
    EXPECT_EQ(acm.entry_names.size(), 2U);
    EXPECT_TRUE(allowed(acm, "right_hand_palm_link", "<octomap>"));
}
