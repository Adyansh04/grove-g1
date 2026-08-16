/**
 * @file test_authority_drift.cpp
 * @brief The BT's acquire sequence against g1_bringup's script, the other implementation of it.
 *
 * Two places now know how to take the arm: this package, in C++, for a mission, and
 * g1_bringup/scripts/activate_arm, in Python, for an operator. They must name the same
 * components and controllers, because a rename that reaches only one of them leaves the other
 * timing out against a component that no longer exists -- and it would time out at exactly the
 * moment a mission tried to grasp something.
 *
 * The script is read as text rather than imported: importing it needs rclpy and a graph, and
 * what is being compared is the names it declares, which are literals.
 */

#include <gmock/gmock.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "g1_orchestration/arm_authority.hpp"

namespace
{

std::string readScript()
{
    std::ifstream file(G1_ACTIVATE_ARM_SCRIPT);
    EXPECT_TRUE(file.is_open()) << "cannot read " << G1_ACTIVATE_ARM_SCRIPT;
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

}  // namespace

class AuthorityDrift : public ::testing::TestWithParam<g1_orchestration::ControlStack>
{
};

INSTANTIATE_TEST_SUITE_P(
    BothStacks, AuthorityDrift,
    ::testing::Values(
        g1_orchestration::ControlStack::kArmSdk, g1_orchestration::ControlStack::kLowCmd));

TEST_P(AuthorityDrift, EveryNameThisPackageUsesAppearsInTheScript)
{
    const std::string script = readScript();

    for (const g1_orchestration::ControlledPart& part :
         g1_orchestration::controlledParts(GetParam()))
    {
        // A part may legitimately have no component or nothing to displace, and an empty
        // string is a substring of everything, so those would assert nothing at all.
        if (!part.component.empty())
        {
            EXPECT_THAT(script, ::testing::HasSubstr(part.component))
                << part.component << " is not named in activate_arm";
        }
        EXPECT_THAT(script, ::testing::HasSubstr(part.controller))
            << part.controller << " is not named in activate_arm";
        if (!part.displaces.empty())
        {
            EXPECT_THAT(script, ::testing::HasSubstr(part.displaces))
                << part.displaces << " is not named in activate_arm";
        }
    }
}

TEST_P(AuthorityDrift, TheArmComesFirstAndBothHandsFollow)
{
    // Order is not cosmetic. The arm is the part whose failure fails the whole acquire, and
    // the hands are best-effort behind it, so the arm has to be parts.front().
    const auto& parts = g1_orchestration::controlledParts(GetParam());
    ASSERT_EQ(parts.size(), 3U);
    EXPECT_EQ(parts[0].controller, "arm_trajectory_controller");
    EXPECT_EQ(parts[1].component, "G1Dex3SystemLeft");
    EXPECT_EQ(parts[2].component, "G1Dex3SystemRight");
}

TEST(AuthorityDrift, TheArmsAreNeverUnownedOnTheLowCmdStack)
{
    // The whole reason the lowcmd arm entry carries a `displaces`: that component leaves any
    // unclaimed joint unpowered, so the freeze has to leave in the same switch the trajectory
    // controller arrives in. An empty `displaces` here would drop the arms.
    const auto& parts = g1_orchestration::controlledParts(g1_orchestration::ControlStack::kLowCmd);
    EXPECT_TRUE(parts.front().component.empty())
        << "the lowcmd body component is active from bring-up and must not be cycled";
    EXPECT_EQ(parts.front().displaces, "arm_freeze_controller");
}

TEST(AuthorityDrift, TheStackNameIsValidated)
{
    EXPECT_EQ(
        g1_orchestration::controlStackFromString("arm_sdk"),
        g1_orchestration::ControlStack::kArmSdk);
    EXPECT_EQ(
        g1_orchestration::controlStackFromString("lowcmd"),
        g1_orchestration::ControlStack::kLowCmd);
    // The cast is for clang-tidy: the function is [[nodiscard]] and EXPECT_THROW discards.
    EXPECT_THROW(
        static_cast<void>(g1_orchestration::controlStackFromString("low_cmd")),
        std::invalid_argument);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
