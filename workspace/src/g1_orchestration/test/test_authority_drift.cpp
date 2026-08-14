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

TEST(AuthorityDrift, EveryNameThisPackageUsesAppearsInTheScript)
{
    const std::string script = readScript();

    for (const g1_orchestration::ControlledPart& part : g1_orchestration::controlledParts())
    {
        EXPECT_THAT(script, ::testing::HasSubstr(part.component))
            << part.component << " is not named in activate_arm";
        EXPECT_THAT(script, ::testing::HasSubstr(part.controller))
            << part.controller << " is not named in activate_arm";
    }
}

TEST(AuthorityDrift, TheArmComesFirstAndBothHandsFollow)
{
    // Order is not cosmetic. The arm is the part whose failure fails the whole acquire, and
    // the hands are best-effort behind it, so the arm has to be parts.front().
    const auto& parts = g1_orchestration::controlledParts();
    ASSERT_EQ(parts.size(), 3U);
    EXPECT_EQ(parts[0].component, "G1ArmSdkSystem");
    EXPECT_EQ(parts[1].component, "G1Dex3SystemLeft");
    EXPECT_EQ(parts[2].component, "G1Dex3SystemRight");
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
