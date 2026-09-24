/**
 * @file test_authority_drift.cpp
 * @brief The acquire sequence matches g1_bringup's activate_arm, and never leaves the arm unowned.
 *
 * The script is read as text: importing it needs rclpy and a graph.
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
        // Empty names are skipped: an empty string matches anything.
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

TEST(AuthorityDrift, TheArmComesFirstAndBothHandsFollow)
{
    // acquireArm treats parts.front() as the required arm; the hands behind it are best-effort.
    const auto& parts = g1_orchestration::controlledParts();
    ASSERT_EQ(parts.size(), 3U);
    EXPECT_EQ(parts[0].controller, "arm_trajectory_controller");
    EXPECT_EQ(parts[1].component, "G1Dex3SystemLeft");
    EXPECT_EQ(parts[2].component, "G1Dex3SystemRight");
}

TEST(AuthorityDrift, TheArmsAreNeverUnowned)
{
    // An unclaimed joint is unpowered, so the freeze must leave in the same switch the
    // trajectory controller arrives in.
    const auto& parts = g1_orchestration::controlledParts();
    EXPECT_TRUE(parts.front().component.empty())
        << "the body component is active from bring-up and must not be cycled";
    EXPECT_EQ(parts.front().displaces, "arm_freeze_controller");
}

TEST(ArmSwitch, AnIncomingControllerThatIsNotLoadedSwitchesNothingAtAll)
{
    // Nothing may be asked for: deactivating the freeze alone would leave the arm unpowered.
    const auto plan = g1_orchestration::planArmSwitch("", "active");
    EXPECT_FALSE(plan.possible) << "the holder must be left alone when nothing can replace it";
}

TEST(ArmSwitch, ALoadedButUnconfiguredControllerIsStillWorthTrying)
{
    // Between load and configure the controller exists but may not activate; STRICT then does
    // both halves or neither.
    const auto plan = g1_orchestration::planArmSwitch("unconfigured", "active");
    EXPECT_TRUE(plan.possible);
    EXPECT_FALSE(plan.already_held);
    EXPECT_TRUE(plan.displace);
}

TEST(ArmSwitch, AnArmAlreadyHeldIsLeftAlone)
{
    // STRICT fails on activating an active controller, and AcquireArm may not be the first.
    const auto plan = g1_orchestration::planArmSwitch("active", "inactive");
    EXPECT_TRUE(plan.possible);
    EXPECT_TRUE(plan.already_held);
}

TEST(ArmSwitch, NothingIsDeactivatedWhenTheOutgoingControllerIsNotHoldingAnything)
{
    // STRICT fails on deactivating an inactive controller, so a second release only activates.
    const auto plan = g1_orchestration::planArmSwitch("inactive", "inactive");
    EXPECT_TRUE(plan.possible);
    EXPECT_FALSE(plan.displace);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
