/**
 * @file test_authority_drift.cpp
 * @brief The BT's acquire sequence against g1_bringup's script, the other implementation of it.
 *
 * Two places know how to take the arm: this package, in C++, for a mission, and
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

TEST(AuthorityDrift, TheArmComesFirstAndBothHandsFollow)
{
    // Order is not cosmetic. The arm is the part whose failure fails the whole acquire, and
    // the hands are best-effort behind it, so the arm has to be parts.front().
    const auto& parts = g1_orchestration::controlledParts();
    ASSERT_EQ(parts.size(), 3U);
    EXPECT_EQ(parts[0].controller, "arm_trajectory_controller");
    EXPECT_EQ(parts[1].component, "G1Dex3SystemLeft");
    EXPECT_EQ(parts[2].component, "G1Dex3SystemRight");
}

TEST(AuthorityDrift, TheArmsAreNeverUnowned)
{
    // The whole reason the arm entry carries a `displaces`: the body component leaves any
    // unclaimed joint unpowered, so the freeze has to leave in the same switch the trajectory
    // controller arrives in. An empty `displaces` here would drop the arms.
    const auto& parts = g1_orchestration::controlledParts();
    EXPECT_TRUE(parts.front().component.empty())
        << "the body component is active from bring-up and must not be cycled";
    EXPECT_EQ(parts.front().displaces, "arm_freeze_controller");
}

TEST(ArmSwitch, AnIncomingControllerThatIsNotLoadedSwitchesNothingAtAll)
{
    // The one that drops the arms. controller_manager's BEST_EFFORT applies whichever half of a
    // paired switch it can and still answers ok, so asking it to displace the freeze for a
    // controller it cannot activate deactivates the freeze alone -- and the body component
    // leaves the fifteen arm joints unclaimed, which on this robot means unpowered.
    const auto plan = g1_orchestration::planArmSwitch("", "active");
    EXPECT_FALSE(plan.possible) << "the holder must be left alone when nothing can replace it";
}

TEST(ArmSwitch, ALoadedButUnconfiguredControllerIsStillWorthTrying)
{
    // Distinct from the case above, and it is the bring-up race the mission's AcquireArm exists
    // for: the controller is spawned --inactive, so between load and configure it is present but
    // not usable. The switch is issued STRICT, which either performs both halves or neither.
    const auto plan = g1_orchestration::planArmSwitch("unconfigured", "active");
    EXPECT_TRUE(plan.possible);
    EXPECT_FALSE(plan.already_held);
    EXPECT_TRUE(plan.displace);
}

TEST(ArmSwitch, AnArmAlreadyHeldIsLeftAlone)
{
    // What BEST_EFFORT was originally chosen for. AcquireArm runs as a tree leaf and cannot
    // assume it is first, and STRICT calls activating an already-active controller a failure --
    // so the already-correct case is answered here rather than sent to the service.
    const auto plan = g1_orchestration::planArmSwitch("active", "inactive");
    EXPECT_TRUE(plan.possible);
    EXPECT_TRUE(plan.already_held);
}

TEST(ArmSwitch, NothingIsDeactivatedWhenTheOutgoingControllerIsNotHoldingAnything)
{
    // A STRICT switch fails outright if asked to deactivate a controller that is already
    // inactive, so a second release must ask only for the activation.
    const auto plan = g1_orchestration::planArmSwitch("inactive", "inactive");
    EXPECT_TRUE(plan.possible);
    EXPECT_FALSE(plan.displace);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
