/**
 * @file test_authority_drift.cpp
 * @brief The acquire sequence matches g1_bringup's activate_arm, and never leaves the arm unowned.
 *
 * The script is read as text: importing it needs rclpy and a graph.
 */

#include <gmock/gmock.h>

#include <format>
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
    const std::string                       script = readScript();
    const g1_orchestration::ControlledPart& arm    = g1_orchestration::armPart();
    for (const std::string& name : { arm.controller, arm.displaces })
    {
        EXPECT_THAT(script, ::testing::HasSubstr(name)) << name << " is not named in activate_arm";
    }
    for (const char* hand : g1_orchestration::kHandControllers)
    {
        EXPECT_THAT(script, ::testing::HasSubstr(hand)) << hand << " is not named in activate_arm";
    }
}

TEST(AuthorityDrift, TheTimeoutsEqualTheScripts)
{
    const std::string script = readScript();

    EXPECT_THAT(
        script,
        ::testing::HasSubstr(std::format("SETTLE_S = {:.1f}", g1_orchestration::kAcquireSettleS)));
    EXPECT_THAT(
        script,
        ::testing::HasSubstr(
            std::format("HAND_ACTIVATE_TIMEOUT_S = {:.1f}", g1_orchestration::kHandTimeoutS)));
}

TEST(AuthorityDrift, TheArmsAreNeverUnowned)
{
    // An unclaimed joint is unpowered, so the freeze must leave in the same switch the
    // trajectory controller arrives in.
    const g1_orchestration::ControlledPart& arm = g1_orchestration::armPart();
    EXPECT_TRUE(arm.component.empty())
        << "the body component is active from bring-up and must not be cycled";
    EXPECT_EQ(arm.displaces, "arm_freeze_controller");
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

namespace
{

using controller_manager_msgs::srv::ListControllers;
using controller_manager_msgs::srv::ListHardwareComponents;

ListHardwareComponents::Response
components(const std::vector<std::pair<std::string, std::vector<std::string>>>& owned)
{
    ListHardwareComponents::Response response;
    for (const auto& [name, interfaces] : owned)
    {
        controller_manager_msgs::msg::HardwareComponentState component;
        component.name = name;
        for (const std::string& interface : interfaces)
        {
            controller_manager_msgs::msg::HardwareInterface command;
            command.name = interface;
            component.command_interfaces.push_back(command);
        }
        response.component.push_back(component);
    }
    return response;
}

ListControllers::Response
controllers(const std::vector<std::pair<std::string, std::vector<std::string>>>& required)
{
    ListControllers::Response response;
    for (const auto& [name, interfaces] : required)
    {
        controller_manager_msgs::msg::ControllerState state;
        state.name                        = name;
        state.required_command_interfaces = interfaces;
        response.controller.push_back(state);
    }
    return response;
}

}  // namespace

TEST(HandPairing, EachHandControllerGetsTheComponentOwningItsJoints)
{
    // Inspire names, listed right before left and among the body: the joints decide, not order.
    const auto pairs = g1_orchestration::pairHands(
        controllers({
            { "arm_trajectory_controller", { "left_elbow_joint/position" } },
            { "right_hand_controller", { "R_index_proximal_joint/position" } },
            { "left_hand_controller", { "L_index_proximal_joint/position" } },
        }),
        components({
            { "G1LowCmdSystem", { "left_elbow_joint/position" } },
            { "G1InspireSystemRight", { "R_index_proximal_joint/position" } },
            { "G1InspireSystemLeft", { "L_index_proximal_joint/position" } },
        }));
    ASSERT_EQ(pairs.size(), 2U);
    EXPECT_EQ(pairs[0].component, "G1InspireSystemLeft");
    EXPECT_EQ(pairs[0].controller, "left_hand_controller");
    EXPECT_EQ(pairs[1].component, "G1InspireSystemRight");
    EXPECT_TRUE(pairs[1].displaces.empty());
}

TEST(HandPairing, AHandWithoutAComponentOrAControllerIsLeftOut)
{
    // hands:=false loads no hand controllers; a component may also cover only part of a hand.
    EXPECT_TRUE(g1_orchestration::pairHands(controllers({}), components({})).empty());
    const auto partial = g1_orchestration::pairHands(
        controllers({ { "left_hand_controller", { "a/position", "b/position" } } }),
        components({ { "G1Dex3SystemLeft", { "a/position" } } }));
    EXPECT_TRUE(partial.empty());
}
