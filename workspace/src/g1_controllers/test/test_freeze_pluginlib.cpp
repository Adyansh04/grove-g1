/**
 * @file test_freeze_pluginlib.cpp
 * @brief Verifies this package's controllers are discoverable via pluginlib.
 */

#include <gmock/gmock.h>

#include <controller_interface/chainable_controller_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>

namespace
{

TEST(G1ControllersPluginlib, PlainControllersResolveAndInstantiate)
{
    pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
        "controller_interface",
        "controller_interface::ControllerInterface");

    for (const auto* name :
         { "g1_controllers/G1FreezeController", "g1_controllers/G1AgileController" })
    {
        ASSERT_TRUE(loader.isClassAvailable(name)) << name;
        // Shared, not unique: the unique-instance deleter trips clang-analyzer inside pluginlib.
        EXPECT_NE(loader.createSharedInstance(name), nullptr) << name;
    }
}

TEST(G1ControllersPluginlib, SafetyControllerResolvesAsChainable)
{
    // Registered against the chainable base, so controller_manager will let another controller
    // claim its reference interfaces.
    pluginlib::ClassLoader<controller_interface::ChainableControllerInterface> loader(
        "controller_interface",
        "controller_interface::ChainableControllerInterface");

    ASSERT_TRUE(loader.isClassAvailable("g1_controllers/G1SafetyController"));
    EXPECT_NE(loader.createSharedInstance("g1_controllers/G1SafetyController"), nullptr);
}

}  // namespace
