/**
 * @file test_freeze_pluginlib.cpp
 * @brief Verifies G1FreezeController is discoverable via pluginlib.
 */

#include <gmock/gmock.h>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>

/**
 * @brief Confirms g1_controllers/G1FreezeController resolves through pluginlib's ament-index
 * lookup, which is the path controller_manager uses to spawn it.
 */
TEST(G1FreezeControllerPluginlib, DiscoversAndInstantiates)
{
    pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
        "controller_interface",
        "controller_interface::ControllerInterface");

    ASSERT_TRUE(loader.isClassAvailable("g1_controllers/G1FreezeController"));

    auto instance = loader.createUniqueInstance("g1_controllers/G1FreezeController");
    ASSERT_NE(instance, nullptr);
}
