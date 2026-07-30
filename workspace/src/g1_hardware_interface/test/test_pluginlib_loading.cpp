/**
 * @file test_pluginlib_loading.cpp
 * @brief Verifies the G1ArmSdkSystem hardware_interface plugin is discoverable via pluginlib.
 */

#include <gmock/gmock.h>

#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>

/**
 * @brief Confirms g1_hardware_interface/G1ArmSdkSystem is actually discoverable through
 * pluginlib's ament-index plugin description lookup -- the same path controller_manager
 * uses -- rather than merely compiling.
 *
 * This is the discovery proof for the plugin export wiring.
 */
TEST(G1ArmSdkSystemPluginlib, DiscoversAndInstantiates)
{
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
        "hardware_interface",
        "hardware_interface::SystemInterface");

    ASSERT_TRUE(loader.isClassAvailable("g1_hardware_interface/G1ArmSdkSystem"));

    auto instance = loader.createUniqueInstance("g1_hardware_interface/G1ArmSdkSystem");
    ASSERT_NE(instance, nullptr);
}
