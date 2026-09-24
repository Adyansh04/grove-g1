/**
 * @file test_pluginlib_loading.cpp
 * @brief Verifies this package's hardware_interface plugin is discoverable via pluginlib.
 */

#include <gmock/gmock.h>

#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>

/**
 * @brief G1LowCmdSystem resolves through the pluginlib lookup controller_manager uses.
 *
 * Also the DT_RPATH canary: the dlopen pulls in unitree_sdk2 and its own CycloneDDS.
 */
TEST(G1LowCmdSystemPluginlib, DiscoversAndInstantiates)
{
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
        "hardware_interface",
        "hardware_interface::SystemInterface");

    ASSERT_TRUE(loader.isClassAvailable("g1_hardware_interface/G1LowCmdSystem"));

    auto instance = loader.createUniqueInstance("g1_hardware_interface/G1LowCmdSystem");
    ASSERT_NE(instance, nullptr);
}
