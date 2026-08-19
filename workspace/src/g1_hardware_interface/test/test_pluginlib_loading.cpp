/**
 * @file test_pluginlib_loading.cpp
 * @brief Verifies this package's hardware_interface plugin is discoverable via pluginlib.
 */

#include <gmock/gmock.h>

#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>

/**
 * @brief Confirms G1LowCmdSystem is discoverable through the same pluginlib lookup
 * controller_manager uses, rather than merely compiling.
 *
 * Also the canary for the SDK's RPATH: this dlopens the library, which links unitree_sdk2 and
 * its own CycloneDDS, so a broken DT_RPATH fails here rather than on the robot.
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
