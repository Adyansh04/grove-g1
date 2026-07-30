#ifndef G1_HARDWARE_INTERFACE__MOTOR_CRC_HG_HPP_
#define G1_HARDWARE_INTERFACE__MOTOR_CRC_HG_HPP_

/**
 * @file motor_crc_hg.hpp
 * @brief Vendored CRC32 helpers for checksumming unitree_hg LowCmd messages
 *        before they are published.
 *
 * Vendored from unitreerobotics/unitree_ros2, commit
 * 668d1ec5a05d1c38d3306bdca7d59f2ba3581a88, paths
 * example/src/include/common/motor_crc_hg.h and
 * example/src/src/common/motor_crc_hg.cpp. BSD-3-Clause (see the repository
 * LICENSE). Wrapped in this package's namespace -- upstream uses the global
 * namespace, which risks colliding with another shared library's symbols of
 * the same name if both end up dlopen'd into the same controller_manager
 * process via pluginlib -- and renamed to this package's camelBack function
 * convention; the CRC algorithm and the raw struct's field layout (which
 * must mirror the firmware/motion-service's expected wire layout
 * bit-for-bit) are otherwise unmodified.
 */

#include <cstdint>

#include "unitree_hg/msg/low_cmd.hpp"

namespace g1_hardware_interface
{
namespace vendored
{

std::uint32_t crc32Core(const std::uint32_t* ptr, std::uint32_t len);
void          computeLowCmdCrc(unitree_hg::msg::LowCmd& msg);

}  // namespace vendored
}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__MOTOR_CRC_HG_HPP_
