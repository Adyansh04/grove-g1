#ifndef G1_HARDWARE_INTERFACE__MOTOR_CRC_HG_HPP_
#define G1_HARDWARE_INTERFACE__MOTOR_CRC_HG_HPP_

/**
 * @file motor_crc_hg.hpp
 * @brief Vendored CRC32 bit loop for checksumming LowCmd frames before publishing.
 *
 * Vendored from unitreerobotics/unitree_ros2, commit
 * 668d1ec5a05d1c38d3306bdca7d59f2ba3581a88, paths
 * example/src/include/common/motor_crc_hg.h and
 * example/src/src/common/motor_crc_hg.cpp. BSD-3-Clause (see the repository
 * LICENSE). Wrapped in this package's namespace, because upstream uses the global
 * namespace, which risks colliding with another shared library's symbols of
 * the same name if both end up dlopen'd into the same controller_manager
 * process via pluginlib, and renamed to this package's camelBack function
 * convention; the algorithm is otherwise unmodified.
 *
 * What gets checksummed lives in lowcmd_assembly: this is only the sum.
 */

#include <cstdint>

namespace g1_hardware_interface::vendored
{

std::uint32_t crc32Core(const std::uint32_t* ptr, std::uint32_t len);

}  // namespace g1_hardware_interface::vendored

#endif  // G1_HARDWARE_INTERFACE__MOTOR_CRC_HG_HPP_
