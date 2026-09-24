#ifndef G1_HARDWARE_INTERFACE__MOTOR_CRC_HG_HPP_
#define G1_HARDWARE_INTERFACE__MOTOR_CRC_HG_HPP_

/**
 * @file motor_crc_hg.hpp
 * @brief Vendored CRC32 bit loop for checksumming LowCmd frames before publishing.
 *
 * From unitreerobotics/unitree_ros2 @ 668d1ec5a05d1c38d3306bdca7d59f2ba3581a88,
 * example/src/{include,src}/common/motor_crc_hg.{h,cpp}, BSD-3-Clause. Moved out of the global
 * namespace, where it could collide inside controller_manager, and renamed; the algorithm is
 * unchanged. Which bytes are summed is lowcmd_assembly's job.
 */

#include <cstdint>

namespace g1_hardware_interface::vendored
{

std::uint32_t crc32Core(const std::uint32_t* ptr, std::uint32_t len);

}  // namespace g1_hardware_interface::vendored

#endif  // G1_HARDWARE_INTERFACE__MOTOR_CRC_HG_HPP_
