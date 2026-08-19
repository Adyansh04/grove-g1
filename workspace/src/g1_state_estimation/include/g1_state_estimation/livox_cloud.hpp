#ifndef G1_STATE_ESTIMATION__LIVOX_CLOUD_HPP_
#define G1_STATE_ESTIMATION__LIVOX_CLOUD_HPP_

/**
 * @file livox_cloud.hpp
 * @brief The Livox CustomMsg -> PointCloud2 conversion, split out so it can be tested.
 *
 * Hardware-only code: in simulation the relay publishes the PointCloud2 directly. Nothing in
 * the sim acceptance test reaches it, so it gets its own unit test instead.
 */

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace g1_state_estimation
{

/**
 * @brief Fills @p cloud with @p custom's points as xyz + intensity, keeping header and frame.
 *
 * The per-point time and line index are dropped: only FAST-LIO wants them and it reads the
 * CustomMsg itself. Reflectivity becomes intensity, the field every consumer already reads.
 */
void toPointCloud2(
    const livox_ros_driver2::msg::CustomMsg& custom, sensor_msgs::msg::PointCloud2& cloud);

}  // namespace g1_state_estimation

#endif  // G1_STATE_ESTIMATION__LIVOX_CLOUD_HPP_
