#ifndef G1_STATE_ESTIMATION__LIVOX_CLOUD_HPP_
#define G1_STATE_ESTIMATION__LIVOX_CLOUD_HPP_

/**
 * @file livox_cloud.hpp
 * @brief Livox CustomMsg -> PointCloud2, hardware only, split out so it tests without a Mid360.
 */

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace g1_state_estimation
{

/**
 * @brief Fills @p cloud with @p custom's points as xyz + intensity, keeping header and frame.
 *
 * Per-point time and line are dropped; only FAST-LIO uses them, from the CustomMsg itself.
 */
void toPointCloud2(
    const livox_ros_driver2::msg::CustomMsg& custom, sensor_msgs::msg::PointCloud2& cloud);

}  // namespace g1_state_estimation

#endif  // G1_STATE_ESTIMATION__LIVOX_CLOUD_HPP_
