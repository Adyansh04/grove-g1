#ifndef G1_SENSOR_RELAY__LIVOX_CUSTOM_MSG_HPP_
#define G1_SENSOR_RELAY__LIVOX_CUSTOM_MSG_HPP_

/**
 * @file livox_custom_msg.hpp
 * @brief PointCloud2 -> Livox CustomMsg, split out so the conversion tests without a graph.
 *
 * The mirror of this conversion on the hardware side (g1_state_estimation's livox_cloud) was
 * extracted for the same reason, and the test written against it found that it had never once
 * produced a valid message. Field-by-field conversions look obviously correct and are not.
 */

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace g1_sensor_relay
{

/**
 * @brief Whether the cloud carries x, y and z as FLOAT32, which the iterators require.
 */
bool hasXyzFloatFields(const sensor_msgs::msg::PointCloud2& cloud);

/**
 * @brief Restates @p cloud as the CustomMsg FAST-LIO consumes.
 *
 * Non-finite points are dropped, so `point_num` counts what is actually there. Returns false
 * and leaves @p out untouched when the cloud has no usable x/y/z fields.
 */
bool toCustomMsg(const sensor_msgs::msg::PointCloud2& cloud, livox_ros_driver2::msg::CustomMsg& out);

}  // namespace g1_sensor_relay

#endif  // G1_SENSOR_RELAY__LIVOX_CUSTOM_MSG_HPP_
