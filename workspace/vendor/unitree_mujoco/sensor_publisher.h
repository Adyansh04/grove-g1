#ifndef GROVE_G1_SENSOR_PUBLISHER_H_
#define GROVE_G1_SENSOR_PUBLISHER_H_

// Sensor sampling inside unitree_mujoco: LiDAR sweep, depth and colour camera, the Mid360 IMU,
// ground-truth base state and object poses. Lives outside the vendored sources so the patch
// against them stays a few lines; see workspace/patches/unitree_mujoco/README.md.
//
// Links no ROS and no DDS: unitree_sdk2 already owns a CycloneDDS domain in this process. Frames
// go over a local socket to g1_sensor_relay, an ordinary ROS node that publishes them.
//
// Off unless GROVE_G1_SENSOR_CONFIG names a config file; unset, the patched binary behaves like
// the stock one.

#include <mujoco/mujoco.h>

#include <mutex>

namespace grove_g1
{

// Starts the sensor threads if configured, otherwise does nothing. Non-blocking.
//
// model/data are the addresses of unitree_mujoco's globals: still null at the call site, and
// reassigned whenever a model is dropped into the viewer. sim_mtx is held only to snapshot
// mjData, never across the ~32 ms raycast, which would stall physics.
void StartSensorPublisher(mjModel** model, mjData** data, std::recursive_mutex* sim_mtx);

// Stops the sensor threads, blocking until they exit. Safe when idle. One-way: nothing restarts
// the sampler, so a reload ends with sensors off (see g1_bringup's README).
//
// Call before mj_deleteModel on any path that replaces the model (the viewer's Reload and
// drag-and-drop). The sampler reads the model outside sim_mtx on purpose, so only a join makes
// freeing it safe.
void StopSensorPublisher();


}  // namespace grove_g1

#endif  // GROVE_G1_SENSOR_PUBLISHER_H_
