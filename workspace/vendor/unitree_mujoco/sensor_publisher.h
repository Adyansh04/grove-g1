#ifndef GROVE_G1_SENSOR_PUBLISHER_H_
#define GROVE_G1_SENSOR_PUBLISHER_H_

// Sensor sampling for the converged perception track. Lives outside the vendored
// unitree_mujoco sources so the patch against them stays a few lines; see
// workspace/patches/unitree_mujoco/README.md.
//
// Deliberately links NO ROS and NO DDS. unitree_sdk2 already owns the CycloneDDS domain in
// this process, and rmw_cyclonedds creates its domain unconditionally, so the two cannot
// coexist here. Sampled frames go out over a local socket to g1_sensor_relay, which is an
// ordinary ROS node and does the publishing.
//
// Off unless GROVE_G1_SENSOR_CONFIG names a config file. With it unset the patched binary
// behaves exactly like the stock one: no thread, no socket, no cost.

#include <mujoco/mujoco.h>

#include <mutex>

namespace grove_g1
{

// Starts the sensor thread if configured, otherwise returns having done nothing. Non-blocking.
//
// model/data are the addresses of unitree_mujoco's globals, not their values: at the call
// site the model has not loaded yet and both are still null, and they are reassigned again
// whenever a model is dropped into the viewer. The thread waits for them the same way the
// SDK bridge thread does.
//
// sim_mtx is unitree_mujoco's own simulation mutex. It is held only long enough to snapshot
// mjData, never across the raycast: a full sweep costs ~32 ms against the G1 scene, and
// holding the lock for that would stall physics exactly as badly as running inline.
void StartSensorPublisher(mjModel** model, mjData** data, std::recursive_mutex* sim_mtx);


}  // namespace grove_g1

#endif  // GROVE_G1_SENSOR_PUBLISHER_H_
