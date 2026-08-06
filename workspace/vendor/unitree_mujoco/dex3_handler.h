#ifndef GROVE_G1_DEX3_HANDLER_H_
#define GROVE_G1_DEX3_HANDLER_H_

// Answers the Dex3-1 hand's DDS contract inside unitree_mujoco, so the same ros2_control
// component (g1_hand_interface) drives the fingers in sim and on the robot.
//
// Lives outside the vendored sources so the patch against them stays a few lines; see
// workspace/patches/unitree_mujoco/README.md.
//
// Why this cannot be an ordinary ROS node: the fingers are driven by writing mjData, which
// is process-local. Same reason sensor_publisher lives here.
//
// Why it does not go through the SDK bridge: that bridge sizes itself from mj_model_->nu and
// indexes a fixed 35-slot LowCmd, so the 14 finger joints deliberately have no actuators and
// nu stays 29. The fingers are driven through qfrc_applied instead, which is also the honest
// model of the hardware -- the Dex3 is a separate device on its own topics, not motors 29-42
// of the body (docs/CONTROL_MODES.md).
//
// A no-op on models without hands (the 23-DoF G1, the perception sandbox): the joint lookup
// fails, it says so once, and nothing starts.

#include <mujoco/mujoco.h>

namespace grove_g1
{

// Starts the hand thread. Non-blocking.
//
// MUST be called after unitree_sdk2's ChannelFactory has been initialised -- it opens DDS
// channels immediately -- which is why the call site is the bridge thread rather than main().
//
// model/data are the addresses of unitree_mujoco's globals, not their values: a model dropped
// into the viewer reassigns both.
void StartDex3Handler(mjModel** model, mjData** data);

// Stops the hand thread and BLOCKS until it has terminated. Zeroes the torques it applied on
// the way out, since qfrc_applied persists across steps and a stale finger torque would keep
// being integrated forever. Safe to call when nothing is running.
//
// Must be called before mj_deleteModel on any path that replaces the model: the joint
// addresses are resolved once and cached, so they outlive the model they came from. One-way,
// like StopSensorPublisher -- a reload deliberately ends with the hands inert rather than
// driving a model they were not resolved against.
void StopDex3Handler();

}  // namespace grove_g1

#endif  // GROVE_G1_DEX3_HANDLER_H_
