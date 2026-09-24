#ifndef GROVE_G1_DEX3_HANDLER_H_
#define GROVE_G1_DEX3_HANDLER_H_

// Answers the Dex3-1 hand's DDS contract inside unitree_mujoco, so the same ros2_control
// component (g1_hand_interface) drives the fingers in sim and on the robot. It has to live in
// this process because the fingers are driven by writing mjData.
//
// Not through the SDK bridge: that sizes itself from nu and indexes a fixed 35-slot LowCmd, so
// the 14 finger joints have no actuators and are driven through qfrc_applied instead. That also
// matches the hardware, where the Dex3 is a separate device on its own topics.
//
// A no-op on models without hands: the joint lookup fails, it says so once, and nothing starts.
//
// Lives outside the vendored sources so the patch against them stays a few lines; see
// workspace/patches/unitree_mujoco/README.md.

#include <mujoco/mujoco.h>

namespace grove_g1
{

// Starts the hand thread. Non-blocking.
//
// Must run after unitree_sdk2's ChannelFactory is initialised, since it opens DDS channels at
// once; hence the bridge thread as call site. model/data are the addresses of unitree_mujoco's
// globals, which a model dropped into the viewer reassigns.
void StartDex3Handler(mjModel** model, mjData** data);

// Stops the hand thread, blocking until it exits, and zeroes its torques, since qfrc_applied
// persists across steps. Safe when idle.
//
// Call before mj_deleteModel on any path that replaces the model: the cached joint addresses
// outlive it. One-way, like StopSensorPublisher, so a reload leaves the hands inert.
void StopDex3Handler();

}  // namespace grove_g1

#endif  // GROVE_G1_DEX3_HANDLER_H_
