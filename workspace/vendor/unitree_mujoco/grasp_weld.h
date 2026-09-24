#ifndef GROVE_G1_GRASP_WELD_H_
#define GROVE_G1_GRASP_WELD_H_

// Makes a closed Dex3 hand hold a scene object in MuJoCo. SIMULATION ONLY: a stand-in for
// friction, since a sphere gripped from above slides out of the finger capsules (patch 008).
//
// The weld engages when the thumb and one other digit touch the object within capture_radius_m
// of the palm, and releases after release_after_s without that contact. It reads the contacts
// MuJoCo solved, so a hand closed in the wrong place picks up nothing. Both thresholds belong to
// this weld, not to the real hand, and are on the hardware re-validation list.
//
// Lives outside the vendored sources so the patch against them stays a few lines; see
// workspace/patches/unitree_mujoco/README.md.

#include <mujoco/mujoco.h>

#include <mutex>

namespace grove_g1
{

// Starts the weld thread if the config enables it. Non-blocking.
//
// Every equality named `grasp_*` that welds two bodies is managed, body1 the palm and body2 the
// object, so the scene alone declares what is graspable. model/data are the addresses of
// unitree_mujoco's globals, which a model dropped into the viewer reassigns. sim_mtx is held for
// each whole tick, so physics cannot move the hand between the contact check and the weld.
void StartGraspWeld(mjModel** model, mjData** data, std::recursive_mutex* sim_mtx);

// Stops the weld thread, blocking until it exits, and releases anything held. Safe when idle.
// Call before mj_deleteModel on any path that replaces the model: the cached ids outlive it.
void StopGraspWeld();

}  // namespace grove_g1

#endif  // GROVE_G1_GRASP_WELD_H_
