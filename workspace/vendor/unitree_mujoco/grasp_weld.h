#ifndef GROVE_G1_GRASP_WELD_H_
#define GROVE_G1_GRASP_WELD_H_

// Makes a closed Dex3 hand actually hold a scene object in MuJoCo.
//
// SIMULATION ONLY, and a stand-in rather than a model of anything. The finger geoms are
// deliberately contact-free (patch 003: grasping is modelled by attaching the object to the
// arm in MoveIt's planning scene, and tuned multi-finger contact is its own project), so
// without this the object simply stays on the table while the planner believes it is held.
// This closes that gap by welding the object to the palm once the hand has measurably closed
// around it.
//
// On hardware the mechanism is friction between real finger pads and a real object, and this
// file never runs. Its thresholds therefore do NOT transfer: capture_radius_m, close_fraction
// and release_fraction are properties of this weld, not of the Dex3's grip, and a grasp that
// holds here says nothing about whether the real hand holds it.
// hardware re-validation list, alongside the gait deadband and the hand-mass scaling.
//
// The trigger is measured, not scripted: it reads the finger joints' actual positions and the
// object's actual distance from the palm, so a plan that closes the hand in the wrong place
// picks up nothing, exactly as it should.
//
// Lives outside the vendored sources so the patch against them stays a few lines; see
// workspace/patches/unitree_mujoco/README.md.

#include <mujoco/mujoco.h>

#include <mutex>

namespace grove_g1
{

// Starts the weld thread if the config enables it, otherwise returns having done nothing.
// Non-blocking.
//
// Managed welds are discovered from the model rather than configured: every equality named
// `grasp_*` that welds two bodies is taken to be one, with body1 the palm and body2 the
// object. The scene is therefore the single source of truth for what is graspable, and a
// model with no such welds (the flat and perception worlds) costs nothing.
//
// model/data are the addresses of unitree_mujoco's globals, not their values: a model dropped
// into the viewer reassigns both.
//
// sim_mtx is unitree_mujoco's own simulation mutex. Unlike the sensor sampler, this holds it
// across its whole tick: the work is a few dozen doubles, and the alternative -- snapshotting
// and then writing back -- would let physics move the hand between the measurement and the
// weld it justifies.
void StartGraspWeld(mjModel** model, mjData** data, std::recursive_mutex* sim_mtx);

// Stops the weld thread and BLOCKS until it has terminated. Releases anything it was holding
// on the way out, so a torn-down simulator does not leave an object glued to a hand. Safe to
// call when nothing is running.
//
// Must be called before mj_deleteModel on any path that replaces the model: constraint and
// body ids are resolved once and cached, so they outlive the model they came from.
void StopGraspWeld();

}  // namespace grove_g1

#endif  // GROVE_G1_GRASP_WELD_H_
