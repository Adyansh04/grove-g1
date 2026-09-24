#ifndef GROVE_G1_STARTUP_POSE_H
#define GROVE_G1_STARTUP_POSE_H

#include <mujoco/mujoco.h>

namespace grove_g1
{

/**
 * @brief Spawns the joints a scene declares a `startup_hold_<joint>` equality for at that angle.
 *
 * Called once per model load, on the physics thread, before the first step, so the equality is
 * satisfied at t=0 instead of dragging a hand out through whatever it spawned inside. The
 * equality still holds the pose until patch 007 hands it to the motors.
 */
void ApplyStartupPose(const mjModel* model, mjData* data);

}  // namespace grove_g1

#endif  // GROVE_G1_STARTUP_POSE_H
