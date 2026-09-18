#ifndef GROVE_G1_STARTUP_POSE_H
#define GROVE_G1_STARTUP_POSE_H

#include <mujoco/mujoco.h>

namespace grove_g1
{

/**
 * @brief Spawns the joints a scene declares a `startup_hold_<joint>` equality for at that angle.
 *
 * Called once per model load, on the physics thread, before the first step. Without it the
 * equalities have to drag the arms out of the pose the URDF zero leaves them in, which since the
 * fingers gained contact geometry means dragging a hand out through whatever it spawned inside.
 * Applying the angle first leaves the equality satisfied at t=0: nothing swings, and the
 * constraint still holds the pose against gravity until patch 007 hands it to the motors.
 */
void ApplyStartupPose(const mjModel* model, mjData* data);

}  // namespace grove_g1

#endif  // GROVE_G1_STARTUP_POSE_H
