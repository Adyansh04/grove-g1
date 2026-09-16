# unitree_mujoco patches

Patches applied to the vendored `unitree_mujoco` source at image build time.

## Why patch the vendor at all

Sensor computation needs the **scene**: geometry, meshes, and current pose. That lives in `mjModel` /
`mjData`, which are process-local globals inside `unitree_mujoco`. Unlike `/lowstate` and
`rt/arm_sdk`, there is no DDS topic carrying it, so no companion process can reach it. A second
process could load the same model and mirror joint state, but that is a second physics instance
whose fidelity is bounded by its pose source — and the real G1 has no odometry topic at all, so that
approach dies exactly at the hardware transition this track exists to protect.

Full reasoning is kept in the maintainer's local engineering notes.

## Rules

- **All real logic goes in new files.** Vendor files get the smallest possible edit that registers
  them. This keeps the reapply surface a few lines rather than a rewrite.
- **`simulate/src/unitree_sdk2_bridge.h` is edited only by 007.** Its `run()` is a 1 kHz callback
  that writes `mj_data_->ctrl[]`, the actuator command path, so nothing in it may block. 007 is the
  one exception, because the startup hold has to replace that command until a controller drives
  the motor.
- **Nothing writes `mjData` state from the bridge thread except `ctrl`.** Other state is changed
  under `sim.mtx` or from `mjcb_control`; flipping `eq_active` mid-step exits the simulator.
- Each patch names the upstream SHA it was generated against, in its header.
- Patches apply in filename order.

## Scene hooks

- `pelvis_startup_hold`: a weld 007 releases once every motor has been driven for half a second.
- `startup_hold_<joint>`: a single-joint equality holding a spawn angle from the first physics step.
  007 makes that angle the motor's hold target and releases the equality on its first tick. The
  pinned tabletop scene uses these to spawn with the arms clear of the table.

The viewer's reset restores both kinds and nothing releases them again.

## Updating the pin

Bumping `UNITREE_MUJOCO_SHA` in `.devcontainer/Dockerfile` is a **two-part change**: the new SHA and
regenerated patches. The build runs `git apply --check` first, so a patch that no longer applies
**fails the image build** rather than silently producing a sim without sensors.

To regenerate against a new SHA:

```bash
git clone https://github.com/unitreerobotics/unitree_mujoco.git /tmp/um
cd /tmp/um && git checkout <NEW_SHA>
# apply and commit the patches before <NNN>, so the diff holds only this patch's change
# re-apply the changes by hand, then:
git diff > /path/to/workspace/patches/unitree_mujoco/<NNN>-<name>.patch
```

Never edit a patch's hunks by hand: the line counts stop matching and `git apply --check` fails.
