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
- **`simulate/src/unitree_sdk2_bridge.h` is off limits.** Its `run()` is a 1 kHz callback that writes
  `mj_data_->ctrl[]`, the actuator command path. Blocking it stalls robot control directly.
- Each patch names the upstream SHA it was generated against, in its header.
- Patches apply in filename order.

## Updating the pin

Bumping `UNITREE_MUJOCO_SHA` in `.devcontainer/Dockerfile` is a **two-part change**: the new SHA and
regenerated patches. The build runs `git apply --check` first, so a patch that no longer applies
**fails the image build** rather than silently producing a sim without sensors.

To regenerate against a new SHA:

```bash
git clone https://github.com/unitreerobotics/unitree_mujoco.git /tmp/um
cd /tmp/um && git checkout <NEW_SHA>
# re-apply the changes by hand, then:
git diff > /path/to/workspace/patches/unitree_mujoco/<NNN>-<name>.patch
```
