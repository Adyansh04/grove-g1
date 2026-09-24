# unitree_mujoco patches

Patches applied to the pinned `unitree_mujoco` checkout at image build time, in filename order.
The logic they register lives in `workspace/vendor/unitree_mujoco/`, which the Dockerfile copies
into `simulate/src/`.

Sensors, the Dex3 hands and the grasp weld need the live `mjModel`/`mjData`, which are
process-local to the simulator and on no DDS topic, so they have to run inside it. A second
process mirroring joint state would be a second physics instance, and the real G1 has no
odometry topic to drive one.

| Patch | Does |
|---|---|
| 001 | Registers the sensor sampler (`sensor_publisher.cc`) and stops it before a model reload. |
| 002 | Lets unitree_sdk2 read `CYCLONEDDS_URI` instead of building its own DDS config. |
| 003 | Adds the Dex3-1 hand bodies, unactuated, at the stock model's hand mass. |
| 004 | Registers the Dex3 handler (`dex3_handler.cc`). |
| 005 | Registers the simulation-only grasp weld (`grasp_weld.cc`). |
| 006 | Adds the IMU inside the Mid360. |
| 007 | Holds the robot until the control stack drives the motors, and holds scene spawn postures. |
| 008 | Gives the Dex3 palm and fingers contact geometry. |
| 009 | Registers `startup_pose.cc`, which applies spawn postures at model load. |

## Rules

- **All real logic goes in new files.** Vendor files get the smallest edit that registers them,
  so the reapply surface stays a few lines.
- **`simulate/src/unitree_sdk2_bridge.h` is edited only by 007.** Its `run()` is a 1 kHz callback
  that writes `mj_data_->ctrl[]`, the actuator command path, so nothing in it may block. 007 is
  the exception because the startup hold has to replace that command until a controller drives
  the motor.
- **Nothing writes `mjData` state from the bridge thread except `ctrl`.** Other state is changed
  under `sim.mtx` or from `mjcb_control`; flipping `eq_active` mid-step exits the simulator.
- Each patch names the upstream SHA it was generated against, in its header.

## The contact firewall (008)

The Dex3 collision capsules are `contype="2" conaffinity="2"` and every other geom in the robot
and the scenes is `1/1`. `(2 & 1) || (1 & 2)` is zero, so the hand collides with **nothing** until
a scene opts a prop in by setting bit 1. Only the two manipulation scenes set their props and
table to `3/3`; every other world keeps the contact count the walking policy was trained with.

The capsules are `group="3"`, outside the groups 0-2 that the viewer and the rendered camera
draw, so perception still sees only the meshes. `priority="1"` makes the finger friction and
solver parameters win outright instead of averaging with the object's, so no prop needs contact
tuning to be pickable.

## Scene hooks

- `pelvis_startup_hold`: a weld 007 releases once every motor has been driven for half a second.
- `startup_hold_<joint>`: a single-joint equality holding a spawn angle. 009 writes the angle
  into `qpos` at load, and 007 makes it the motor's hold target and releases the equality on its
  first tick. The manipulation scenes use these to spawn with the hands clear of the table;
  without them the right hand spawns inside a prop and throws it across the room.
- `grasp_*`: body welds, palm first, that `grasp_weld.cc` engages when the thumb and one other
  digit touch the object within `capture_radius_m` of the palm, and releases after
  `release_after_s` without that contact.

The viewer's reset restores the startup holds, and nothing releases them again.

## Updating the pin

Bumping `UNITREE_MUJOCO_SHA` in `.devcontainer/Dockerfile` is a **two-part change**: the new SHA
and regenerated patches. The build runs `git apply --check` first, so a patch that no longer
applies **fails the image build** rather than silently producing a sim without sensors.

To regenerate against a new SHA:

```bash
git clone https://github.com/unitreerobotics/unitree_mujoco.git /tmp/um
cd /tmp/um && git checkout <NEW_SHA>
# apply and commit the patches before <NNN>, so the diff holds only this patch's change
# re-apply the changes by hand, then:
git diff > /path/to/workspace/patches/unitree_mujoco/<NNN>-<name>.patch
```

Never edit a patch's hunks by hand: the line counts stop matching and `git apply --check` fails.
