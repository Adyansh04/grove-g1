# g1_sim

**SIM-ONLY.** The perception simulation track: a simplified mobile body carrying a 3D LiDAR and an
RGB-D camera in MuJoCo, run through `mujoco_ros2_control`.

This is a **second, independent simulation track**. `unitree_mujoco` (driven by `g1_bringup`) is
untouched and remains the source of truth for arm and locomotion fidelity. The two are never run at
the same time.

## This is not the G1

The body here is a deliberately non-physical mobility stand-in: a cylinder on three planar joints
(slide-x, slide-y, hinge-z). It cannot topple and is commanded directly through `ros2_control`.

Early perception work does not need bipedal dynamics to be correct, and a 29-DoF humanoid with no
balance controller would only topple or need a weld. Nothing here commands a motor on the G1 model,
so no new writer appears on any low-level control channel.

**What is real:** the sensor mount poses. They are taken from Unitree's own vendored URDF
(`g1_description`, joints `mid360_joint` and `d435_joint`), with `torso_link`'s offset from `pelvis`
folded in, and `base_link` sits at the pelvis spawn height of 0.793 m. So the sensors sit where they
sit on the robot.

## Scene

An 8 x 8 m room, walls with inner faces at +/-4.0 m, floor at z = 0, and three obstacles at known
poses. The geometry is deliberate: every sensor assertion in the integration tests is a geometric
fact of `mjcf/g1_perception_base.xml`, so the tests measure something real rather than checking that
data merely arrives.

## Sensors

| | Mount vs `base_link` | Notes |
|---|---|---|
| `livox_frame` | xyz `-0.00368 0.00003 0.472434`, rpy `pi 0.05112069 0` | **Roll is exactly pi: the Mid360 is mounted upside down on the real G1.** Replicated here, because missing it inverts every point cloud relative to hardware. |
| `camera_link` | xyz `0.05366 0.01753 0.473870`, rpy `0 0.83077672 0` | **Pitched 47.6 degrees downward.** That makes it a manipulation / near-ground camera, not a forward-looking navigation one. |

LiDAR is configured to the real Mid360 envelope: 360 degrees azimuth, -7 to +52 degrees elevation,
40 m range, 10 Hz, at 360 x 32 resolution (11,520 rays, about 115k pts/s against the real sensor's
~200k).

**What this LiDAR is not.** It is a uniform azimuth/elevation grid raycast, not the Mid360's
non-repetitive rosette scan. Point density is uniform where the real sensor's is time-varying. There
is no intensity, no per-point timestamps, no ring field, no noise, no dropout, no reflectivity
dependence, and no motion distortion. Anything whose correctness depends on the real scan pattern,
such as Livox-tuned LIO feature extraction or de-skewing, is **not** validated here. The real
driver's `CustomMsg` format is not produced either; this publishes `PointCloud2`.

The camera is `848x480 @ fovy 58`, about 87 degrees horizontal, matching the real D435i **depth**
stream. One MuJoCo camera cannot carry two intrinsics, so the colour stream is wider than a real
D435i colour stream. Matched to depth, which is what this track is for. No depth noise, no stereo
dropout on textureless surfaces, no IR projector behaviour.

## MJCF plugin API

The 3D LiDAR is a MuJoCo **engine plugin**, configured in `<extension><instance>` and referenced
from `<sensor>`. Config keys are `resolution`, `azimuth_range`, `elevation_range`, `max_range`,
`min_range`, `update_rate`, `async`.

The upstream README documents an older API (`mujoco.sensor.lidar` with `size` and `fov`) that does
**not** work at the pinned commit. `elevation_range` being an explicit min/max, rather than a
symmetric field of view, is why the asymmetric Mid360 band needs no compensating sensor pitch.

The plugin registers itself under the ament resource index key `mujoco_plugins`, which the node
scans at startup. No `MUJOCO_PLUGIN_DIR` is needed, and setting it does not work.
