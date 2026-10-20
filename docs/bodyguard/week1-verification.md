# Week 1 read-only verification

Run from the repository root **inside the already running ROS 2 Jazzy development container**, in a shell with the existing ROS installation and built Grove-G1 overlay sourced. Do not start another simulator. The operator must arrange any missing stack separately; this script never starts or changes it.

```bash
source /opt/ros/jazzy/setup.bash
source workspace/install/setup.bash
bash scripts/bodyguard/verify_week1.sh
printf 'Exit status: %s\n' "$?"
```

Use the actual overlay path if this checkout is mounted elsewhere. Source the same DDS configuration as the running stack. The script prints domain and RMW settings; it does not change them or manage the ROS daemon.

Every diagnostic has a timeout (5 seconds by default, plus up to 2 seconds for forced termination). Each of nine topic rate observations lasts at most 8 seconds plus that grace period. The whole sequential run is bounded by the finite number of checks, typically several minutes; with defaults its command timeout budget is under 9 minutes. Ctrl-C interrupts the run. Exit codes: `0` all checks passed, `1` at least one failure, `2` incomplete/skipped checks or invalid timeout configuration.

```bash
BODYGUARD_CHECK_SECONDS=8 BODYGUARD_RATE_SECONDS=12 \
  bash scripts/bodyguard/verify_week1.sh
# For a namespaced controller manager:
BODYGUARD_CONTROLLER_MANAGER=/robot/controller_manager \
  bash scripts/bodyguard/verify_week1.sh
```

No added dependencies: Bash, GNU timeout, and the ROS tools/packages already installed in the development image. Missing CLI plugins are skipped where identified; missing required packages, interfaces, data, or inactive lifecycle nodes fail with likely causes. Adjust timeout values only within 1..99 seconds. No motion goals, publications, lifecycle transitions, controller switches, launches, Docker operations, or resets are issued. CLI discovery and temporary subscriptions/service clients add diagnostic participants and consume some bandwidth.

## Evidence and limits

Defaults come from `g1_sensor_relay/config/g1_sensor_relay.yaml`, `g1_description/urdf/g1_common.xacro`, the odometry configs, and `g1_navigation/launch/nav2.launch.py`. Runtime types and publishers are checked separately from received headers and measured rates. Head and chest color/depth images and CameraInfo are expected. A deliberately disabled camera or a namespace/remapping will fail these defaults; inspect the printed graph before interpreting the result. Rate observations have no minimum performance threshold and can fail with incompatible QoS or very slow streams. Header receipt does not validate image content, intrinsics, stamp freshness, or odometry accuracy.

The odometry source parameter distinguishes `ground_truth` (exact MuJoCo pelvis state, supplied through `/g1_sensor_relay/base_state`) from `fast_lio` (LiDAR-inertial estimate). Camera renders are synthetic sensor inputs. Relay `object_poses` are ground truth, not perception outputs. The script does not test detection or person tracking.

Fixed-goal readiness checks expect active AMCL/map_server and the five Nav2 managed servers, plus active odometry. Mapping-only or simulation-only sessions therefore fail navigation checks by design. An action name alone is insufficient: its type, server count, and lifecycle states are checked. No goal is sent, so planning, collision avoidance, locomotion, and goal completion remain unverified.

TF checks request latest available transforms from `map` to `odom`, `base_footprint`, `pelvis`, and all four optical frames. Successful lookups show connectivity during observation, not transform freshness, calibration, or temporal synchronization. Failures can originate anywhere along the chain. Do not automatically restart anything to resolve them.

## Offline validation

```bash
bash -n scripts/bodyguard/verify_week1.sh
python3 -m unittest discover -s scripts/bodyguard/tests -v
git diff --check
```

The offline tests use fake ROS/timeout executables and never contact DDS. Real package/API availability, sensors, rates, TF, lifecycle states, and action servers must still be checked manually with the command above in the existing container.
