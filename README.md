![Grove-G1](docs/media/banner.svg)

# Grove-G1

[![CI](https://github.com/Adyansh04/grove-g1/actions/workflows/ci.yml/badge.svg)](https://github.com/Adyansh04/grove-g1/actions/workflows/ci.yml)
[![License: BSD-3-Clause](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)

> The earlier Humble line, where Unitree's own internal leg policy walks the robot and this stack
> only does arm manipulation on top, is kept on
> [`humble-unitree`](https://github.com/Adyansh04/grove-g1/tree/humble-unitree).

An autonomy stack for the [Unitree G1](https://www.unitree.com/g1) humanoid, built on ROS 2 Jazzy
and developed simulation-first against `unitree_mujoco`.

The simulator speaks the same DDS channels as the real robot, so the hardware interface, the
navigation stack and the control-authority logic all carry over to hardware without code changes.
Moving to the physical G1 is a domain-ID and interface change, not a rewrite.

## What it does today

The robot maps a facility with SLAM Toolbox, localizes against the saved map, and drives itself to
a goal pose under Nav2. Balance is ours: a learned locomotion policy runs at 50 Hz inside a
`ros2_control` controller and commands all 29 body motors over `rt/lowcmd`, with no onboard
controller underneath. Arm trajectories are an ordinary `JointTrajectoryController` claiming the
14 arm joints on the same component, so MoveIt executes while the policy keeps the robot up.

MoveIt plans for either arm or both together, collision-checked against a live octomap built from
the LiDAR, and each Dex3-1 hand is its own planning group with `open` and `closed` postures.

On top of that, pick and place are served as actions, and a BehaviorTree.CPP behaviour tree
sequences them with navigation into a mission that runs end to end in the facility world: drive
to a workbench, walk the last half metre under closed-loop control, pick a cube up, carry it
across the building, and put it down on a bench. Object poses come from a source that refuses to
run on hardware, because there is no object-detection pipeline yet; a real one replaces it
without the skills changing.

Nav2 parks within 0.5 m of a goal and the arm's usable window is about 0.2 m wide, so a base
approach skill closes the gap against the measured object rather than against the map. The tree
is editable in Groot2 against a generated node palette.

Learned manipulation for unstructured scenes is the next milestone and is not built yet.

## Nav2 Demo

![Nav2 Demo](docs/media/grove_nav2_demo.gif)

## MoveIt Demo

![MoveIt Demo](docs/media/grove_moveit_demo.gif)

## Pick & Place Demo

![Pick & Place Demo](docs/media/grove_pick_place_demo.gif)

## Architecture

![Grove-G1 architecture](docs/media/architecture.svg)

On hardware the simulation card becomes the physical G1 and the LiDAR front end becomes
`livox_ros_driver2`. Everything above the DDS rail is unchanged.

Two rules shape the design, and both apply in simulation so the habits transfer:

- Only one publisher ever commands `rt/lowcmd`. Control-mode ownership is explicit, and the
  hardware component leaves any joint no controller claims unpowered, so "who owns this joint" is
  also "does this joint hold".
- Commanding `rt/lowcmd` means owning balance. There is no onboard controller left underneath to
  catch a mistake.

## Packages

| Package | What it does |
|---|---|
| [`g1_bringup`](workspace/src/g1_bringup) | The entry point. Launch files, scenes and config that compose everything below. |
| [`g1_description`](workspace/src/g1_description) | Vendored G1 URDF plus the `ros2_control` xacro wrapper. |
| [`g1_hand_interface`](workspace/src/g1_hand_interface) | `ros2_control` plugin for one Dex3-1 hand, over the hand's own SDK channels. |
| [`g1_controllers`](workspace/src/g1_controllers) | The locomotion policy, its chained safety controller and the freeze controllers. |
| [`g1_hardware_interface`](workspace/src/g1_hardware_interface) | `ros2_control` plugin owning all 29 body motors over `rt/lowcmd`. |
| [`g1_locomotion`](workspace/src/g1_locomotion) | Walks the base into arm's reach of a measured object, and backs it out again. |
| [`g1_manipulation`](workspace/src/g1_manipulation) | Pick and place as actions, and the object-pose source behind them. |
| [`g1_moveit_config`](workspace/src/g1_moveit_config) | MoveIt config: arm and hand planning groups, kinematics, the octomap. |
| [`g1_msgs`](workspace/src/g1_msgs) | The mission's own actions: pick, place, approach, retreat, arm posture. |
| [`g1_navigation`](workspace/src/g1_navigation) | SLAM Toolbox mapping, AMCL localization and Nav2. |
| [`g1_orchestration`](workspace/src/g1_orchestration) | The behaviour tree that sequences navigation and manipulation into a mission. |
| [`g1_sensor_relay`](workspace/src/g1_sensor_relay) | Publishes LiDAR and depth frames sampled inside the simulator. |
| [`g1_state_estimation`](workspace/src/g1_state_estimation) | Publishes `odom` to `base_footprint` and the TF chain Nav2 needs. |

## Quick start

The ROS build and runtime commands run inside the dev container, which pins the ROS 2 Jazzy and
Ubuntu 24.04 toolchain the stack is built and tested against.

### Prerequisites

Install Docker Engine with Docker Compose v2, the NVIDIA driver and the NVIDIA Container Toolkit
on the host. The simulator uses the GPU exposed by `docker-compose.yml`. For the GUI modes, run
from an X11 desktop session; `manage.sh start` grants the container local X11 access.

Install [`vcstool`](https://github.com/dirk-thomas/vcstool) on the host as well, because the
import script uses its `vcs` command before the development container exists:

```bash
sudo apt install python3-vcstool
```

### Start the development container

From the repository root, on the host:

```bash
cp .env.example .env
./scripts/import-externals.sh
./scripts/manage.sh start
./scripts/manage.sh exec
```

`import-externals.sh` pulls the third-party packages listed in `workspace.repos` into
`workspace/src` and puts the two that ship a non-standard layout into a buildable one. Run it
again whenever `workspace.repos` changes.

### Build and run the stack

Inside the container:

```bash
cd /root/workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

Then bring up the robot. One command covers every mode:

```bash
# Simulator only, with the MuJoCo viewer
ros2 launch g1_bringup bringup.launch.py headless:=false

# Build a map, with RViz but without the MuJoCo viewer
ros2 launch g1_bringup bringup.launch.py mode:=mapping rviz:=true headless:=true

# Localize against the committed map and navigate, with RViz but without the MuJoCo viewer
ros2 launch g1_bringup bringup.launch.py mode:=localization nav:=true rviz:=true headless:=true

# Plan for the arms and hands, with the LiDAR octomap in the planning scene
ros2 launch g1_bringup bringup.launch.py moveit:=true sensors:=true rviz:=true headless:=true

# Pick and place skills, on the small test world where the object is already within reach
ros2 launch g1_bringup bringup.launch.py \
  moveit:=true manipulation:=true pin_pelvis:=true world:=manipulation \
  activate_arm:=true activate_arm_delay_s:=40.0 headless:=true

# Everything at once, including the MuJoCo viewer and RViz
ros2 launch g1_bringup bringup.launch.py \
  mode:=localization nav:=true moveit:=true manipulation:=true rviz:=true \
  activate_arm:=true activate_arm_delay_s:=40.0 headless:=false
```

`headless:=true` is the default and suits any run without a display. Set `headless:=false` only
when a local display is available to open the MuJoCo viewer, and do not use the viewer's Reload
button while sensors are running.

After launching a mode, confirm the ROS graph is up from a second container shell:

```bash
ros2 topic list -t
```

Run a mission. The tree drives navigation and manipulation; nothing else needs starting, and
Groot2 on the host can watch it tick at `localhost:1667`:

```bash
ros2 launch g1_orchestration mission.launch.py tree:=pick_and_place_in_place.xml
```

The full navigate-pick-carry-place mission needs the facility world and a map:

```bash
ros2 launch g1_bringup bringup.launch.py \
  mode:=localization nav:=true moveit:=true manipulation:=true world:=navigation \
  rviz:=true activate_arm:=true activate_arm_delay_s:=55.0
```

```bash
ros2 launch g1_orchestration mission.launch.py tree:=pick_and_place.xml
```

With `rviz:=true` and both MoveIt and Nav2 running, this opens two RViz windows: the MoveIt one
for the arm and a second on the navigation config for the map and costmaps.

Send it somewhere:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: -2.5}, orientation: {w: 1.0}}}}"
```

Moving the arms or hands needs an explicit acquire step first, and a matching release. It takes
the arm and both hands together; a hand that is absent or unpowered logs and leaves the arm
usable.

```bash
ros2 launch g1_bringup activate_arm.launch.py
# plan from RViz's MotionPlanning panel, or send FollowJointTrajectory goals to
# arm_trajectory_controller, left_hand_controller or right_hand_controller
ros2 launch g1_bringup deactivate_arm.launch.py
```

## Development environment

Docker Compose is the runtime source of truth; `.devcontainer/devcontainer.json` is the VS Code
layer on top. In VS Code, use `Dev Containers: Reopen in Container`.

| Setting | Value |
|---|---|
| ROS distro | Jazzy, pinned. |
| Middleware | `rmw_fastrtps_cpp` image-wide. The SDK carries its own CycloneDDS, pinned to loopback, and ROS must not load a second one. |
| `ROS_DOMAIN_ID` | 1 |
| Robot override | `GROVE_G1_ROS_DOMAIN_ID`, `GROVE_G1_CYCLONEDDS_URI`, `GROVE_G1_ROBOT_NIC` |
| C++ standard | C++20 on GCC 13.3 |
| Workspace | `/root/workspace` |
| Shared data | `/root/data` |

The container runs `privileged` with `network_mode: host` and a `/dev` bind mount. That is
deliberate for local robotics development: DDS discovery between the bare-DDS simulator and the
ROS graph happens over loopback, and device access has to work.

Pointing the container at a real G1 is three environment variables, not an image rebuild:
`GROVE_G1_CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.hardware.xml` (baked in beside the
loopback one, differing only in the interface), `GROVE_G1_ROBOT_NIC` for the NIC that reaches
the robot, and `GROVE_G1_ROS_DOMAIN_ID` for its domain. They are prefixed because the base
image's own `/etc/profile.d/10-ros-env.sh` rewrites the unprefixed names. `sim.launch.py`
refuses to start unless `CYCLONEDDS_URI` names a profile that pins `lo`, so the simulator
cannot be brought up pointing at a robot.

Lifecycle:

```bash
./scripts/manage.sh start | stop | restart | recreate | logs | exec
```

Use `exec-as-me` instead of `exec` for anything that rewrites source files in place, such as
`clang-tidy --fix` or `clang-format -i`. It runs as your host user, so the files do not come back
owned by root.

Project dependencies belong in `.devcontainer/Dockerfile`, followed by
`./scripts/manage.sh recreate`. Do not install into a running container and forget about it.

## Tests

```bash
colcon test --packages-select g1_description g1_locomotion g1_navigation
colcon test-result --all
```

Leftover nodes from a previous run are the most productive source of phantom failures here:
several copies of the stack on one DDS graph look like bugs everywhere except where they are.
Clear them first, and note it verifies the graph rather than the process table:

```bash
./scripts/clean-stack.sh
```

Suites that launch a simulator are timing-sensitive and serialize on a shared ctest resource lock.
Run them **one package at a time**, and check nothing is left over from a previous run
(`pgrep -x unitree_mujoco`) before trusting a result: a stray simulator is the usual explanation
for a batch of failures that all pass on a clean rerun. Each package README says which of its
tests need a simulator.

Those suites carry the ctest label `simulator`, so the rest can be run on their own:

```bash
colcon test --ctest-args -LE simulator   # everything that needs no simulator
colcon test --ctest-args -L  simulator   # only the simulator suites
```

## Continuous integration

Every pull request, and every push to `main`, builds the workspace and runs the tests that need
no simulator, in the image built by `.github/ci.Dockerfile`. Lint runs as part of `colcon test`,
not separately.

The simulator suites are excluded: they are CPU-time-sensitive and measure a shared runner
rather than the stack. Run them locally before merging anything that touches locomotion,
navigation or the sensor path.

A per-package C++ coverage table is printed to each run's summary. It covers only the tests CI
runs, so the node and launch layer reads low there by construction. It is a signal on the pure
logic, not a figure for the repository, which is why there is no badge for it.

## Repository layout

```
.devcontainer/     derived dev image
workspace/src/     ROS 2 packages
workspace/patches/ patches applied to vendored sources at image build
workspace/vendor/  our source compiled into the vendored simulator
scripts/           container lifecycle, and stack teardown
```
