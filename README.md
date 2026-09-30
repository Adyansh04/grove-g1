![Grove-G1](docs/media/banner.svg)

# Grove-G1

[![CI](https://github.com/Adyansh04/grove-g1/actions/workflows/ci.yml/badge.svg)](https://github.com/Adyansh04/grove-g1/actions/workflows/ci.yml)
[![License: BSD-3-Clause](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)

An autonomy stack for the [Unitree G1](https://www.unitree.com/g1) humanoid, built on ROS 2 Jazzy
and developed in simulation first. The simulator, `unitree_mujoco`, speaks the same DDS channels as
the robot, so the hardware interface, navigation and control code run unchanged on the real G1:
moving over is a change of DDS domain and network interface.

The earlier Humble line, where Unitree's own leg controller walks the robot and this stack only
adds the arms, lives on the
[`humble-unitree`](https://github.com/Adyansh04/grove-g1/tree/humble-unitree) branch.

## What it does

- A learned locomotion policy walks and balances the robot. It runs at 50 Hz as a `ros2_control`
  controller and drives the legs and waist over `rt/lowcmd`. One hardware component owns all 29
  body motors, and the 14 arm joints belong to an ordinary `JointTrajectoryController`, so MoveIt
  moves the arms while the policy keeps the robot up.
- SLAM Toolbox maps a building, AMCL localizes against the saved map, and Nav2 drives to a goal.
  Odometry comes from FAST-LIO2 on the Mid360 LiDAR.
- MoveIt plans for either arm or both, collision-checked against a live octomap
  from the LiDAR, and each Dex3-1 hand is a planning group with named postures. Pick and place
  are actions, and a BehaviorTree.CPP tree strings them together with navigation: drive to a bench,
  close the last half metre on the measured object, pick a ball, carry it across the building and
  drop it into a box.
- Objects are named in plain text, segmented in the head camera and placed in 3D
  with the aligned depth image. Without `perception:=true`, their poses come from the simulator.
- [canopy](https://github.com/Adyansh04/canopy) explores a building until its
  cameras have seen every room, and keeps the rooms, the objects in them and the camera coverage
  on the SLAM map. Missions then name their targets ("the dustbin in the office") instead of
  carrying coordinates.
- A vision-language-action policy (GR00T N1.7) proposes arm motion, and each
  action chunk is checked against the planning scene before it runs. The pipeline works end to end,
  but the pretrained policy does not grasp yet: that needs demonstrations recorded on this robot.

## Demos

### Navigation with Nav2

![Nav2 demo](docs/media/grove_nav2_demo.gif)

### Arm planning with MoveIt

![MoveIt demo](docs/media/grove_moveit_demo.gif)

### Pick and place

![Pick and place demo](docs/media/grove_pick_place_demo.gif)

### Exploring an apartment with canopy

https://github.com/user-attachments/assets/347d91c4-3810-4e02-85bc-f8b860215505

## Architecture

![Grove-G1 architecture](docs/media/architecture.svg)

On the robot, the simulator becomes the physical G1 and the LiDAR front end becomes
`livox_ros_driver2`. Everything above the DDS rail stays the same. Two rules shape the design,
and both hold in simulation too:

- Only the hardware component writes `rt/lowcmd`. Each joint belongs to exactly one controller,
  and a joint no controller claims is unpowered, so owning a joint is what makes it hold.
- Commanding `rt/lowcmd` means owning balance. There is no onboard controller underneath to catch
  a mistake.

## Packages

| Package | What it does |
|---|---|
| [`g1_bringup`](workspace/src/g1_bringup) | The entry point: launch files, scenes and config that compose everything below. |
| [`g1_description`](workspace/src/g1_description) | The G1 URDF and its `ros2_control` xacro wrapper. |
| [`g1_hardware_interface`](workspace/src/g1_hardware_interface) | `ros2_control` plugin that owns all 29 body motors over `rt/lowcmd`. |
| [`g1_hand_interface`](workspace/src/g1_hand_interface) | `ros2_control` plugin for one Dex3-1 hand, on the hand's own SDK channels. |
| [`g1_controllers`](workspace/src/g1_controllers) | The locomotion policy, its chained safety controller and the freeze controllers. |
| [`g1_state_estimation`](workspace/src/g1_state_estimation) | Odometry (`odom` to `base_footprint`) and the TF chain Nav2 needs. |
| [`g1_sensor_relay`](workspace/src/g1_sensor_relay) | The LiDAR, cameras, IMU and object poses sampled inside the simulator. |
| [`g1_navigation`](workspace/src/g1_navigation) | SLAM Toolbox mapping, AMCL localization and Nav2. |
| [`g1_locomotion`](workspace/src/g1_locomotion) | Walks the base into arm's reach of a measured object, and back out. |
| [`g1_moveit_config`](workspace/src/g1_moveit_config) | MoveIt: arm and hand planning groups, kinematics, the octomap. |
| [`g1_manipulation`](workspace/src/g1_manipulation) | Pick and place as actions, and the object poses behind them. |
| [`g1_perception`](workspace/src/g1_perception) | Objects named in text, measured from the camera, plus grasp generation and instruction grounding. |
| [`g1_vla`](workspace/src/g1_vla) | Learned grasping: a policy's action chunks, checked against the planning scene before they run. |
| [`g1_orchestration`](workspace/src/g1_orchestration) | The behaviour trees that turn navigation and manipulation into missions. |
| [`g1_msgs`](workspace/src/g1_msgs) | The stack's own actions, services and messages. |

The world model, [canopy](https://github.com/Adyansh04/canopy), is its own repository, checked out
as a submodule in `workspace/src/canopy`. It holds the world model node, its detector front end and
the host model servers it talks to, and it knows nothing about the G1: the G1's topics and values
live in `g1_bringup/launch/world_model.launch.py`.

## Quick start

You need Docker Engine with Compose v2, the NVIDIA driver and the NVIDIA Container Toolkit on the
host. The GUI demos need an X11 desktop session; `manage.sh start` lets the container use it.

```bash
git clone --recurse-submodules https://github.com/Adyansh04/grove-g1.git
cd grove-g1
cp .env.example .env
./scripts/manage.sh start
./scripts/manage.sh build
```

In an existing clone, `git submodule update --init` fetches the submodules, and
`git config submodule.recurse true` makes `git pull` keep them in step. Third-party code the
stack changes lives in forks on their `grove` branches: `livox_ros_driver2` and
`fast_lio_humanoid` in `workspace/src`, and the simulator, `unitree_mujoco`, in `workspace/vendor`.

Then pick a demo. Each guide explains what runs and why; each launcher opens its commands at once.

| Guide | What it covers | Launcher |
|---|---|---|
| [Navigation and arm planning](docs/guides/navigation-and-moveit.md) | Mapping, localization, Nav2 goals, and MoveIt planning against the LiDAR octomap. | `navigation-and-moveit.sh` |
| [Pick and place](docs/guides/pick-and-place.md) | The manipulation skills and the behaviour trees that sequence them with navigation. | `pick-and-place.sh` |
| [Learned grasping](docs/guides/learned-grasping.md) | A vision-language-action policy behind the planning-scene gate. Runs; does not grasp yet. | `learned-grasping.sh` |
| [Open-vocabulary perception](docs/guides/open-vocabulary-grasping.md) | Objects named in text and measured in 3D, generated grasps, and instructions turned into phrases. | `open-vocabulary-grasping.sh` |
| [Exploring and asking what is where](docs/guides/world-model.md) | canopy exploring an apartment: its rooms, its objects, and finding them by name. | `world-model.sh` |
| [Talking to the robot with NervROS](docs/guides/nervros.md) | An agent you chat with: it looks, finds and segments things, fixes the world model, runs missions you approve (pick and place included), and checks, watches and plots the robot for you. | `nervros.sh` |
| [Checking the map by hand](workspace/src/canopy/editor/doc/guide.md) | canopy's map editor: what to check in a saved world and how to fix it. | `world-model.sh editor` |

The launchers live in `scripts/demos/`. Run one without arguments to list its variants, then name
one: `./scripts/demos/pick-and-place.sh in-place`. It tears down any stack left running, opens each
command in a pane of one tilix window (or a window each in another terminal), and makes commands
that need the stack wait for it. `stop` ends the demo. A launcher that opens RViz or the
simulator's viewer first checks that the container can reach your display, and prints the `xhost`
fix if it cannot. `ros2 launch g1_bringup bringup.launch.py --show-args` lists every launch
argument with its description.

## Development

Development happens in the dev container: Docker Compose runs it, and
`.devcontainer/devcontainer.json` adds VS Code on top (`Dev Containers: Reopen in Container`).

```bash
./scripts/manage.sh start | stop | restart | recreate | logs
./scripts/manage.sh exec [command]          # a shell in /root/workspace, sourced; or one command
./scripts/manage.sh build [package...]      # colcon with the dev flags, capped to spare memory
./scripts/manage.sh test [--sim] [package...]
./scripts/manage.sh format | lint           # clang-format and ruff over our sources
```

| Setting | Value |
|---|---|
| ROS distro | Jazzy, pinned in `.env` |
| Middleware | `rmw_fastrtps_cpp`, image-wide. The Unitree SDK carries its own CycloneDDS, pinned to loopback, and ROS must not load a second one. |
| `ROS_DOMAIN_ID` | 1 |
| C++ | C++20 on GCC 13.3 |
| Paths | `/root/workspace` (this repo's `workspace/`), `/root/data` (shared data) |

The container runs privileged, on the host network, with `/dev` mounted: DDS discovery between the
simulator and the ROS graph runs over loopback, and device access has to work. Project
dependencies go in `.devcontainer/Dockerfile`, followed by `./scripts/manage.sh recreate`.

To point the container at a real G1, set three variables in `.env`; no rebuild is needed.
`GROVE_G1_CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.hardware.xml` selects the hardware
profile, `GROVE_G1_ROBOT_NIC` names the network interface that reaches the robot, and
`GROVE_G1_ROS_DOMAIN_ID` sets its domain. They carry a prefix because the base image's
`/etc/profile.d/10-ros-env.sh` overwrites the plain names. `sim.launch.py` refuses to start unless
`CYCLONEDDS_URI` pins loopback, so the simulator cannot come up pointed at a robot.

## Tests

```bash
./scripts/manage.sh test
./scripts/manage.sh test --sim [package...]
```

The first runs what CI runs: every test of our packages and canopy's that needs no simulator, lint
included. `--sim` runs only the suites that start a simulator, one package at a time, after
`./scripts/clean-stack.sh` has cleared any stack left running. Several stacks on one DDS graph are
the usual cause of failures that pass on a clean rerun. Each package README says which of its tests
need the simulator.

CI builds the workspace and runs the non-simulator tests on every pull request and every push to
`main`, in the image from `.github/ci.Dockerfile`. The simulator suites stay local: on a shared
runner they measure the runner rather than the stack, so run them before merging anything they
cover.

## Repository layout

```
.devcontainer/     the dev image
docs/              one guide per demo, and the README's media
scripts/           the container, stack teardown, host model servers and demo launchers
servers/           grove-g1's host-side model servers
workspace/src/     ROS 2 packages, and the canopy, Livox and FAST-LIO submodules
workspace/vendor/  the simulator, our unitree_mujoco fork
```

[scripts/README.md](scripts/README.md) lists what each script does.
