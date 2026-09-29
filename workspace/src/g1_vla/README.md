# g1_vla

The learned-grasp skill. A policy engine returns chunks of joint targets; `g1_vla_server`
validates each chunk against the MoveIt planning scene and only then hands it to the trajectory
controllers MoveIt already drives, so the skill adds no command path of its own. It takes no
control authority either: the arm and hands must already be acquired. The pipeline runs end to
end, but the base GR00T checkpoint does not grasp yet; see the
[learned grasping guide](../../../docs/guides/learned-grasping.md).

```mermaid
flowchart LR
    BT["g1_bt_executor"] -- "Grasp" --> S["g1_vla_server"]
    S -- "GetActionChunk" --> E["policy engine"]
    S -- "/check_state_validity" --> MG["move_group"]
    S -- "FollowJointTrajectory" --> C["arm_trajectory_controller<br/>left/right_hand_controller"]
```

```bash
./scripts/manage.sh build g1_vla
```

## Nodes

| Node | Does |
|---|---|
| `g1_vla_server` | Serves `Grasp`. Queries the engine, validates each chunk, executes it, and measures the lift. |
| `g1_vla_mock_engine` | Stand-in engine that walks named joints toward a fixed target. No model needed. |
| `g1_vla_groot_adapter` | Engine backed by a GR00T policy server over ZMQ. |

`g1_vla_groot_adapter` is Python because it only marshals dictionaries and arrays against a
schema read at runtime, on a slow path with no timing or safety role. It speaks the policy
server's wire protocol directly, so the container never needs the model's dependencies.

`vla.launch.py` starts the server and the engine chosen by `engine:=mock|groot` (default `mock`).
`execution_mode:=trajectory|servo` (default `trajectory`) sets how a validated chunk runs:
`trajectory` sends it as a `FollowJointTrajectory` goal without waiting, so the next chunk
replaces it mid-motion; `servo` streams the arm's share as jog commands into a `servo_node` and
keeps the hands on trajectories. Validation is the same in both, and the
[learned grasping guide](../../../docs/guides/learned-grasping.md) covers the trade-off.
`g1_bringup` passes the two arguments as `vla_engine` and `vla_execution_mode`, and starts the
`servo_node` for `servo`. The launch needs `move_group` and the controllers already running.

## Interfaces

| Direction | Name | Type |
|---|---|---|
| Action server | `~/grasp` | `g1_msgs/action/Grasp` |
| Service client | `engine_service`, set by the launch to `/g1_vla_engine/get_action_chunk` | `g1_msgs/srv/GetActionChunk` |
| Service client | `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` |
| Service client | `/get_planning_scene`, `/apply_planning_scene` (exempt the grasping hand from the octomap during a goal) | `moveit_msgs/srv/GetPlanningScene`, `moveit_msgs/srv/ApplyPlanningScene` |
| Action client | `/{arm_trajectory,left_hand,right_hand}_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| Publisher and service client (servo mode) | `servo_topic`, `/servo_node/switch_command_type` | `control_msgs/msg/JointJog`, `moveit_msgs/srv/ServoCommandType` |
| Subscriber | `/objects`, `/joint_states` | `vision_msgs/msg/Detection3DArray`, `sensor_msgs/msg/JointState` |
| Service server (engines) | `~/get_action_chunk`, remapped by the launch to the name above | `g1_msgs/srv/GetActionChunk` |

The GR00T adapter also reads `/joint_states`, the image topics in `video_topics` (sensor QoS), and
TF for the wrist poses. A chunk carries absolute joint positions for any subset of the 14 arm and
14 hand joints, and its `time_from_start` must increase.

## Parameters

`config/g1_vla_server.yaml`:

| Parameter | Default | Meaning |
|---|---|---|
| `engine_timeout_s` | 10.0 | How long one chunk request may take |
| `max_start_jump_rad` | 0.15 | Largest gap allowed between the measured pose and a chunk's first waypoint |
| `max_segment_step_rad` | 0.20 | Largest move allowed between consecutive waypoints |
| `velocity_scaling` | 0.8 | Fraction of each joint's velocity limit a chunk may ask for |
| `replan_period_s` | 0.15 | Minimum time between chunks sent in trajectory mode |
| `max_rejected_chunks` | 5 | Consecutive rejections before the goal aborts |
| `timeout_s` | 90.0 | Overall goal deadline |
| `chunk_exec_timeout_s` | 10.0 | Per-controller deadline for one chunk |
| `success_lift_m` | 0.05 | Rise in the object's height that counts as a grasp |
| `object_timeout_ms` | 1000.0 | How stale an `/objects` pose may be |
| `servo_topic` | `/servo_node/delta_joint_cmds` | Where jog commands go in servo mode |
| `servo_publish_rate` | 50.0 | Jog commands per second while streaming a chunk |

All but `servo_topic` are re-read at the start of every goal, so `ros2 param set` takes effect on
the next grasp. `engine_service` and `execution_mode` come from the launch file and stay out of
the yaml: a key under the node's name there would override the launch's `/**` value.

Velocity limits come from `robot_description_planning.joint_limits`, which `vla.launch.py` loads
from `config/joint_limits.yaml`. That file raises MoveIt's arm limit from 0.8 rad/s to 1.0, since
a chunk is timed at its training rate; planned motions keep MoveIt's limits. The server logs the
resolved arm limit at startup.

`config/g1_vla_mock_engine.yaml`: `joint_names`, `target_positions`, `steps_per_chunk`,
`action_dt_s` and `step_rad`. The default target swings the right arm out to the side, away from
the bench, and `target_positions` is read per request so a test can retarget it live.

`config/g1_vla_groot_adapter.yaml`:

| Parameter | Default | Meaning |
|---|---|---|
| `server_address` | `tcp://127.0.0.1:5555` | The policy server on the host |
| `zmq_timeout_ms` | 8000 | Per request |
| `action_dt_s` | 0.125 | Minimum waypoint spacing, s |
| `max_horizon` | 8 | Waypoints kept from each prediction |
| `max_joint_speed` | 0.7 | rad/s; slower segments keep `action_dt_s`, faster ones are stretched |
| `max_image_age_s` | 5.0 | Oldest camera frame accepted |
| `action_mode` | `absolute` | `relative_to_observation` only for a server that returns deltas |
| `history_step_s` | 0.0333 | Wall time per step of frame history |
| `state_joints`, `state_eef_frames`, `action_joints`, `video_topics` | | Map the checkpoint's modality keys to joints, TF frames and image topics. The adapter logs every key the server reports and refuses to serve until every state and video key is mapped. |

## Failure behaviour

A rejected chunk never reaches a controller and cancels any trajectory still running, so the arm
stops where it is. `max_rejected_chunks` in a row aborts the goal with a message starting
`blocked:`. Every exit cancels any running trajectory and restores the hand's octomap exemption.
Moving away afterwards is the behaviour tree's job.

## Running in simulation

```bash
ros2 launch g1_bringup bringup.launch.py world:=manipulation pin_pelvis:=true \
    odometry:=ground_truth moveit:=true manipulation:=true vla:=true vla_engine:=mock \
    activate_arm:=true activate_arm_delay_s:=40.0
```

Once the log says `activation complete`, in another container shell:

```bash
ros2 action send_goal /g1_vla_server/grasp g1_msgs/action/Grasp \
    "{instruction: 'pick up the red block', object_id: red_block, arm: right}"
```

The mock engine never grasps, so that goal ends on its timeout; it exists to exercise the gate.
`./scripts/demos/learned-grasping.sh mock|groot|servo` opens the bring-up and the
`vla_grasp_in_place.xml` tree in panes. For a real policy, run `./scripts/setup-groot.sh` once,
then `./scripts/serve.sh groot` on the host, and pass `vla_engine:=groot`.

## Tests

`./scripts/manage.sh test g1_vla` skips the two simulator suites, which need `--sim`.

| Test | Needs a simulator | Covers |
|---|---|---|
| `test_chunk_utils` | No | Chunk shape, the start-jump, segment-step and velocity checks, the controller split and the servo tracking velocity. |
| `test_groot_adapter` | No | The adapter against a stub policy server: wire protocol, key mapping, and action integration. Needs no GPU. |
| `test_vla_grasp_mock` | Yes | Valid chunks reach the controllers and move the arm; a chunk aimed at a colliding pose is refused with the arm still where it started. |
| `test_vla_grasp_servo` | Yes | Validated chunks are streamed as jog commands, a blocked chunk never is, and servo halts for a collision while the arm is already moving. |
