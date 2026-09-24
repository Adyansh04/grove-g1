# Learned grasping

A vision-language-action policy proposes joint targets, and every chunk it proposes is checked
against MoveIt's live planning scene before any of it reaches a controller. The instruction is
plain text: "pick up the red block".

## What works and what does not

The pipeline works end to end: a real 3B model drives the arm, the gate validates every chunk, and
a refused chunk never moves the arm.

**The base checkpoint does not grasp.** It reaches toward the block and drifts back without
closing the hand. That is expected of a base model, not an integration fault: a real grasp needs
post-training on demonstrations from this robot, and there is no recorder for them yet. Treat this
as a demonstration of the gate, not of the policy.

## Without a model

The mock engine serves the same service as the real one, so the gate cannot tell them apart. No
GPU, no downloads. In a container shell, opened with `./scripts/manage.sh exec`:

```bash
cd /root/workspace && source install/setup.bash

ros2 launch g1_bringup bringup.launch.py \
  moveit:=true manipulation:=true vla:=true vla_engine:=mock \
  world:=manipulation pin_pelvis:=true odometry:=ground_truth \
  activate_arm:=true activate_arm_delay_s:=40.0
```

```bash
ros2 launch g1_orchestration mission.launch.py tree:=vla_grasp_in_place.xml
```

The mock walks the right arm out to the side, away from the bench. It never grasps, so each of the
tree's two attempts ends on the 90 s goal timeout and the tree fails. What it shows is that only
validated chunks move the arm.

`vla:=true` requires `manipulation:=true`, which requires `moveit:=true`: the gate validates
against move_group's planning scene and measures the lift on `/objects`, which comes with
manipulation. `pin_pelvis:=true` and `odometry:=ground_truth` are needed for the same reasons as
in the [pick and place guide](pick-and-place.md).

## With the real model

The policy server runs on the host, outside the container. The adapter speaks its ZeroMQ protocol
directly, so the ROS image never needs the model's CUDA dependencies.

One-time setup, from the repository root on the host. It needs `uv`:

```bash
./scripts/setup-groot.sh
```

It clones Isaac-GR00T at a pinned commit into `~/ref/Isaac-GR00T` and installs the inference
dependencies in a virtualenv. Two things it cannot do for you:

1. Every GR00T checkpoint loads the gated `nvidia/Cosmos-Reason2-2B` backbone. Accept its terms on
   the Hugging Face model page, then authenticate:

   ```bash
   ~/ref/Isaac-GR00T/.venv/bin/hf auth login
   ~/ref/Isaac-GR00T/.venv/bin/hf download nvidia/Cosmos-Reason2-2B config.json
   ```

2. The weights, about 11 GB, download into `~/.cache/huggingface` on the first run.

Serve the policy from this repository:

```bash
~/ref/Isaac-GR00T/.venv/bin/python scripts/groot_server.py \
    --model-path nvidia/GR00T-N1.7-3B --embodiment-tag real_g1 --port 5555
```

`groot_server.py` wraps upstream's entry point to load the checkpoint in bf16. Upstream upcasts to
fp32 in host memory first, about 4 GB more at peak, which on a 30 GB machine gets the load
OOM-killed with an error that reads like a CUDA fault.

Then bring the stack up against it, in the container:

```bash
ros2 launch g1_bringup bringup.launch.py \
  moveit:=true manipulation:=true vla:=true vla_engine:=groot \
  world:=manipulation pin_pelvis:=true odometry:=ground_truth \
  activate_arm:=true activate_arm_delay_s:=40.0
```

Compose is host-networked, so the adapter reaches the server at `tcp://127.0.0.1:5555` with no
configuration. It logs the modality keys the checkpoint reports and refuses to serve until every
state and video key is mapped in `g1_vla/config/g1_vla_groot_adapter.yaml`.

## How a chunk is validated

Each chunk is checked in order: shape and finiteness, how far its first waypoint is from the arm's
measured pose, the step between consecutive waypoints, the speed that spacing implies, then a
`/check_state_validity` call per waypoint against move_group's live scene.

A rejected chunk never reaches a controller, and the trajectory still running is cancelled, so the
arm stops where the refusal found it; the server then asks the engine again. After
`max_rejected_chunks` refusals in a row the goal aborts with a message starting `blocked:`.
Feedback counts the chunks executed and the chunks refused.

In trajectory mode the server does not wait for a chunk to finish. It sends a fresh one at most
every `replan_period_s`, and each replaces the last mid-motion, so the arm never stands still
waiting on inference.

## Servo mode

`vla_execution_mode:=servo` streams the arm's share of each validated chunk through MoveIt Servo
instead of sending it as a trajectory, which adds proximity slowdown while the arm moves. The hands
stay on trajectories. It starts a `servo_node` beside move_group.

```bash
ros2 launch g1_bringup bringup.launch.py \
  moveit:=true manipulation:=true vla:=true vla_engine:=mock vla_execution_mode:=servo \
  world:=manipulation pin_pelvis:=true odometry:=ground_truth \
  activate_arm:=true activate_arm_delay_s:=40.0
```

Validation is the same in both modes, and a refused chunk is never streamed. Servo tracks velocity,
not position, so the arm can end up slightly off the validated path, with Servo's own collision
monitor halting it on proximity. Prefer trajectory mode unless that reaction is what you want.

## Arguments used here

| Argument | Default | Notes |
|---|---|---|
| `vla` | `false` | Needs `manipulation:=true`. |
| `vla_engine` | `mock` | `mock` needs no model; `groot` needs the host server. |
| `vla_execution_mode` | `trajectory` | `servo` streams through MoveIt Servo. |

Gate tuning lives in `g1_vla/config/g1_vla_server.yaml`. Every key but `servo_topic` is re-read at
the start of each goal, so `ros2 param set` takes effect on the next grasp. The adapter's key and
joint mapping is read once at startup; change it and restart.
