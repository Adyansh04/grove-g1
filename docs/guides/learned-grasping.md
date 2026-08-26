# Learned grasping

A vision-language-action policy proposes joint targets, and every chunk it proposes is checked
against MoveIt's live planning scene before any of it reaches a controller. The instruction is
plain text: "pick up the red cube".

## What works and what does not

The pipeline works end to end. A real 3B model drives the arm, the gate validates and refuses,
and refusals are enforced before the arm moves rather than after.

**The base checkpoint does not actually grasp the cube.** It reaches toward it, converges to
roughly 8 cm, then drifts back without ever closing the hand. This is expected and is not an
integration fault: NVIDIA's own FAQ states that no true zero-shot cross-embodiment VLA exists, and
their smooth G1 demos are post-trained on collected demonstrations, not the base model. Getting a
real grasp needs fine-tuning on demonstrations from this robot, and there is no recorder for that
yet.

So treat this as a demonstration of the gate, not of the policy. Run it with the mock engine to
see the mechanism without a GPU.

## Without a model

The mock engine serves the same service the real one does, so the gate cannot tell them apart. No
GPU, no downloads.

```bash
./scripts/manage.sh exec
cd /root/workspace && source install/setup.bash

ros2 launch g1_bringup bringup.launch.py \
  moveit:=true manipulation:=true vla:=true vla_engine:=mock \
  world:=manipulation pin_pelvis:=true odometry:=ground_truth \
  activate_arm:=true activate_arm_delay_s:=40.0
```

```bash
ros2 launch g1_orchestration mission.launch.py tree:=vla_grasp_in_place.xml
```

`vla:=true` requires `manipulation:=true`, which in turn requires `moveit:=true`. The gate
validates against move_group's planning scene and measures its result from `/objects`, and the
object-pose source arrives with manipulation.

The `odometry:=ground_truth` and `pin_pelvis:=true` arguments are needed for the same reasons as
in the [pick and place guide](pick-and-place.md): a pinned pelvis gives a repeatable arm base, and
FAST-LIO cannot produce odom from a bench at arm's length.

## With the real model

The policy server runs on the host, outside the container. That is deliberate: the adapter speaks
the server's ZeroMQ protocol directly instead of importing its package, so the ROS image never
grows a CUDA dependency tree.

One-time setup, from the repository root on the host:

```bash
./scripts/setup-groot.sh
```

It clones Isaac-GR00T at a pinned commit, builds a virtualenv, and installs the inference
dependencies. Two things it cannot do for you:

1. The VLM backbone `nvidia/Cosmos-Reason2-2B` is a gated repository and every GR00T checkpoint
   loads it. Sign in to Hugging Face, accept its terms on the model page, then authenticate:

   ```bash
   ~/ref/Isaac-GR00T/.venv/bin/hf auth login
   ~/ref/Isaac-GR00T/.venv/bin/hf download nvidia/Cosmos-Reason2-2B config.json
   ```

2. The weights are about 11 GB and download on first run into `~/.cache/huggingface`.

Serve the policy, from this repository so the load fix applies:

```bash
~/ref/Isaac-GR00T/.venv/bin/python scripts/groot_server.py \
    --model-path nvidia/GR00T-N1.7-3B --embodiment-tag real_g1 --port 5555
```

Then bring the stack up against it, in the container:

```bash
ros2 launch g1_bringup bringup.launch.py \
  moveit:=true manipulation:=true vla:=true vla_engine:=groot \
  world:=manipulation pin_pelvis:=true odometry:=ground_truth \
  activate_arm:=true activate_arm_delay_s:=40.0
```

Compose is host-networked, so the container reaches the server at `tcp://127.0.0.1:5555` with no
configuration.

### Why the launcher exists

`scripts/groot_server.py` wraps upstream's entry point rather than replacing it. Upstream loads
the checkpoint with a bare `AutoModel.from_pretrained`, which upcasts bf16 weights to fp32 in host
memory and only casts back down afterwards: 15.2 GB peak resident, measured, against 11.1 GB with
the dtype set at load time. Four gigabytes is uninteresting on a large machine and is the whole
margin on a 30 GB one, where the kernel kills the load partway through and it reads as a CUDA
fault rather than an out-of-memory one.

Fixing it in the launcher keeps the upstream checkout a clean tag, which matters because the
alternative is a patched clone that every `git pull` silently reverts.

## How a chunk is validated

Each chunk goes through, in order: shape and finiteness, how far its first waypoint sits from the
arm's measured pose, how far consecutive waypoints are apart, the speed that spacing implies, then
a per-waypoint `/check_state_validity` call against move_group's live scene and collision matrix.

A rejected chunk never reaches a controller, and anything still running from the previous turn is
cancelled, so the arm stops where the refusal found it. After `max_rejected_chunks` refusals in a
row the goal aborts with a message beginning `blocked:`. The result message carries how many
chunks ran and how many were refused, which is the measurement this skill exists to produce.

The gate does not wait for a chunk to finish. It hands the controller a fresh plan roughly every
`replan_period_s`, and each one replaces the last mid-motion, so the arm is never standing still
waiting on inference.

## Servo mode

`vla_execution_mode:=servo` streams a validated chunk through MoveIt Servo instead of sending it
as a trajectory, which adds proximity-based slowdown while the arm is moving. It starts a
`servo_node` alongside move_group.

```bash
ros2 launch g1_bringup bringup.launch.py \
  moveit:=true manipulation:=true vla:=true vla_engine:=mock vla_execution_mode:=servo \
  world:=manipulation pin_pelvis:=true odometry:=ground_truth \
  activate_arm:=true activate_arm_delay_s:=40.0
```

Validation is identical in both modes and a refused chunk is never streamed. The difference is
what they guarantee about the path: trajectory mode executes the validated waypoints, while servo
steers toward them and can end up slightly off the checked path, with its own collision monitor
covering the difference. Prefer trajectory mode unless that reaction is what you want.

## Arguments used here

| Argument | Default | Notes |
|---|---|---|
| `vla` | `false` | Needs `manipulation:=true`. |
| `vla_engine` | `mock` | `mock` needs no model; `groot` needs the host server. |
| `vla_execution_mode` | `trajectory` | `servo` streams through MoveIt Servo. |

Tuning for the gate lives in `g1_vla/config/g1_vla_server.yaml`, and the model's key and joint
mapping in `g1_vla/config/g1_vla_groot_adapter.yaml`. Both are re-read at the start of every goal,
so `ros2 param set` takes effect on the next grasp rather than needing a restart.
