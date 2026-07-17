# ROS 2 Compose-First Development Template

This template is a Compose-first ROS 2 development environment with VS Code Dev Containers on top.

- Docker Compose is the runtime source of truth.
- `.devcontainer/devcontainer.json` is the VS Code experience layer.
- Privileged/host-style access is intentionally enabled for local robotics development.

## What This Gives You

- Stable in-container paths:
  - Workspace: `/root/workspace`
  - Shared data: `/root/data`
- Small custom Dockerfile layer for project-specific dependencies.
- GPU + GUI support (NVIDIA + X11).
- Root-level, single-source tooling config for clangd, clang-format, and Ruff.

## Prerequisites

- Docker Engine and Docker Compose
- (Optional for GPU) NVIDIA Container Toolkit
- VS Code with Dev Containers extension

## Single-Point Runtime Config (`.env`)

Copy once, then edit only `.env` for distro/image/container naming knobs:

```bash
cp .env.example .env
```

Main variables:

- `ROS_DISTRO` (for example: `jazzy`, `humble`)
- `BASE_IMAGE_REPO`
- `BASE_IMAGE_TAG_SUFFIX`
- `DEV_IMAGE_REPO`
- `CONTAINER_PREFIX`
- `COMPOSE_PROJECT_NAME`

## Source Repositories (`.repos`)

This template keeps real ROS packages in separate repos and assembles them into `workspace/src` using a manifest file.

1) Edit `workspace.repos` with your repo URLs and branches.
2) Import sources (run on host or inside the container):

```bash
vcs import workspace/src < workspace.repos
```

To update to the latest branch tips later:

```bash
cd workspace/src
vcs pull
```

Commits happen inside each repo under `workspace/src`. The template repo only tracks the manifest.

If `vcs` is missing, install once:

```bash
sudo apt-get install -y python3-vcstool
```

## Daily Workflow (Recommended)

### 1) Start the environment

```bash
./scripts/manage.sh start
```

After startup, the script prints `docker compose ps` so the resolved container name is immediately visible.

### 2) Open in VS Code container

1. Open this repository in VS Code.
2. Run `Dev Containers: Reopen in Container`.
3. Work inside `/root/workspace`.

This is the primary flow for consistent extensions and settings.

Fallback: if the service is already running and you prefer attach mode, use `Dev Containers: Attach to Running Container...` and select the container shown by `docker compose ps`.

### 3) Use multiple ROS terminals

Open as many integrated terminals as you need (`Terminal -> New Terminal`), then run:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /root/workspace/install/setup.bash ] && source /root/workspace/install/setup.bash
```

Now each terminal is ready for ROS commands, nodes, launch files, bag tools, and debugging.

### 4) Build

Use `Ctrl+Shift+B` (task: `colcon build (R)`) or run:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd /root/workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Compose Runtime Contract

`docker-compose.yml` keeps host-oriented robotics access enabled by default:

- `privileged: true`
- `network_mode: host`
- `ipc: host`
- `pid: host`
- `/dev` bind mount
- `/tmp/.X11-unix` bind mount
- NVIDIA GPU reservation

### Base vs Derived Image

- Base image (upstream): `${BASE_IMAGE_REPO}:${ROS_DISTRO}${BASE_IMAGE_TAG_SUFFIX}`
- Derived dev image (this repo build): `${DEV_IMAGE_REPO}:${ROS_DISTRO}`

The derived image is built using `.devcontainer/Dockerfile` and is where you add project-specific dependencies.

## Mounts

| Host Path | Container Path | Purpose |
|---|---|---|
| `./workspace` | `/root/workspace` | ROS 2 workspace |
| `./data` | `/root/data` | Shared files (bags, datasets, configs) |
| `./.vscode` | `/root/workspace/.vscode` | VS Code workspace config |
| `./.clangd` | `/root/workspace/.clangd` | clangd config |
| `./.clang-format` | `/root/workspace/.clang-format` | C/C++ format style |
| `./ruff.toml` | `/root/workspace/ruff.toml` | Ruff lint/format config |
| `${HOME}/.ssh` | `/root/.ssh` (ro) | SSH credentials |
| `/dev` | `/dev` | Hardware device access |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | GUI socket |

## Dev Container Behavior

`.devcontainer/devcontainer.json` is configured to:

- Use Compose service `ros-dev`
- Open `/root/workspace`
- Start required service(s)
- Keep container running when VS Code closes (`"shutdownAction": "none"`)
- Auto-install recommended extensions in the container

## Tooling Setup

### C++

- `clangd` is the primary language server.
- `C_Cpp.intelliSenseEngine` is disabled to avoid dual-diagnostics conflicts.
- C/C++ formatting uses the `xaver.clang-format` VS Code extension + `.clang-format`.
- Format-on-save is intentionally disabled.
- Manual formatting: `Format Document` (`Ctrl+Shift+I` on Linux).
- Build task exports `compile_commands.json` for accurate indexing.

### Python

- Ruff is the default formatter.
- Format-on-save is intentionally disabled.
- Manual formatting: `Format Document` (`Ctrl+Shift+I` on Linux).
- Ruff fix/import actions are available manually from code actions / command palette.
- `ruff.toml` uses `src`-based detection for first-party import resolution:
  - `src = ["workspace/src", "src"]`

## Management Script

Use `scripts/manage.sh` for lifecycle operations:

```bash
./scripts/manage.sh start
./scripts/manage.sh stop
./scripts/manage.sh restart
./scripts/manage.sh recreate
./scripts/manage.sh logs
./scripts/manage.sh exec
```

`exec` uses Compose service targeting (`docker compose exec ros-dev bash`), so it stays consistent across ROS distro changes via environment or `.env`.
`start` and `recreate` also print `docker compose ps` to show the active container name and state.

## Dockerfile Customization

Add project dependencies to `.devcontainer/Dockerfile` (this derived layer), then rebuild:

```bash
./scripts/manage.sh recreate
```

## Data Folder Usage

Anything in `./data` on host is available at `/root/data` in container.

Example:

```bash
cp ~/Downloads/my_recording.db3 ./data/
```

Inside container:

```bash
ros2 bag play /root/data/my_recording.db3
```

## Debug Launch Template

`.vscode/launch.json` includes a template launch entry. Replace:

- `src/<your_package>/launch/<your_launch_file>.launch.py`

with your package-specific launch file path.

## Troubleshooting

### GUI apps do not open

```bash
xhost +local:docker
```

### GPU not visible

```bash
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
```

### Validate Compose config

```bash
docker compose config
```
