# scripts

Host-side scripts, run from the repository root.

| Script | What it does |
|---|---|
| `manage.sh` | Runs the dev container and the everyday commands inside it: `start`, `stop`, `restart`, `recreate`, `logs`, `exec`, `build`, `test [--sim]`, `format`, `lint`. |
| `robot.sh` | The robot side, on the G1's Orin, which needs no internet: `image` builds `robot/Dockerfile` for arm64 here and loads it there, `sync` copies the robot-side packages, `build` builds them there, `up`/`down` start or remove the idle container, `exec` runs a command in it. Taking the motors stays a separate `control.launch.py hardware:=true`. |
| `clean-stack.sh` | Tears down whatever stack is running, host or container, and exits non-zero unless the ROS graph is empty. Run it between launches. |
| `demos/*.sh` | One launcher per guide in [`docs/guides/`](../docs/guides). Opens a demo's commands in split panes. `stop` ends it. |
| `serve.sh` | Starts a model server on the host: `groot` (5555), `graspgen` (5556), `vision` (5560) or `canopy` (5561). |
| `setup-groot.sh`, `setup-vision.sh`, `setup-graspgen.sh` | Installs the matching model server on the host. All three are optional: every simulator test runs on the mock engine, detector or grasp source instead. |
| `setup-world-assets.sh` | Fetches the apartment world's meshes and textures into `workspace/assets/`. Only `world:=apartment` needs them. |
| `lib/panes.sh` | Shared by the demo launchers: argument parsing, the pane layout, and waiting for the stack. |
