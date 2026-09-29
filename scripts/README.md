# scripts

Host-side scripts, run from the repository root.

| Script | What it does |
|---|---|
| `manage.sh` | Runs the dev container and the everyday commands inside it: `start`, `stop`, `restart`, `recreate`, `logs`, `exec`, `build`, `test [--sim]`, `format`, `lint`. |
| `clean-stack.sh` | Tears down whatever stack is running, host or container, and exits non-zero unless the ROS graph is empty. Run it between launches. |
| `demos/*.sh` | One launcher per guide in [`docs/guides/`](../docs/guides): opens a demo's commands in split panes, and `stop` ends it. |
| `serve.sh` | Starts a model server on the host: `groot` (5555), `graspgen` (5556), `vision` (5560) or `canopy` (5561). |
| `setup-groot.sh`, `setup-vision.sh`, `setup-graspgen.sh` | Installs the matching model server on the host. All three are optional: every simulator test runs on the mock engine, detector or grasp source instead. |
| `setup-world-assets.sh` | Fetches the apartment world's meshes and textures into `workspace/assets/`. Only `world:=apartment` needs them. |
| `lib/panes.sh` | Shared by the demo launchers: argument parsing, the pane layout, and waiting for the stack. |
