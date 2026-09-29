# maps

One occupancy grid per world that has a map. `localization.launch.py` picks the one for `world:=`:
`facility` for `navigation`, `apartment` for `apartment`, and `facility` for anything else.

`facility` is the `navigation` world (`g1_navigation_scene.xml`), mapped with `mode:=mapping`:
361 by 359 cells at 5 cm, origin `[-9.05, -8.96]`, covering the 18 by 18 m facility with all four
rooms, the doorways, the shelving, the workbench and the pillars.

`apartment` is the `apartment` world: 420 by 260 cells at 5 cm, origin `[-5.5, -6.5]`.
`g1_bringup`'s `tools/build_world.py` rasterizes it from the scene instead of mapping it; see
[worlds/README.md](../../g1_bringup/worlds/README.md).

## Only the occupancy grid is committed

`map_saver_cli` writes `facility.pgm` and `facility.yaml`. slam_toolbox's serialized pose graph,
which its `localization` mode needs, comes out at 33 MB for this map against 130 KB for the grid,
so localization runs on `nav2_map_server` and AMCL instead.

## Regenerating the facility map

```bash
ros2 launch g1_bringup bringup.launch.py mode:=mapping
# drive the robot through all four rooms, then:
ros2 run nav2_map_server map_saver_cli -f /root/workspace/src/g1_navigation/maps/facility
```

Re-map whenever `g1_bringup`'s `g1_navigation_scene.xml` changes. Nothing checks that a map still
matches its scene; a stale map shows up as Nav2 planning through walls that moved.
