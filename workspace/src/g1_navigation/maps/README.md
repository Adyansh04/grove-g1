# maps

`facility` is the `g1_navigation_scene` world, mapped with `mode:=mapping`: 361 by 359 cells at
5 cm, origin `[-9.05, -8.96]`, covering the 18 by 18 m facility with all four rooms, the doorways,
the shelving, the workbench and the pillars.

## Only the occupancy grid is committed

`map_saver_cli` writes `facility.pgm` and `facility.yaml`. slam_toolbox's serialized pose graph,
which its `localization` mode needs, comes out at 33 MB for this map against 130 KB for the grid,
so localization runs on `nav2_map_server` and AMCL instead. AMCL's `alpha1` to `alpha5` stay at
Nav2's defaults: neither simulated odometry source says anything about the real robot's noise.

## Regenerating

```bash
ros2 launch g1_bringup bringup.launch.py mode:=mapping
# drive the robot through all four rooms, then:
ros2 run nav2_map_server map_saver_cli -f facility
```

Re-map whenever `g1_bringup`'s `g1_navigation_scene.xml` changes. Nothing checks that the map still
matches the scene; a stale map shows up as Nav2 planning through walls that moved.
