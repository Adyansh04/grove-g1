# maps

`facility` is the `g1_navigation_scene` world, mapped with `slam.launch.py mode:=mapping` while
the robot was driven around the four rooms. 361 x 359 cells at 5 cm, origin `[-9.05, -8.96]`,
covering 18.1 x 18.0 m — the facility is 18 x 18 m. All four rooms, the corridor doorways, the
shelving, the workbench and the pillars are resolved.

## Only the occupancy grid is committed

`map_saver_cli` produces `facility.pgm` + `facility.yaml`, which is what is here. slam_toolbox can
also serialize its pose graph, and that is what its `localization` mode loads — but for this map
those come out at **33 MB (`.posegraph`) and 1.6 MB (`.data`)**, against 130 KB for the grid. A
33 MB binary that has to be regenerated whenever the scene changes does not belong in git.

The consequence is that localization runs on `nav2_map_server` + AMCL rather than slam_toolbox's
`localization` mode, which needs the pose graph. The cost is real and worth knowing: AMCL's motion
model is parameterised on odometry noise, and this track's odometry is exact ground truth, so the
model is degenerate and its `alpha1..5` cannot be tuned to anything that transfers to hardware.

## Regenerating

```bash
ros2 launch g1_navigation nav_sim.launch.py mode:=mapping
# drive the robot through all four rooms, then:
ros2 run nav2_map_server map_saver_cli -f facility
```

Re-map whenever `g1_bringup`'s `g1_navigation_scene.xml` changes. Nothing checks that the committed
map still matches the scene — a stale map shows up as Nav2 planning through walls that moved.
