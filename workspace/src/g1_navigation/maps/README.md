# maps

`facility` is the `g1_navigation_scene` world, mapped with `mode:=mapping` while the robot was
driven around the four rooms. 361 by 359 cells at 5 cm, origin `[-9.05, -8.96]`, covering 18.1 by
18.0 m against a facility that is 18 by 18 m. All four rooms, the corridor doorways, the shelving,
the workbench and the pillars are resolved.

## Only the occupancy grid is committed

`map_saver_cli` produces `facility.pgm` and `facility.yaml`, which is what is here. slam_toolbox can
also serialize its pose graph, and that is what its `localization` mode loads, but for this map
those come out at 33 MB and 1.6 MB against 130 KB for the grid. A 33 MB binary that has to be
regenerated whenever the scene changes does not belong in git.

The consequence is that localization runs on `nav2_map_server` and AMCL rather than slam_toolbox's
`localization` mode, which needs the pose graph. AMCL's motion model is parameterised on odometry
noise, so its `alpha1` through `alpha5` are left at Nav2's defaults: under `odometry:=ground_truth`
the model is degenerate, and under `fast_lio` the noise is the simulated sensor's rather than the
robot's. Neither transfers to hardware as a tuning.

## Regenerating

```bash
ros2 launch g1_bringup bringup.launch.py mode:=mapping
# drive the robot through all four rooms, then:
ros2 run nav2_map_server map_saver_cli -f facility
```

Re-map whenever `g1_bringup`'s `g1_navigation_scene.xml` changes. Nothing checks that the committed
map still matches the scene, and a stale map shows up as Nav2 planning through walls that moved.
