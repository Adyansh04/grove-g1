# The apartment, explored

`apartment/` is the world the acceptance test saved after exploring the apartment from no map,
with both cameras:
- `map.pgm` and `map.yaml`: the floor plan.
- `semantic_map.png`: a picture of it with the rooms and objects drawn in.
- `world.yaml`: every room and object.
- `coverage.bin`, `objects.bin` and `wall_hits.png`: the layers the world model and the C++
  replays load.

The object crops are left out. Its map lies in the simulator's world at (0.46, -0.02) m, -5.5°,
as the test's "map to world" line printed it:

```bash
ros2 run g1_world_model compare_truth.py src/g1_world_model/doc/apartment --frame 0.46 -0.02 -5.5
```

The pictures:
- `apartment_truth.png` draws the scene's walls (blue) and furniture (orange) over the saved floor
  plan (`compare_truth.py --overlay`).
- `apartment_wardrobe.png` is the corner behind the bedroom's wardrobe, from the same command with
  `--crop 4 2.5 10.5 7.2`. The gap is narrower than a spot the robot can stand in, so it looks in
  from the nearest one, and the LiDAR sees only part of it.
- `apartment_walk.png` is the robot's walk, blue early to red late, with each viewpoint
  (`run_summary.py --plot`).
- `frontier_pass_end.png` is SLAM's map as the frontier pass ended, from an earlier run
  (`snapshot_map.py`). Free is white, occupied black, unknown grey, and frontiers red. Most of the
  red is speckle between LiDAR beams, which the camera pass fills in.
