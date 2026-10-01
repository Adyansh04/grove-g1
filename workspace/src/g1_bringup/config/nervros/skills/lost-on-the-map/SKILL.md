---
name: lost-on-the-map
description: Use when the robot arrived somewhere but cannot find what should be there, a pick finds nothing called the object, or the 3D view shows the robot somewhere it is not.
---

# The robot may be lost on the map

Localization can settle far off, even 90 degrees turned, and then Nav2 "arrives" at the wrong
place. Everything after that fails as if perception were broken.

1. Call `tf` for `map` to `pelvis` and note x, y and the heading.
2. In simulation, call `topic_sample` on `/g1_sensor_relay/base_state`: it is where the robot
   truly stands (its frame is labelled odom; it is the simulator's world within about 1 cm).
3. If the two disagree by more than about 0.5 m or 20 degrees, the robot is lost. Do not replan
   the same mission: it would fail the same way.
4. Call `look` to see what the camera sees now, and compare it with what should be at the place.

Tell the operator the robot is mislocalized, by how much, and that it needs its pose set again
(the 2D Pose Estimate in RViz, or a restart of the stack) before any walk.
