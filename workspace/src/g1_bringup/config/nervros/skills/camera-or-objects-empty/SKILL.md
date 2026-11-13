---
name: camera-or-objects-empty
description: Use when look gets no frame or a stale one, the camera rate is low or uneven, or find_objects and the detector find nothing although things are in view.
---

# The camera or the detector sees nothing

Several causes look the same from here: the camera is not publishing, its detector stopped, or the
detector is asked about too much to answer in time.

1. Call `why` first: an incident the robot opened by itself may already name the cause, such as
   a camera's detector that stopped. If it does, say that cause and its fix.
2. Call `topic_sample` on the colour image topic for a few seconds. No messages: the camera or its
   relay is down; tell the operator which topic is silent.
3. If the camera is fine but nothing is found through it, look for its detector in `ros_graph`
   (`detector_head`, `detector_chest`): one that is missing is down, and the operator restarts the
   world model stack. If it runs, call `log_tail` for it.
   "The newest camera frame is N s old; not asking about it" on a loop means the detector's
   answers come too late: SAM 3.1 takes a few tenths of a second a frame, shared with the world
   model's mapping and the simulator's GPU, and a frame older than 2.5 s is not asked about. Ask for fewer things at
   a time, such as `look` with one `question`, or tell the operator to narrow the detector's
   phrase list.
4. Otherwise, check where the robot is (the `lost-on-the-map` skill): an empty list often means
   the robot faces the wrong way.

Say which of these it was, in one or two sentences, and what the operator should do.
