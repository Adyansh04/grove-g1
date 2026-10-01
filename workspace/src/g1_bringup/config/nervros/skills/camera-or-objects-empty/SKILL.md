---
name: camera-or-objects-empty
description: Use when look gets no frame or a stale one, the camera rate is low or uneven, or find_objects and the detector find nothing although things are in view.
---

# The camera or the detector sees nothing

Three causes look the same from here: the camera is not publishing, its large images are being
dropped, or the detector is asked about too much to answer in time.

1. Call `topic_sample` on the colour image topic for a few seconds. No messages: the camera or its
   relay is down; tell the operator which topic is silent.
2. A rate that swings between about 1 and 6 Hz, or depth "not within tolerance of the mask stamp",
   means shared memory failed and big images fall back to dropped UDP. It happens after processes
   were killed with `kill -9`. The operator stops everything, clears `/dev/shm` in the container
   (`rm -f /dev/shm/*`) and starts again. You cannot do this yourself.
3. If the camera is fine but the objects list stays empty, call `log_tail` for `g1_detector`.
   "The newest camera frame is N s old; not asking about it" on a loop means the detector is asked
   for too many phrases: each costs about 0.25 s, and the answer must come within 1 s. Ask for
   fewer things at a time, such as `look` with one `question`, or tell the operator to narrow the
   detector's phrase list.
4. Otherwise, check where the robot is (the `lost-on-the-map` skill): an empty list often means
   the robot faces the wrong way.

Say which of these it was, in one or two sentences, and what the operator should do.
