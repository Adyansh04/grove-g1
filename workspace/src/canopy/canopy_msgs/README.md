# canopy_msgs

canopy's interfaces. `ament_cmake` with `rosidl_default_generators`, no source of its own.

```bash
colcon build --symlink-install --packages-select canopy_msgs
```

## Messages

| Message | Carries | Notes |
|---|---|---|
| `InstanceMask` | `label`, `score`, `roi`, `data`, `embedding` | One object. `data` is a 0-or-255 crop of `roi`, not a full frame. `label` is the phrase the detector was asked for, so the object id holds when the model rewords its answer. `embedding` is empty unless the detector embeds. |
| `InstanceMaskArray` | `header`, `image_width`, `image_height`, `model`, `instances` | `header` is the image's stamp and frame, not the publish time: consumers pair it with depth on it. Nothing reads `model`, which keeps the segmenter swappable. |
| `DescribeRequest` | `subject_id`, `task`, `images`, `labels`, `votes` | An object crop or a room's frames, with what the map believes so far. |
| `Description` | `name`, `caption`, `label_ok`, `room_type`, `confidence`, `model` | The describer's answer. `model` is for logs only. |
| `Room`, `RoomArray` | id, name, type and its source, outline, doorways, coverage | Ids (`R3`) are stable across re-segmentation and never reused. |
| `Doorway` | `to_room`, `centre`, `width` | Where two rooms' regions touch. |
| `WorldObject`, `WorldObjectArray` | id, label, describer name, room, support, oriented box, state | Ids (`O12`) live as long as the world file. Stale and removed objects are kept, so a query can say where one used to be. |

## Services

| Service | Notes |
|---|---|
| `FindObjects` | A label, name, synonym or free text, optionally in one room, best first. With a `query_embedding` from the objects' embedding model, it also ranks by similarity. `found` false with a coverage figure means "not in what the camera has seen", not "absent". |
| `GetApproachPose` | An object id, room or label in; a reachable pose facing it out, ready for `NavigateToPose`. |
| `NextViewpoint` | Where to stand and which headings to face next, in `frontier` or `coverage` mode. Called in a loop by an exploration tree. |
| `ReportViewpoint` | Whether a viewpoint was reached, so an unreachable one is not offered again. |
