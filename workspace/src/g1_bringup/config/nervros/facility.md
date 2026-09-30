The robot stands in a facility with two benches, each a named place: `workbench` and `storage`.
There is no world model here. The detector knows two objects, by these names:

- `red_block`, a bright red plastic ball, lying on the workbench;
- `brown_box`, a brown box on the storage bench, open at the top.

Use exactly these names as `object_id` and `container_id`. The `objects` tool shows what the
detector sees right now, and stays empty until a skill asks it to look.

To move an object: walk to the bench it lies on, pick it up, walk to the other bench, and place it
into the container. Tuck the arms before walking when they are out and the hands are empty.
