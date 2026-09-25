# A request for a describer to name an object or a room from a few images.

# The object ("O12") or room ("R3") the answer belongs to.
string subject_id

string TASK_OBJECT="object"
string TASK_ROOM="room"
string task

# Best views, cropped for objects, whole frames for rooms.
sensor_msgs/CompressedImage[] images

# What the map already believes: label votes for an object, object labels for a room.
string[] labels
float32[] votes
