# One object the world model fused from several views. Poses are in the header frame of the
# enclosing array, normally `map`.

# Stable for the life of the world file: "O12". Never reused after removal.
string id

# The detector label with the most votes.
string label

# Canonical name and one-line caption from the describer. Empty until described.
string name
string caption

# Share of all label votes held by `label`, 0..1.
float32 confidence

# Room the footprint centre falls in, and the object it rests on. Empty when unknown or on the
# floor.
string room_id
string support_id

# Centre of the oriented bounding box; only yaw is set in the orientation.
geometry_msgs/Pose pose

# Full extents along the box axes, m.
geometry_msgs/Vector3 size

uint32 observations
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen

# Stale: looked at and not found; removed: looked at and not found repeatedly. Kept rather than
# deleted so a query can still say where the object used to be.
uint8 STATE_ACTIVE=0
uint8 STATE_STALE=1
uint8 STATE_REMOVED=2
uint8 state
