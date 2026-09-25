# A region of the map, segmented from the occupancy grid. Frame from the enclosing array.

# Stable across re-segmentation: "R3". Never reused.
string id

# What people call it: "room C", or a name an operator gave it.
string name

# "kitchen", "office", ...; empty until typed. `type_source` says what decided it: objects,
# describer or operator.
string type
float32 type_confidence
string type_source

geometry_msgs/Polygon outline
geometry_msgs/Point centroid
float32 area

Doorway[] doorways

# Fractions of the observable targets seen well enough by the camera, 0..1.
float32 floor_coverage
float32 face_coverage
float32 surface_coverage

# Targets no reachable standing pose can see, e.g. a shelf above camera height.
uint32 unobservable_cells

uint32 object_count
