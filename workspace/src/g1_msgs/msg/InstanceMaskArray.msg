# Every instance a detector found in one frame.

# The stamp and frame of the IMAGE these were computed from, never the moment they were
# published. A detector takes over a second, and the geometry node pairs on this stamp to find
# the depth frame that matches rather than the newest one.
std_msgs/Header header

# The image the ROIs are indexed against, so a consumer can refuse a mismatched depth frame.
uint32 image_width
uint32 image_height

# Which model produced them, for the log. Nothing branches on this: that is what lets the
# segmentation model be swapped without touching anything downstream.
string model

InstanceMask[] instances
