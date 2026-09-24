# Every instance a detector found in one frame.

# Stamp and frame of the source IMAGE, not the publish time: detection takes over a second, and
# the geometry node pairs on this stamp to find the matching depth frame.
std_msgs/Header header

# Size of the image the ROIs index into, so a consumer can refuse a mismatched depth frame.
uint32 image_width
uint32 image_height

# The model that produced them, for logs only. Nothing branches on it, so the segmenter can be
# swapped without touching anything downstream.
string model

InstanceMask[] instances
