# One segmented instance, cropped to its own bounding box: a full 848x480 mask is about 400 kB
# per instance, the crop of a small object a few kB.

# The phrase the detector was asked for, not the model's wording: object ids are built from it.
string label

# Detector confidence, calibrated or a bare ranking depending on the model. Carried through to
# the hypothesis score on /objects.
float32 score

# Where the crop sits in the full image.
sensor_msgs/RegionOfInterest roi

# roi.height * roi.width bytes, row major, 0 outside the instance and 255 inside. Bytes rather
# than packed bits, since erosion works on bytes.
uint8[] data
