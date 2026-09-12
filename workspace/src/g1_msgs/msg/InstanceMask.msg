# One segmented object instance, cropped to its own bounding box.
#
# The crop is what travels: a full-frame mask at 848x480 is 407 kB per instance against about
# 3 kB for a 6 cm object at half a metre, and a consumer walks the rectangle either way.

# What this instance answers to. The detector writes the noun phrase it was asked for, not the
# model's own wording, because the object id downstream is built from the phrase.
string label

# The detector's confidence. Calibrated on models that say so, a bare ranking otherwise; it is
# carried through to the hypothesis score on /objects rather than interpreted here.
float32 score

# Where the crop sits in the full image.
sensor_msgs/RegionOfInterest roi

# roi.height * roi.width bytes, row major, 0 outside the instance and 255 inside. Not bit packed:
# at this size the saving is not worth the bit arithmetic on both ends, and erosion wants bytes.
uint8[] data
