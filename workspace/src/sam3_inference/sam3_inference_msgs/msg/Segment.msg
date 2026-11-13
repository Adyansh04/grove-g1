# One instance mask from the open-vocabulary segmenter.

# The prompt this instance answers.
string label

# Segmentation score, 0 to 1.
float32 score

# Box in the image, pixels.
sensor_msgs/RegionOfInterest roi

# mono8 mask cropped to roi, 255 inside the instance.
sensor_msgs/Image mask
