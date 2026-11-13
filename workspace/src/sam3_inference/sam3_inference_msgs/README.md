# sam3_inference_msgs

The interface `sam3_inference` serves, kept in its own package so a client builds without CUDA or
TensorRT.

- `srv/SegmentObjects.srv`: an image (or none, for the server's newest camera frame), the phrases
  to look for and a minimum score in; success, a message, the segmented image's header and the
  instances out.
- `msg/Segment.msg`: one instance: its phrase, score, box in the image and `mono8` mask cropped
  to the box, 255 inside.
