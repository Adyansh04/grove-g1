"""The vision server's parts, run by servers/vision_server.py.

masks          masks as they go on the wire, and one region per phrase
segmentation   the Grounded SAM 2 and SAM 3 backends behind one reply format
grounding      instructions turned into noun phrases by a vision-language model
server         the request handlers and the ZMQ loop
"""
