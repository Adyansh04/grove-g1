"""ZMQ client for GraspGenX upstream, on the transport canopy_perception's model clients use."""

from canopy_perception.host_clients import ReqClient


class GraspClient(ReqClient):
    """GraspGenX upstream: the call is named by "action" and its body is flattened beside it."""

    def __init__(self, address, timeout_ms):
        super().__init__(address, timeout_ms, "the grasp generator")

    def call(self, action, payload=None):
        return self._send(action, {"action": action, **(payload or {})})
