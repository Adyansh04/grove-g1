#!/usr/bin/env python3
"""A SetLocoMode action server standing in for g1_loco_bridge, recording every goal it gets.

The real bridge cannot be held in the states these tests need. It succeeds, reports HELD and
puts itself back to active through loco.launch.py's lifecycle handlers, so the paths that only
run when an acquire goes wrong are unreachable against it. This stub can be parked in any of
them and left there.

Parameters:
  mode        "ok"       accept and succeed every goal
              "no_held"  accept and succeed, but never report HELD -- the acquire then fails at
                         waitForHeld(), which is the one failure that happens AFTER the node
                         believes it holds authority and therefore owes a release
  fsm_ids     latched output: the fsm_id of every goal received, in order
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32MultiArray

from g1_msgs.action import SetLocoMode
from g1_msgs.msg import LocoStatus

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class Stub(Node):
    def __init__(self):
        super().__init__("setmode_stub")
        self.mode = self.declare_parameter("mode", "ok").value
        self.held = False
        self.fsm_ids = []

        group = ReentrantCallbackGroup()
        self.srv = ActionServer(
            self,
            SetLocoMode,
            "~/set_mode",
            execute_callback=self.execute,
            callback_group=group,
        )
        self.status = self.create_publisher(LocoStatus, "~/status", LATCHED)
        # The goal log goes out on a topic rather than a service so the test can read it after
        # the node under test has already failed and stopped talking.
        self.goals = self.create_publisher(Int32MultiArray, "~/goals", LATCHED)
        self.create_timer(0.2, self.publish_status, callback_group=group)
        self.publish_goals()

    def publish_status(self):
        m = LocoStatus()
        m.stamp = self.get_clock().now().to_msg()
        m.fsm_id = 500 if self.held else 4
        m.authority = LocoStatus.HELD if self.held else LocoStatus.RELEASED
        self.status.publish(m)

    def publish_goals(self):
        self.goals.publish(Int32MultiArray(data=self.fsm_ids))

    def execute(self, handle):
        fsm_id = handle.request.fsm_id
        self.fsm_ids.append(fsm_id)
        self.publish_goals()
        self.get_logger().info(f"goal {len(self.fsm_ids)}: fsm_id={fsm_id}")

        if fsm_id == SetLocoMode.Goal.START and self.mode != "no_held":
            self.held = True
        if fsm_id == SetLocoMode.Goal.STAND_UP:
            self.held = False

        result = SetLocoMode.Result()
        result.success = True
        result.error_code = 0
        result.message = "stub ok"
        handle.succeed()
        return result


def main():
    rclpy.init()
    node = Stub()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
