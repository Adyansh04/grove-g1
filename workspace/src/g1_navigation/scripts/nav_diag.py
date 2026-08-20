"""Records navigation health while the robot drives.

Started by `nav_soak`, and usable by hand against an already-running stack:

    ros2 run g1_navigation nav_diag.py 200

Prints a snapshot every 20 s so a long run can be watched, and a summary on exit. The
summary is the output that matters; the snapshots are progress.
"""
import math
import signal
import statistics as st
import sys

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Duration, Time
from sensor_msgs.msg import PointCloud2

SWEEP_S = 0.032        # measured cost of one full 360x32 sweep
OBSTACLE_RANGE = 5.0   # obstacle_max_range in config/nav2_params.yaml
FLOOR_CUT = 0.08       # min_obstacle_height
# Measured envelope of the walking policy: translation commands below this produce essentially
# no motion. Yaw has no deadband and tracks near 1:1, so it only appears here as "is it turning".
VEL_DEADBAND = 0.15


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pitch_of(q):
    s = 2.0 * (q.w * q.y - q.z * q.x)
    return math.asin(max(-1.0, min(1.0, s)))


class NavDiag(Node):
    def __init__(self):
        super().__init__("nav_diag")
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.lis = tf2_ros.TransformListener(self.buf, self)

        self.tilt_err = []
        self.map_odom = []
        self.jumps = []
        self.pitch = []
        self.lethal = []
        self.odom_xy = []
        self.nav_cmd = 0
        self.nav_cmd_in_deadband = 0

        self.create_subscription(PointCloud2, "/livox/lidar", self.on_cloud,
                                 QoSProfile(depth=5,
                                            reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                            history=QoSHistoryPolicy.KEEP_LAST))
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.on_costmap,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST))
        self.create_subscription(Twist, "/cmd_vel", self.on_nav_cmd, 10)
        self.create_timer(0.05, self.on_tick)
        self.create_timer(20.0, self.snapshot)

    def on_nav_cmd(self, msg):
        """Nav2's output. A command inside the deadband with no turn moves the robot nowhere."""
        self.nav_cmd += 1
        if (abs(msg.linear.x) < VEL_DEADBAND and abs(msg.linear.y) < VEL_DEADBAND
                and abs(msg.angular.z) < 1e-6):
            self.nav_cmd_in_deadband += 1

    def on_cloud(self, msg):
        """How far the pelvis tilt moves across one sweep window.

        The cloud is stamped at relay publish time, but the geometry it describes was sampled
        up to a sweep earlier, so this is the attitude error the costmap transform inherits.
        """
        stamp = Time.from_msg(msg.header.stamp)
        try:
            a = self.buf.lookup_transform("base_footprint", "pelvis", stamp)
            b = self.buf.lookup_transform("base_footprint", "pelvis",
                                          stamp - Duration(seconds=SWEEP_S))
        except Exception:
            return
        self.tilt_err.append(
            math.degrees(abs(pitch_of(a.transform.rotation) - pitch_of(b.transform.rotation))))

    def on_costmap(self, msg):
        # >= 99, not >= 253. The costmap is republished as an OccupancyGrid, and
        # Costmap2DPublisher rescales on the way out: LETHAL_OBSTACLE 254 -> 100,
        # INSCRIBED_INFLATED_OBSTACLE 253 -> 99, NO_INFORMATION -> -1, everything else 0-98.
        # Thresholding on the raw costmap values could never match anything, so this read zero
        # through every run whatever the costmap actually contained.
        self.lethal.append(sum(1 for v in msg.data if v >= 99))

    def on_tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        try:
            tr = self.buf.lookup_transform("map", "odom", rclpy.time.Time())
            x, y = tr.transform.translation.x, tr.transform.translation.y
            yaw = yaw_of(tr.transform.rotation)
            if self.map_odom:
                _, px, py, pyaw = self.map_odom[-1]
                dxy = math.hypot(x - px, y - py)
                dyaw = abs(math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw)))
                if dxy > 0.005 or dyaw > 0.005:
                    self.jumps.append((t, dxy, dyaw))
            self.map_odom.append((t, x, y, yaw))
        except Exception:
            pass
        try:
            tr = self.buf.lookup_transform("base_footprint", "pelvis", rclpy.time.Time())
            self.pitch.append(pitch_of(tr.transform.rotation))
        except Exception:
            pass
        try:
            tr = self.buf.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            self.odom_xy.append((tr.transform.translation.x, tr.transform.translation.y))
        except Exception:
            pass

    def distance(self):
        return sum(math.hypot(self.odom_xy[i][0] - self.odom_xy[i - 1][0],
                              self.odom_xy[i][1] - self.odom_xy[i - 1][1])
                   for i in range(1, len(self.odom_xy)))

    def snapshot(self):
        pct = (100.0 * self.nav_cmd_in_deadband / self.nav_cmd) if self.nav_cmd else 0.0
        print("[snap] driven=%.2fm  nav2_cmds=%d (%.0f%% in deadband)  "
              "amcl_corrections=%d  lethal_now=%s"
              % (self.distance(), self.nav_cmd, pct, len(self.jumps),
                 self.lethal[-1] if self.lethal else "n/a"))
        sys.stdout.flush()

    def report(self):
        def stats(v, scale=1.0, unit=""):
            if not v:
                return "no data"
            w = sorted(x * scale for x in v)
            return ("n=%d  min=%.3f  med=%.3f  p95=%.3f  max=%.3f %s"
                    % (len(w), w[0], st.median(w), w[int(0.95 * (len(w) - 1))], w[-1], unit))

        print("\n================ NAV DIAGNOSTICS ================")
        print("distance driven       : %.2f m" % self.distance())
        if self.nav_cmd:
            print("nav2 cmd_vel          : %d, of which %d (%.0f%%) inside the gait deadband"
                  % (self.nav_cmd, self.nav_cmd_in_deadband,
                     100.0 * self.nav_cmd_in_deadband / self.nav_cmd))
        print("pelvis pitch          :", stats(self.pitch, 180 / math.pi, "deg"))
        if self.pitch:
            print("  swing               : %.2f deg peak to peak"
                  % ((max(self.pitch) - min(self.pitch)) * 180 / math.pi))
        print("tilt change per sweep :", stats(self.tilt_err, 1.0, "deg"))
        if self.tilt_err:
            mx = max(self.tilt_err)
            print("  vertical error at %.0f m: max %.2f m  (floor cut is %.2f m)"
                  % (OBSTACLE_RANGE, OBSTACLE_RANGE * math.sin(math.radians(mx)), FLOOR_CUT))
        print("local costmap lethal  :", stats(self.lethal))
        span = (self.map_odom[-1][0] - self.map_odom[0][0]) if self.map_odom else 0
        print("map->odom corrections : %d over %.0f s" % (len(self.jumps), span))
        if self.jumps:
            print("  translation         :", stats([j[1] for j in self.jumps], 100, "cm"))
            print("  rotation            :", stats([j[2] for j in self.jumps], 180 / math.pi, "deg"))
            if len(self.jumps) > 1:
                print("  gaps between them   :",
                      stats([self.jumps[i][0] - self.jumps[i - 1][0]
                             for i in range(1, len(self.jumps))], 1.0, "s"))
        print("=================================================\n")
        sys.stdout.flush()


def main():
    rclpy.init()
    node = NavDiag()

    # The summary has to survive a signal. KeyboardInterrupt is a BaseException, so an
    # `except Exception` around the spin loop silently skips report() and the run ends with
    # no numbers at all.
    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 200.0
    end = node.get_clock().now().nanoseconds * 1e-9 + duration
    try:
        while rclpy.ok() and not stop["now"]:
            if node.get_clock().now().nanoseconds * 1e-9 >= end:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except BaseException:
        pass
    node.report()


if __name__ == "__main__":
    main()
