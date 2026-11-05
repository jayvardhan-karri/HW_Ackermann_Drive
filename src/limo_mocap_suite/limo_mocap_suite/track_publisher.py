# #############################################################################
# [FILE]   track_publisher.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/track_publisher.py
# ROLE  : Sends a capsule track instead of a hand-drawn path, once the controller
#         is ready, then leaves. The controller cannot tell the difference.
#
# Console script : ros2 run limo_mocap_suite track_publisher
# Also           : hw_tracking.launch.py track_radius:=1.0
# Publishes      : /reference_path
# Subscribes     : /run_status
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
# #############################################################################

import json
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from limo_mocap_suite.ros_compat import init_ros
from limo_mocap_suite.track_tools import capsule_message


class TrackPublisher(Node):
    def __init__(self):
        super().__init__('track_publisher')
        for name, value in (('track_radius', 1.0), ('track_straight', 3.0), ('track_x', 3.75), ('track_y', 2.5),
                            ('lab_x', [-0.5, 8.0]), ('lab_y', [-1.0, 6.0]), ('start_delay', 5.0)):
            self.declare_parameter(name, value)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.track = capsule_message(g('track_straight'), g('track_radius'), g('track_x'), g('track_y'),
                                     self.get_clock().now().to_msg(), g('lab_x'), g('lab_y'))
        self.pub = self.create_publisher(Path, '/reference_path', 10)
        self.create_subscription(String, '/run_status', self.status_cb, 10)
        self.start_at = time.monotonic() + float(g('start_delay'))
        self.sent = 0.0
        self.done = False
        self.get_logger().warn(f"Capsule track R {g('track_radius')} m: the robot moves when the controller is "
                               f"ready, at least {g('start_delay')} s from now. Stop it with KILL in the live view.")

    # =========================================================================
    # [ROS-IO] status_cb(msg: String)
    # -------------------------------------------------------------------------
    # Sends the track while the controller reports it is waiting, every 3 s in
    # case it was missed; done as soon as the controller has started.
    # =========================================================================
    def status_cb(self, msg):
        phase = json.loads(msg.data)['phase']
        now = time.monotonic()
        if phase in ('APPROACH', 'TRACK'):
            self.done = True
        elif phase == 'WAIT_PATH' and now >= self.start_at and now - self.sent > 3.0:
            self.pub.publish(self.track)
            self.sent = now


def main(args=None):
    init_ros(args)
    node = TrackPublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
