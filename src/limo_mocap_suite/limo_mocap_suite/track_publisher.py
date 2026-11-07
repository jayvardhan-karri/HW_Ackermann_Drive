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
# Subscribes     : /run_status, /hardware_odom (start_side auto: top, or bottom if the robot is nearer it)
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
#   _name    a leading underscore = internal helper, only used inside this file/class
# #############################################################################

import json
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String

from limo_mocap_suite.ros_compat import init_ros
from limo_mocap_suite.track_tools import (LAB_X, LAB_Y, TRACK_RADIUS, TRACK_STRAIGHT, TRACK_X, TRACK_Y,
                                          capsule_message, pick_side)


# =============================================================================
# [LIFE]   class TrackPublisher(Node)
# -----------------------------------------------------------------------------
# Waits start_delay seconds and for the controller to report WAIT_PATH, publishes
# the track on /reference_path, and ends the process once the run has started.
# =============================================================================
class TrackPublisher(Node):
    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Declares the track parameters (lengths in m, start_delay in s), checks the
    # track fits the lab, creates the topics. Nothing is sent yet.
    # =========================================================================
    def __init__(self):
        super().__init__('track_publisher')
        for name, value in (('track_radius', TRACK_RADIUS), ('track_straight', TRACK_STRAIGHT),
                            ('track_x', TRACK_X), ('track_y', TRACK_Y), ('lab_x', list(LAB_X)), ('lab_y', list(LAB_Y)),
                            ('start_delay', 5.0), ('start_side', 'auto'), ('direction', 'ccw'),
                            ('start_at', 'middle')):
            self.declare_parameter(name, value)
        g = lambda n: self.get_parameter(n).value  # noqa: E731   short parameter reader
        self.g = g
        # checked now, before anything moves; the side is chosen when it is sent
        capsule_message(g('track_straight'), g('track_radius'), g('track_x'), g('track_y'),
                        self.get_clock().now().to_msg(), g('lab_x'), g('lab_y'), 'top', g('direction'), g('start_at'))
        self.robot_y = None
        self.create_subscription(Odometry, '/hardware_odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Path, '/reference_path', 10)
        self.create_subscription(String, '/run_status', self.status_cb, 10)
        self.start_at = time.monotonic() + float(g('start_delay'))
        self.sent = 0.0
        self.done = False
        self.get_logger().warn(f"Capsule track R {g('track_radius')} m: the robot moves when the controller is "
                               f"ready, at least {g('start_delay')} s from now. Stop it with KILL in the live view.")

    # =========================================================================
    # [ROS-IO] odom_cb(msg: Odometry)
    # -------------------------------------------------------------------------
    # Keeps the robot's y (m) to pick the start side (start_side 'auto').
    # =========================================================================
    def odom_cb(self, msg):
        self.robot_y = msg.pose.pose.position.y

    # =========================================================================
    # [CORE]   track() -> Path, with the start side chosen from the robot now
    # -------------------------------------------------------------------------
    # pick_side() chooses top / bottom, then capsule_message() builds the Path.
    # =========================================================================
    def track(self):
        g = self.g
        side = pick_side(g('start_side'), self.robot_y, g('track_y'))
        self.get_logger().info(f'Start: {g("start_at")} of the {side} straight', once=True)
        return capsule_message(g('track_straight'), g('track_radius'), g('track_x'), g('track_y'),
                               self.get_clock().now().to_msg(), g('lab_x'), g('lab_y'), side, g('direction'), g('start_at'))

    # =========================================================================
    # [ROS-IO] status_cb(msg: String)
    # -------------------------------------------------------------------------
    # Sends the track while the controller reports it is waiting, every 3 s in
    # case it was missed; done as soon as the controller has started.
# The first send waits for start_delay, and for odometry when start_side is 'auto'.
    # =========================================================================
    def status_cb(self, msg):
        phase = json.loads(msg.data)['phase']
        now = time.monotonic()
        if phase in ('APPROACH', 'TRACK'):
            self.done = True
        elif phase == 'WAIT_PATH' and now >= self.start_at and now - self.sent > 3.0:
            if self.g('start_side') == 'auto' and self.robot_y is None:
                return          # wait for the robot's position to pick the side
            self.pub.publish(self.track())
            self.sent = now


# =============================================================================
# [LIFE]   main(args=None)  -> console script track_publisher
# -----------------------------------------------------------------------------
# Spins the node until the controller has started (done), then shuts down.
# Ctrl-C is a normal exit.
# =============================================================================
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
