# #############################################################################
# [FILE]   live_plotter_node.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/live_plotter_node.py
# ROLE  : Live off-board view of a hardware run: reference path, robot trail,
#         robot glyph with heading, and the bridge's obstacle map.
#
# Console script : ros2 run limo_mocap_suite live_plotter
# Subscribes     : /reference_path (Path), /hardware_odom (Odometry),
#                  /hardware_obstacles (Float32MultiArray, latched)
# Parameters     : rezero_odom (True): must match the controller's value
#                  lab_x, lab_y ([-0.5, 8.0], [-1.0, 6.0]): canvas = lab floor
# Robot glyph    : arrow from rear marker M1 (blue) to front marker M0 (red),
#                  so the heading is readable without the Gazebo view.
# Hardware port of limo_gz_sim/live_plotter_node.py. Mostly [UI].
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
# #############################################################################

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from limo_mocap_suite.plot_helpers import (
    StartFrame, RobotGlyph, draw_obstacles, quat_to_yaw, declare_lab_params, style_lab_axes,
    init_gui_ros, start_spin_thread)


# =============================================================================
# [ROS-IO] class LivePlotterNode(Node)
# -----------------------------------------------------------------------------
# Caches path, trail, current pose and obstacle map for the animation.
# =============================================================================
class LivePlotterNode(Node):
    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Reads rezero_odom and lab_x / lab_y, creates the StartFrame, the buffers
    # and the three
    # subscriptions (/hardware_obstacles with latched QoS).
    # =========================================================================
    def __init__(self):
        super().__init__('live_plotter_node')
        self.declare_parameter('rezero_odom', True)
        self.frame = StartFrame(bool(self.get_parameter('rezero_odom').value))
        self.lab_x, self.lab_y = declare_lab_params(self)

        self.ref_x, self.ref_y = [], []
        self.trail_x, self.trail_y = [], []
        self.pose = None            # (x, y, yaw) in the controller frame
        self.obstacles = []
        self.obstacles_version = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/reference_path', self.path_cb, 10)
        self.create_subscription(Odometry, '/hardware_odom', self.odom_cb, 10)
        self.create_subscription(Float32MultiArray, '/hardware_obstacles', self.obstacles_cb, latched)
        self.get_logger().info('Live plotter started. Waiting for data...')

    # =========================================================================
    # [ROS-IO] path_cb(msg: Path)
    # -------------------------------------------------------------------------
    # Stores the drawn path (already in the controller frame) and clears the
    # trail: a new path means a new run.
    # =========================================================================
    def path_cb(self, msg):
        self.ref_x = [p.pose.position.x for p in msg.poses]
        self.ref_y = [p.pose.position.y for p in msg.poses]
        self.trail_x.clear()
        self.trail_y.clear()

    # =========================================================================
    # [ROS-IO] odom_cb(msg: Odometry)
    # -------------------------------------------------------------------------
    # Latches the frame on the first pose (like the controllers), stores the
    # transformed pose and appends a trail point when the robot moved > 2 cm.
    # =========================================================================
    def odom_cb(self, msg):
        p = msg.pose.pose
        raw = (p.position.x, p.position.y, quat_to_yaw(p.orientation))
        self.frame.latch(*raw)
        x, y, yaw = self.frame.apply(*raw)
        self.pose = (x, y, yaw)
        if not self.trail_x or math.hypot(x - self.trail_x[-1], y - self.trail_y[-1]) > 0.02:
            self.trail_x.append(x)
            self.trail_y.append(y)

    # =========================================================================
    # [ROS-IO] obstacles_cb(msg: Float32MultiArray)
    # -------------------------------------------------------------------------
    # Caches the bridge's obstacle map (flat [x, y, r, ...]).
    # =========================================================================
    def obstacles_cb(self, msg):
        self.obstacles = list(msg.data)
        self.obstacles_version += 1


# =============================================================================
# [LIFE]   main(args=None)
# -----------------------------------------------------------------------------
# Spins the node in a daemon thread (Matplotlib owns the main thread), builds
# the window and runs a 10 Hz animation until the window is closed.
# UI ELEMENTS CREATED HERE:
#   fig / ax    : window 'LIMO Live Tracking (Hardware)', canvas = lab_x x lab_y
#                 with the projector style (red axes, 1 m grid, big labels)
#   ref_line    : green dashed 'Reference Path'
#   trail_line  : orange 'Live Trail'
#   robot       : RobotGlyph (body, M1 -> M0 heading arrow, red M0 / blue M1)
#   obstacles   : red circles / dashed walls from /hardware_obstacles
#   status      : top-left text with pose, heading and frame
# =============================================================================
def main(args=None):
    init_gui_ros(args)  # Ctrl-C closes the window; cleanup below still runs
    node = LivePlotterNode()
    spin_thread = start_spin_thread(node)

    fig, ax = plt.subplots(figsize=(11, 9))
    style_lab_axes(ax, node.lab_x, node.lab_y, 'LIMO Live Tracking (Hardware)')
    ref_line, = ax.plot([], [], 'g--', lw=2, label='Reference Path')
    trail_line, = ax.plot([], [], '-', color='darkorange', lw=2, label='Live Trail')
    robot = RobotGlyph(ax)
    status = ax.text(0.01, 0.99, 'waiting for /hardware_odom ...', transform=ax.transAxes,
                     va='top', fontsize=12, family='monospace',
                     bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'))
    overlay = {'patches': [], 'version': -1}
    ax.legend(loc='upper right')

    # =========================================================================
    # [UI]     update_plot(frame_idx)  (nested)
    # -------------------------------------------------------------------------
    # Animation callback: path, trail, robot glyph, obstacles (only when a new
    # map arrived) and the status text.
    # =========================================================================
    def update_plot(_frame_idx):
        ref_line.set_data(node.ref_x, node.ref_y)
        trail_line.set_data(node.trail_x, node.trail_y)
        if node.pose is not None:
            x, y, yaw = node.pose
            robot.update(x, y, yaw)
            mode = 'start-pose frame' if node.frame.rezero else 'PhaseSpace frame'
            status.set_text(f'robot ({x:+.2f}, {y:+.2f}) heading {math.degrees(yaw):+.0f} deg  [{mode}]')
        if node.frame.ready and overlay['version'] != node.obstacles_version:
            overlay['patches'] = draw_obstacles(ax, node.obstacles, node.frame, overlay['patches'])
            overlay['version'] = node.obstacles_version
            ax.legend(loc='upper right')

    ani = animation.FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)  # noqa: F841

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
