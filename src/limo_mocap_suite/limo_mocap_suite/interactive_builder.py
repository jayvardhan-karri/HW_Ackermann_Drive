# #############################################################################
# [FILE]   interactive_builder.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/interactive_builder.py
# ROLE  : Matplotlib GUI to hand-draw a path and publish it to ROS 2.
#
# Console script : ros2 run limo_mocap_suite interactive_builder
# Publishes      : /reference_path (nav_msgs/Path, frame 'odom')
# Subscribes     : /hardware_odom (robot glyph), /hardware_obstacles (latched map)
# Parameters     : rezero_odom (False): draw in the same frame as the controllers
#                  lab_x, lab_y ([-0.5, 8.0], [-1.0, 6.0]): canvas = lab floor
#                  view ('behind'): canvas orientation, same as the other GUIs
# The canvas may be rotated, but /reference_path always carries real X, Y.
# Mostly [UI]. The only ROS logic is in TrajectoryPublisherNode.
# Robot glyph: arrow from rear marker M1 (blue) to front marker M0 (red).
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
import numpy as np
from scipy.interpolate import splprep, splev
import tkinter as tk

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from limo_mocap_suite.plot_helpers import (
    StartFrame, RobotGlyph, draw_obstacles, quat_to_yaw, declare_lab_params, declare_view_param,
    style_lab_axes, to_view, from_view, lab_legend, init_gui_ros, start_spin_thread)

# =============================================================================
# [ROS-IO] class TrajectoryPublisherNode(Node)
# -----------------------------------------------------------------------------
# rclpy node that owns the /reference_path publisher and caches the robot
# pose and obstacle map for the canvas overlay.
# =============================================================================
class TrajectoryPublisherNode(Node):
    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Creates node 'interactive_builder_node', a Path publisher on
    # /reference_path (queue depth 10), the rezero_odom and lab_x / lab_y
    # parameters, and subscriptions to /hardware_odom and the latched
    # /hardware_obstacles.
    # =========================================================================
    def __init__(self):
        super().__init__('interactive_builder_node')
        self.path_pub = self.create_publisher(Path, '/reference_path', 10)
        self.declare_parameter('rezero_odom', False)
        self.lab_x, self.lab_y = declare_lab_params(self)
        self.behind_view = declare_view_param(self)
        self.frame = StartFrame(bool(self.get_parameter('rezero_odom').value))
        self.robot_pose = None      # latest (x, y, yaw) in the PhaseSpace frame
        self.obstacles = []         # flat [x, y, r, ...]
        self.obstacles_version = 0  # bumped on every new map
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/hardware_odom', self.odom_cb, 10)
        self.create_subscription(Float32MultiArray, '/hardware_obstacles', self.obstacles_cb, latched)

    # =========================================================================
    # [ROS-IO] odom_cb(msg: Odometry)
    # -------------------------------------------------------------------------
    # Caches the latest robot pose and latches the drawing frame on the first
    # message (same rule as the controllers, so start them together).
    # =========================================================================
    def odom_cb(self, msg):
        p = msg.pose.pose
        self.robot_pose = (p.position.x, p.position.y, quat_to_yaw(p.orientation))
        self.frame.latch(*self.robot_pose)

    # =========================================================================
    # [ROS-IO] obstacles_cb(msg: Float32MultiArray)
    # -------------------------------------------------------------------------
    # Caches the bridge's obstacle map (flat [x, y, r, ...]).
    # =========================================================================
    def obstacles_cb(self, msg):
        self.obstacles = list(msg.data)
        self.obstacles_version += 1

    # =========================================================================
    # [ROS-IO] publish_trajectory(x_pts, y_pts)
    # -------------------------------------------------------------------------
    # Packs points in real X, Y (not canvas coordinates: see publish_data) into
    # one nav_msgs/Path and publishes it.
    # For each point i: position = (x_i, y_i); yaw = direction to point i+1
    # (last point re-uses direction from i-1); yaw stored as a z-axis
    #   quaternion.
    # =========================================================================
    def publish_trajectory(self, x_pts, y_pts):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(x_pts)):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x_pts[i])
            ps.pose.position.y = float(y_pts[i])
            if i < len(x_pts) - 1:
                yaw = math.atan2(y_pts[i+1] - y_pts[i], x_pts[i+1] - x_pts[i])
            else:
                yaw = math.atan2(y_pts[i] - y_pts[i-1], x_pts[i] - x_pts[i-1]) if i > 0 else 0.0
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(ps)
        self.path_pub.publish(msg)
        self.get_logger().info(f"Published trajectory with {len(x_pts)} waypoints.")

# =============================================================================
# [UI]     get_dynamic_figsize(ratio=0.80)
# -----------------------------------------------------------------------------
# Opens a hidden Tk root just to read the monitor resolution and returns a
# Matplotlib figsize (inches, assuming 100 dpi) = ratio * screen size.
# Falls back to (12, 8) if Tk is unavailable (e.g. no display).
# =============================================================================
def get_dynamic_figsize(ratio=0.80):
    try:
        root = tk.Tk()
        root.withdraw()
        w, h = root.winfo_screenwidth(), root.winfo_screenheight()
        root.destroy()
        return ((w * ratio) / 100.0, (h * ratio) / 100.0)
    except:
        return (12, 8)

# =============================================================================
# [LIFE]   main(args=None)
# -----------------------------------------------------------------------------
# Entry point. Starts rclpy, spins the node in a daemon thread (so Matplotlib
# owns the main thread), builds the GUI below and blocks in plt.show().
# UI ELEMENTS CREATED HERE:
#   fig / ax      : window titled 'Interactive Trajectory Builder', canvas =
#                   lab_x x lab_y with the projector style (red axes, 1 m grid)
#   line          : 'Drawn Path' curve (blue while raw, green after smoothing)
#   btn_smooth    : button 'Smooth Path' -> apply_smoothing
#   btn_clear     : button 'Clear'       -> clear_canvas
#   btn_pub       : button 'Publish'     -> publish_data
#   mouse events  : press -> on_press, drag -> on_motion, release -> on_release
#   robot         : RobotGlyph (body, M1 -> M0 heading arrow, red M0 / blue M1 dots)
#   obstacles     : red circles / dashed walls from /hardware_obstacles
#   status        : top-left text with robot pose, heading and frame
#   overlay_timer : 5 Hz timer -> refresh_overlay
# =============================================================================
def main(args=None):
    init_gui_ros(args)  # Ctrl-C closes the window; cleanup below still runs
    ros_node = TrajectoryPublisherNode()
    spin_thread = start_spin_thread(ros_node)

    behind = ros_node.behind_view
    raw_x, raw_y = [], []   # stroke in CANVAS coordinates; from_view() before publishing
    drawing = False
    
    fig, ax = plt.subplots(figsize=get_dynamic_figsize())
    plt.subplots_adjust(bottom=0.15)
    style_lab_axes(ax, ros_node.lab_x, ros_node.lab_y, 'Interactive Trajectory Builder',
                   behind_view=behind)
    x_min, x_max = ax.get_xlim()
    y_min, y_max = ax.get_ylim()

    line, = ax.plot([], [], 'b-', lw=2.5, label="Drawn Path")
    robot = RobotGlyph(ax)
    status = ax.text(0.01, 0.99, 'waiting for /hardware_odom ...', transform=ax.transAxes,
                     va='top', fontsize=12, family='monospace',
                     bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'))
    overlay = {'patches': [], 'version': -1}
    lab_legend(ax, behind)

    # =========================================================================
    # [UI]     on_press(event)
    # -------------------------------------------------------------------------
    # Mouse-button-down on the canvas: start a NEW stroke (clears old points)
    # and record the first point, clipped to the canvas limits.
    # =========================================================================
    def on_press(event):
        nonlocal drawing
        if event.inaxes != ax: return
        drawing = True
        raw_x.clear(); raw_y.clear()
        raw_x.append(np.clip(event.xdata, x_min, x_max))
        raw_y.append(np.clip(event.ydata, y_min, y_max))

    # =========================================================================
    # [UI]     on_motion(event)
    # -------------------------------------------------------------------------
    # Mouse drag while drawing: append the clipped cursor point and redraw
    #   line.
    # =========================================================================
    def on_motion(event):
        if not drawing or event.inaxes != ax: return
        raw_x.append(np.clip(event.xdata, x_min, x_max))
        raw_y.append(np.clip(event.ydata, y_min, y_max))
        line.set_data(raw_x, raw_y)
        fig.canvas.draw_idle()

    # =========================================================================
    # [UI]     on_release(event)
    # -------------------------------------------------------------------------
    # Mouse-button-up: stop recording points.
    # =========================================================================
    def on_release(event):
        nonlocal drawing
        drawing = False

    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('button_release_event', on_release)

    # =========================================================================
    # [UI]     refresh_overlay()  <- 5 Hz canvas timer
    # -------------------------------------------------------------------------
    # Redraws the robot glyph (M1 -> M0 arrow) and, when a new map arrived,
    # the obstacles, both in the controller frame. Updates the status text.
    # =========================================================================
    def refresh_overlay():
        frame = ros_node.frame
        if ros_node.robot_pose is None or not frame.ready:
            return
        x, y, yaw = frame.apply(*ros_node.robot_pose)
        robot.update(*to_view(x, y, yaw, behind))
        if overlay['version'] != ros_node.obstacles_version:
            overlay['patches'] = draw_obstacles(ax, ros_node.obstacles, frame, overlay['patches'], behind)
            overlay['version'] = ros_node.obstacles_version
            lab_legend(ax, behind)
        mode = 'start-pose frame' if frame.rezero else 'PhaseSpace frame'
        status.set_text(f'robot ({x:+.2f}, {y:+.2f}) heading {math.degrees(yaw):+.0f} deg  [{mode}]')
        fig.canvas.draw_idle()

    overlay_timer = fig.canvas.new_timer(interval=200)
    overlay_timer.add_callback(refresh_overlay)
    overlay_timer.start()

    # Clean, relative button placement
    # =========================================================================
    # [UI]     Button widgets: 'Smooth Path' | 'Clear' | 'Publish'
    # -------------------------------------------------------------------------
    # Three buttons along the bottom of the window (figure-relative axes).
    # =========================================================================
    btn_smooth = Button(plt.axes([0.30, 0.05, 0.1, 0.05]), 'Smooth Path')
    btn_clear  = Button(plt.axes([0.45, 0.05, 0.1, 0.05]), 'Clear')
    btn_pub    = Button(plt.axes([0.60, 0.05, 0.1, 0.05]), 'Publish')

    # =========================================================================
    # [UI]     apply_smoothing(event)  <- 'Smooth Path' button
    # -------------------------------------------------------------------------
    # 1. Needs >= 4 points. Drops consecutive points closer than 1 mm
    #    (duplicates break splprep).
    # 2. Fits a parametric smoothing B-spline (scipy splprep, s=0.08).
    # 3. Resamples it at max(300, n_raw) evenly spaced parameter values,
    #    clips to canvas, replaces the stroke and recolours the line green.
    # =========================================================================
    def apply_smoothing(event):
        if len(raw_x) < 4: return
        # Filter duplicate points to prevent splprep math errors
        fx, fy = [raw_x[0]], [raw_y[0]]
        for cx, cy in zip(raw_x[1:], raw_y[1:]):
            if math.hypot(cx - fx[-1], cy - fy[-1]) > 1e-3:
                fx.append(cx)
                fy.append(cy)
        if len(fx) < 4:
            print("[WARN] Not enough unique points to smooth.")
            return
        tck, u = splprep([fx, fy], s=0.08)
        out = splev(np.linspace(0, 1.0, max(300, len(raw_x))), tck)
        raw_x[:], raw_y[:] = np.clip(out[0], x_min, x_max), np.clip(out[1], y_min, y_max)
        line.set_data(raw_x, raw_y)
        line.set_color('green')
        fig.canvas.draw_idle()

    # =========================================================================
    # [UI]     clear_canvas(event)  <- 'Clear' button
    # -------------------------------------------------------------------------
    # Erases the stroke and resets the line colour to blue.
    # =========================================================================
    def clear_canvas(event):
        raw_x.clear(); raw_y.clear()
        line.set_data([], [])
        line.set_color('blue')
        fig.canvas.draw_idle()

    # =========================================================================
    # [UI]     publish_data(event)  <- 'Publish' button
    # -------------------------------------------------------------------------
    # If a stroke exists, converts it from canvas coordinates to real X, Y and
    # hands it to ros_node.publish_trajectory().
    # =========================================================================
    def publish_data(event):
        if raw_x: ros_node.publish_trajectory(*from_view(np.array(raw_x), np.array(raw_y), behind))

    btn_smooth.on_clicked(apply_smoothing)
    btn_clear.on_clicked(clear_canvas)
    btn_pub.on_clicked(publish_data)

    try: plt.show()
    except KeyboardInterrupt: pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)

if __name__ == '__main__': main()
