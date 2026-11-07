# #############################################################################
# [FILE]   live_plotter_node.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/live_plotter_node.py
# ROLE  : Live off-board view of a hardware run: reference path, robot trail,
#         robot glyph with heading, and the bridge's obstacle map.
#
# Console script : ros2 run limo_mocap_suite live_plotter
# Subscribes     : /reference_path (Path), /hardware_odom (Odometry),
#                  /hardware_obstacles (Float32MultiArray, latched), /run_status
# Publishes      : /run_kill (Bool, latched), /cmd_vel (zero only, on KILL)
# KILL button    : every controller stops and refuses to move until RE-ARM. It relies
#                  on this window and ROS running; it is not a hardware stop.
# Parameters     : rezero_odom (False): must match the controller's value
#                  lab_x, lab_y ([-0.5, 7.0], [-0.5, 3.0]): canvas = lab floor
#                  view ('behind'): canvas orientation, same as the other GUIs
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

import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32MultiArray, String

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

from limo_mocap_suite.plot_helpers import (
    StartFrame, RobotGlyph, draw_obstacles, quat_to_yaw, declare_lab_params, declare_view_param,
    style_lab_axes, to_view, lab_legend, init_gui_ros, start_spin_thread)


# =============================================================================
# [ROS-IO] class LivePlotterNode(Node)
# -----------------------------------------------------------------------------
# Caches path, trail, current pose and obstacle map for the animation.
# =============================================================================
class LivePlotterNode(Node):
    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Reads rezero_odom, lab_x / lab_y and view, creates the StartFrame (maps the
    # mocap pose into the controller's frame) and the buffers. Subscribes to
    # /reference_path, /hardware_odom, /hardware_obstacles (latched QoS) and
    # /run_status; publishes /run_kill (latched) and /cmd_vel. A 10 Hz timer
    # (kill_tick) repeats the zero command while KILL is on.
    # =========================================================================
    def __init__(self):
        super().__init__('live_plotter_node')
        self.declare_parameter('rezero_odom', False)
        self.frame = StartFrame(bool(self.get_parameter('rezero_odom').value))
        self.lab_x, self.lab_y = declare_lab_params(self)
        self.behind_view = declare_view_param(self)

        # Data for the animation, written by the ROS thread, read by the GUI thread
        self.ref_x, self.ref_y = [], []
        self.trail_x, self.trail_y = [], []
        self.pose = None            # (x, y, yaw) in the controller frame
        self.obstacles = []
        self.obstacles_version = 0    # +1 per new map, so the GUI redraws only then

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/reference_path', self.path_cb, 10)
        self.create_subscription(Odometry, '/hardware_odom', self.odom_cb, 10)
        self.create_subscription(Float32MultiArray, '/hardware_obstacles', self.obstacles_cb, latched)
        self.status = None
        self.status_time = 0.0
        self.hist = deque(maxlen=300)   # (t, cte, steer, steer_ff) from /run_status: last 60 s
        self.killed = False
        self.kill_pub = self.create_publisher(Bool, '/run_kill', latched)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/run_status', self.status_cb, 10)
        self.create_timer(0.1, self.kill_tick)
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
    # Caches the bridge's obstacle map (flat [x, y, r, ...], m).
    # =========================================================================
    def obstacles_cb(self, msg):
        self.obstacles = list(msg.data)
        self.obstacles_version += 1

    # =========================================================================
    # [ROS-IO] status_cb(msg: String)
    # -------------------------------------------------------------------------
    # Latest controller report (JSON) and the history for the two strip charts.
    # Only the APPROACH and TRACK phases add a point (cte m, steer rad, steer_ff rad).
    # =========================================================================
    def status_cb(self, msg):
        self.status = json.loads(msg.data)
        self.status_time = time.monotonic()
        if self.status['phase'] in ('APPROACH', 'TRACK'):
            s = self.status
            self.hist.append((self.status_time, s['cte'], s['steer'], s['steer_ff']))

    # =========================================================================
    # [ROS-IO] set_kill(on: bool) / kill_tick()
    # -------------------------------------------------------------------------
    # KILL latches /run_kill (a controller started later sees it too) and keeps
    # sending zero commands, in case a controller is not listening. RE-ARM clears it.
    # =========================================================================
    def set_kill(self, on):
        self.killed = on
        self.kill_pub.publish(Bool(data=on))
        if on:
            self.cmd_pub.publish(Twist())
            self.get_logger().error('KILL pressed')

    # 10 Hz timer: repeats the zero command while KILL is on.
    def kill_tick(self):
        if self.killed:
            self.cmd_pub.publish(Twist())


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
#   ax_cte      : strip chart, error with the guard's slow / rejoin levels
#   ax_steer    : strip chart, steering sent, what the geometry needs, the limit
#   info        : controller, phase, lap, guard, steering use
#   btn_kill    : KILL / RE-ARM button
# =============================================================================
def main(args=None):
    init_gui_ros(args)  # Ctrl-C closes the window; cleanup below still runs
    node = LivePlotterNode()
    spin_thread = start_spin_thread(node)

    fig = plt.figure(figsize=(15, 9))
    ax = fig.add_axes([0.05, 0.07, 0.56, 0.87])
    ax_cte = fig.add_axes([0.67, 0.68, 0.30, 0.25])
    ax_steer = fig.add_axes([0.67, 0.36, 0.30, 0.25])
    ax_info = fig.add_axes([0.65, 0.14, 0.33, 0.18])
    ax_info.axis('off')
    info = ax_info.text(0.0, 1.0, 'no controller yet', va='top', fontsize=13, family='monospace')
    btn_kill = Button(fig.add_axes([0.67, 0.02, 0.30, 0.10]), 'KILL', color='#e53935', hovercolor='#ff5252')
    btn_kill.label.set_fontsize(24)
    btn_kill.label.set_fontweight('bold')
    btn_kill.label.set_color('white')

    # =========================================================================
    # [UI]     on_kill(_event)  (nested)
    # -------------------------------------------------------------------------
    # KILL button click: toggles KILL / RE-ARM, and the button label and colour.
    # =========================================================================
    def on_kill(_event):
        node.set_kill(not node.killed)
        btn_kill.label.set_text('RE-ARM' if node.killed else 'KILL')
        btn_kill.color = '#43a047' if node.killed else '#e53935'
        btn_kill.ax.set_facecolor(btn_kill.color)
        fig.canvas.draw_idle()

    btn_kill.on_clicked(on_kill)
    cte_line, = ax_cte.plot([], [], color='#2f6db5', lw=1.3)
    ax_cte.set_title('Error (cm); with guard on: dashed = slows, solid red = rejoins', fontsize=10)
    ax_cte.grid(alpha=0.4)
    steer_line, = ax_steer.plot([], [], color='#2f6db5', lw=1.3, label='steering sent')
    ff_line, = ax_steer.plot([], [], 'k--', lw=1.1, label='needed by the geometry')
    ax_steer.set_title('Steering (deg); red = limit', fontsize=10)
    ax_steer.grid(alpha=0.4)
    ax_steer.legend(fontsize=8, loc='upper left')
    behind = node.behind_view
    style_lab_axes(ax, node.lab_x, node.lab_y, 'LIMO Live Tracking (Hardware)', behind_view=behind)
    ref_line, = ax.plot([], [], 'g--', lw=2, label='Reference Path')
    trail_line, = ax.plot([], [], '-', color='darkorange', lw=2, label='Live Trail')
    robot = RobotGlyph(ax)
    status = ax.text(0.01, 0.99, 'waiting for /hardware_odom ...', transform=ax.transAxes,
                     va='top', fontsize=12, family='monospace',
                     bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'))
    overlay = {'patches': [], 'version': -1}
    lab_legend(ax, behind)

    # =========================================================================
    # [UI]     view_xy(xs, ys) -> (u, v)  (nested)
    # -------------------------------------------------------------------------
    # Lists of real X, Y to canvas coordinates. The ROS thread may be appending,
    # so both are cut to the shorter length.
    # =========================================================================
    def view_xy(xs, ys):
        n = min(len(xs), len(ys))
        return to_view(np.array(xs[:n]), np.array(ys[:n]), behind_view=behind)[:2]

    # =========================================================================
    # [UI]     update_side(now)  (nested)
    # -------------------------------------------------------------------------
    # Right-hand panel: the two strip charts (last 60 s; now = time.monotonic())
    # and the info text. Text turns red when no report arrives for 1.5 s, on
    # KILL / ABORT, or when the guard is in REJOIN / HOLD.
    # =========================================================================
    def update_side(now):
        st = node.status
        hist = list(node.hist)
        if hist:
            h = np.array(hist)
            t = h[:, 0] - now
            cte_line.set_data(t, 100.0 * h[:, 1])
            steer_line.set_data(t, np.degrees(h[:, 2]))
            ff_line.set_data(t, np.degrees(h[:, 3]))
            ax_cte.set_xlim(-60.0, 0.0)
            ax_steer.set_xlim(-60.0, 0.0)
            top = max(40.0, 1.2 * float(np.max(np.abs(100.0 * h[:, 1]))))
            ax_cte.set_ylim(-top, top)
        if st is None:
            return
        limit = math.degrees(st['max_steer'])
        ax_steer.set_ylim(-1.15 * limit, 1.15 * limit)
        if not getattr(ax_steer, 'limit_drawn', False):    # limit lines are drawn once
            ax_steer.axhline(limit, color='#c62828', lw=1.5)
            ax_steer.axhline(-limit, color='#c62828', lw=1.5)
            ax_steer.limit_drawn = True
            for level, style in (((st['slow_cte'], '--'), (st['rejoin_cte'], '-')) if st['guard_on'] else ()):
                ax_cte.axhline(100.0 * level, color='#c62828', ls=style, lw=1.0)
                ax_cte.axhline(-100.0 * level, color='#c62828', ls=style, lw=1.0)
        silent = now - node.status_time > 1.5
        use = 100.0 * abs(st['steer']) / st['max_steer']
        need = 100.0 * abs(st['steer_ff']) / st['max_steer']
        rms = '-' if st['rms_last'] is None else f"{100.0 * st['rms_last']:.1f} cm"
        info.set_text(
            f"{st['controller'].upper()}  {'NO REPORT' if silent else st['phase']}\n"
            f"lap     : {st['lap']} / {st['laps']}   last RMS {rms}\n"
            f"error   : {100.0 * st['cte']:+.1f} cm   guard {st['guard'] or '-'}\n"
            f"steering: using {use:.0f}% of limit, curve needs {need:.0f}%\n"
            f"{(st['result'] or '') + ': ' + st['reason'] if st['result'] else ''}")
        bad = silent or st['phase'] == 'KILLED' or st['result'] in ('ABORTED', 'KILLED') or st['guard'] in ('REJOIN', 'HOLD')
        info.set_color('#c62828' if bad else 'black')

    # =========================================================================
    # [UI]     update_plot(_frame_idx)  (nested)
    # -------------------------------------------------------------------------
    # Animation callback, 10 Hz: side panel, path, trail, robot glyph, obstacles
    # (only when a new map arrived) and the status text.
    # =========================================================================
    def update_plot(_frame_idx):
        update_side(time.monotonic())
        ref_line.set_data(*view_xy(node.ref_x, node.ref_y))
        trail_line.set_data(*view_xy(node.trail_x, node.trail_y))
        if node.pose is not None:
            x, y, yaw = node.pose
            robot.update(*to_view(x, y, yaw, behind))
            mode = 'start-pose frame' if node.frame.rezero else 'PhaseSpace frame'
            status.set_text(f'robot ({x:+.2f}, {y:+.2f}) heading {math.degrees(yaw):+.0f} deg  [{mode}]')
        if node.frame.ready and overlay['version'] != node.obstacles_version:
            overlay['patches'] = draw_obstacles(ax, node.obstacles, node.frame, overlay['patches'], behind)
            overlay['version'] = node.obstacles_version
            lab_legend(ax, behind)

    # Keep a reference to ani, or Matplotlib stops the animation
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
