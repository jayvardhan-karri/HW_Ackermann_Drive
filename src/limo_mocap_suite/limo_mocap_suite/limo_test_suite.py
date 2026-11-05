# #############################################################################
# [FILE]   limo_test_suite.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/limo_test_suite.py
# ROLE  : Open-loop actuator test for the LIMO (wheels lifted, or on the floor
#         with NO feedback). Plays a test path as /cmd_vel commands and shows,
#         seen from BEHIND the robot, where the robot should be and how its
#         front wheels should be steered, so you can compare with the real LIMO.
#
# Console script : ros2 run limo_mocap_suite limo_test_suite
# Publishes      : /cmd_vel (geometry_msgs/Twist)  <- the only output
# Subscribes     : /reference_path (nav_msgs/Path), only for the 'drawn' path
# No odometry / MoCap is used: every command comes from the path geometry.
#
# Test paths (robot start frame: +X forward, +Y left):
#   straight : straight_length metres forward               (steering 0)
#   circle   : one full LEFT circle of radius `radius`       (steer left)
#   figure8  : full LEFT circle, then full RIGHT circle      (left, then right)
#   drawn    : the last path received on /reference_path (e.g. from the builder);
#              commands use its own start frame (starts at the robot, heading forward)
# Display: built-in paths are centred on the lab grid by default (auto_place), or
#          drawn from (start_x, start_y). A drawn path stays where it was drawn, so
#          it matches the builder. Display only; it never changes the commands.
# View   : parameter 'view' ('behind' by default), the same in all three GUIs.
# Commands: v = speed, delta = atan(L * kappa(s)) clipped to max_steer,
#           omega = v * tan(delta) / L, with s = speed * t (arc length).
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
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from matplotlib.widgets import Button, RadioButtons

from limo_mocap_suite.plot_helpers import (
    RobotGlyph, declare_lab_params, declare_view_param, style_lab_axes, to_view, BODY_LENGTH, BODY_WIDTH,
    init_gui_ros, start_spin_thread)

PATH_NAMES = ('straight', 'circle', 'figure8', 'drawn')
DS = 0.01  # path sampling step (m)


# =============================================================================
# [CORE]   path_from_segments(segments) -> dict
# -----------------------------------------------------------------------------
# Builds a path from constant-curvature pieces [(length, kappa), ...] by
# integrating heading and position every DS metres from the origin, heading
# +X. Returns dict with arrays s, x, y, yaw, kappa (all the same length).
# =============================================================================
def path_from_segments(segments):
    kappa = np.concatenate([np.full(max(1, int(round(length / DS))), k) for length, k in segments])
    s = np.arange(len(kappa) + 1) * DS
    yaw = np.concatenate(([0.0], np.cumsum(kappa * DS)))
    x = np.concatenate(([0.0], np.cumsum(np.cos(yaw[:-1] + 0.5 * kappa * DS) * DS)))
    y = np.concatenate(([0.0], np.cumsum(np.sin(yaw[:-1] + 0.5 * kappa * DS) * DS)))
    kappa = np.concatenate((kappa, kappa[-1:]))
    return {'s': s, 'x': x, 'y': y, 'yaw': yaw, 'kappa': kappa}


# =============================================================================
# [CORE]   path_from_points(px, py) -> dict | None
# -----------------------------------------------------------------------------
# Converts a drawn path into the same format:
# 1. drop points closer than 1 cm, need >= 4 left;
# 2. move it into its own start frame (first point = origin, first segment
#    heading = +X) so the robot starts at the origin heading forward;
# 3. resample every DS metres of arc length;
# 4. heading and signed curvature by finite differences (lightly smoothed);
# 5. keep the start frame in 'origin' (x0, y0, yaw0), to display it as drawn.
# =============================================================================
def path_from_points(px, py):
    pts = [(px[0], py[0])]
    for x, y in zip(px[1:], py[1:]):
        if math.hypot(x - pts[-1][0], y - pts[-1][1]) > 0.01:
            pts.append((x, y))
    if len(pts) < 4:
        return None
    p = np.array(pts)
    yaw0 = math.atan2(p[1, 1] - p[0, 1], p[1, 0] - p[0, 0])
    c, sn = math.cos(yaw0), math.sin(yaw0)
    d = p - p[0]
    local = np.column_stack((c * d[:, 0] + sn * d[:, 1], -sn * d[:, 0] + c * d[:, 1]))
    seg = np.hypot(np.diff(local[:, 0]), np.diff(local[:, 1]))
    s_raw = np.concatenate(([0.0], np.cumsum(seg)))
    s = np.arange(0.0, s_raw[-1], DS)
    x = np.interp(s, s_raw, local[:, 0])
    y = np.interp(s, s_raw, local[:, 1])
    dx, dy = np.gradient(x), np.gradient(y)
    yaw = np.unwrap(np.arctan2(dy, dx))
    kappa = np.gradient(yaw) / DS
    kernel = np.ones(9) / 9.0                      # 9 cm moving average
    kappa = np.convolve(kappa, kernel, mode='same')
    return {'s': s, 'x': x, 'y': y, 'yaw': yaw, 'kappa': kappa,
            'origin': (float(p[0, 0]), float(p[0, 1]), yaw0)}


# =============================================================================
# [ROS-IO] class LimoTestSuiteNode(Node)
# -----------------------------------------------------------------------------
# Holds the selected test path and plays it open loop on /cmd_vel at 50 Hz.
# Phases: IDLE -> COUNTDOWN (zeros for start_delay s) -> RUNNING -> DONE.
# Also dead-reckons the EXPECTED pose from the commands it sends (what the
# robot would do on the floor), for the GUI.
# =============================================================================
class LimoTestSuiteNode(Node):
    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Declares parameters, builds the built-in paths, creates the /cmd_vel
    # publisher, the /reference_path subscription and the 50 Hz timer.
    # =========================================================================
    def __init__(self):
        super().__init__('limo_test_suite')
        self.declare_parameter('path', 'figure8')
        self.declare_parameter('speed', 0.20)
        self.declare_parameter('wheelbase', 0.20)
        self.declare_parameter('max_steer', 0.4084)  # centre angle at the driver's 28 deg inner-wheel limit (R 0.46 m)
        self.declare_parameter('radius', 0.80)
        self.declare_parameter('straight_length', 2.0)
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('repeat', False)
        self.declare_parameter('autostart', False)
        self.declare_parameter('auto_place', True)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.lab_x, self.lab_y = declare_lab_params(self)
        self.behind_view = declare_view_param(self)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.speed = float(g('speed'))
        self.L = float(g('wheelbase'))
        self.max_steer = float(g('max_steer'))
        self.start_delay = float(g('start_delay'))
        self.repeat = bool(g('repeat'))
        self.auto_place = bool(g('auto_place'))
        self.start_xy = (float(g('start_x')), float(g('start_y')))
        r, length = float(g('radius')), float(g('straight_length'))
        circle = 2.0 * math.pi * r
        self.paths = {
            'straight': path_from_segments([(length, 0.0)]),
            'circle': path_from_segments([(circle, 1.0 / r)]),
            'figure8': path_from_segments([(circle, 1.0 / r), (circle, -1.0 / r)]),
            'drawn': None,
        }

        self.lock = threading.Lock()
        self.phase = 'IDLE'
        self.name = None
        self.path = None
        self.t_phase = 0.0
        self.v = self.delta = self.omega = 0.0
        self.select(str(g('path')))

        self.cmd_pub = self.create_publisher(Twist, '/limo1/cmd_vel', 1) # updated from /cmd_vel to /limo1/cmd_vel as Limo takes Velocity commands via this topic.
        self.create_subscription(Path, '/reference_path', self.path_cb, 10)
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info(
            f'LIMO test suite ready | path={self.name} speed={self.speed} m/s | '
            f'press Start (or autostart:=true)')
        if bool(g('autostart')):
            self.start()

    # =========================================================================
    # [ROS-IO] path_cb(msg: Path)
    # -------------------------------------------------------------------------
    # Stores a received /reference_path as the 'drawn' test path.
    # =========================================================================
    def path_cb(self, msg):
        p = path_from_points([q.pose.position.x for q in msg.poses],
                             [q.pose.position.y for q in msg.poses])
        if p is None:
            self.get_logger().warn('Received /reference_path with < 4 distinct points; ignored')
            return
        with self.lock:
            self.paths['drawn'] = p
            if self.name == 'drawn' and self.phase in ('IDLE', 'DONE'):
                self.path = p
                self.reset_expected()
        self.get_logger().info(f"Stored drawn path: {p['s'][-1]:.2f} m")

    # =========================================================================
    # [UTIL]   reset_expected()
    # -------------------------------------------------------------------------
    # Puts the expected robot back at the path start and clears its trail.
    # =========================================================================
    def reset_expected(self):
        self.ex, self.ey, self.eth = 0.0, 0.0, 0.0
        self.trail = [(0.0, 0.0)]
        self.v = self.delta = self.omega = 0.0

    # =========================================================================
    # [LIFE]   select(name) / start() / stop()
    # -------------------------------------------------------------------------
    # select: choose a test path (only while not running).
    # start : begin the countdown (needs a path; 'drawn' needs a received one).
    # stop  : abort, publish one zero Twist, back to IDLE.
    # =========================================================================
    def select(self, name):
        with self.lock:
            if name not in PATH_NAMES:
                self.get_logger().warn(f"Unknown path '{name}', using figure8")
                name = 'figure8'
            if self.phase in ('COUNTDOWN', 'RUNNING'):
                return
            self.name = name
            self.path = self.paths[name]
            self.phase = 'IDLE'
            self.reset_expected()

    def start(self):
        with self.lock:
            if self.path is None or self.phase in ('COUNTDOWN', 'RUNNING'):
                return
            self.reset_expected()
            self.phase = 'COUNTDOWN'
            self.t_phase = self.now()
        self.get_logger().info(f'Starting {self.name} in {self.start_delay:.0f} s')

    def stop(self):
        with self.lock:
            self.phase = 'IDLE'
            self.reset_expected()
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Stopped (zero velocity sent)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # =========================================================================
    # [UTIL]   placement(path) -> (start_x, start_y)
    # -------------------------------------------------------------------------
    # Where the path start is drawn on the lab grid. auto_place: shift the
    # path so its bounding box is centred on the lab extents; otherwise the
    # start_x / start_y parameters (robot assumed to face +X there).
    # Display only: the commands never depend on it.
    # =========================================================================
    def placement(self, path):
        if not self.auto_place or path is None:
            return self.start_xy
        cx = 0.5 * (self.lab_x[0] + self.lab_x[1]) - 0.5 * (path['x'].min() + path['x'].max())
        cy = 0.5 * (self.lab_y[0] + self.lab_y[1]) - 0.5 * (path['y'].min() + path['y'].max())
        return cx, cy

    def duration(self):
        return self.path['s'][-1] / self.speed if self.path is not None else 0.0

    # =========================================================================
    # [CORE]   tick()  <- 50 Hz timer
    # -------------------------------------------------------------------------
    # COUNTDOWN: publish zeros until start_delay has passed, then RUNNING.
    # RUNNING  : s = speed * t; kappa = interp(s); delta = clip(atan(L kappa));
    #            omega = v tan(delta) / L; publish; integrate the expected pose
    #            with the same (v, delta). At the end of the path: zero Twist,
    #            then DONE (or restart if repeat).
    # IDLE/DONE: publish nothing (does not fight other nodes on /cmd_vel).
    # =========================================================================
    def tick(self):
        msg = None
        with self.lock:
            t = self.now() - self.t_phase
            if self.phase == 'COUNTDOWN':
                msg = Twist()
                if t >= self.start_delay:
                    self.phase = 'RUNNING'
                    self.t_phase = self.now()
                    self.t_last = 0.0
            elif self.phase == 'RUNNING':
                s = self.speed * t
                if s >= self.path['s'][-1]:
                    msg = Twist()
                    self.v = self.delta = self.omega = 0.0
                    if self.repeat:
                        self.reset_expected()
                        self.t_phase = self.now()
                        self.t_last = 0.0
                    else:
                        self.phase = 'DONE'
                        self.get_logger().info(f'{self.name} finished')
                else:
                    kappa = float(np.interp(s, self.path['s'], self.path['kappa']))
                    self.v = self.speed
                    self.delta = max(-self.max_steer, min(self.max_steer, math.atan(self.L * kappa)))
                    self.omega = self.v * math.tan(self.delta) / self.L
                    msg = Twist()
                    msg.linear.x, msg.angular.z = self.v, self.omega
                    # Expected pose on the floor (bicycle model, same commands)
                    dt = t - self.t_last
                    self.t_last = t
                    self.ex += self.v * math.cos(self.eth) * dt
                    self.ey += self.v * math.sin(self.eth) * dt
                    self.eth += self.omega * dt
                    lx, ly = self.trail[-1]
                    if math.hypot(self.ex - lx, self.ey - ly) > 0.01:
                        self.trail.append((self.ex, self.ey))
        if msg is not None:
            self.cmd_pub.publish(msg)

    # =========================================================================
    # [UTIL]   snapshot() -> dict
    # -------------------------------------------------------------------------
    # Thread-safe copy of everything the GUI draws.
    # =========================================================================
    def snapshot(self):
        with self.lock:
            t = self.now() - self.t_phase
            return {
                'phase': self.phase, 'name': self.name, 'path': self.path,
                'trail': list(self.trail), 'v': self.v, 'delta': self.delta,
                'omega': self.omega, 't': t, 'pose': (self.ex, self.ey, self.eth),
            }


# =============================================================================
# [UI]     class WheelPanel
# -----------------------------------------------------------------------------
# Top-down LIMO seen from behind (forward = up): chassis, rear wheels, and
# FRONT wheels rotated by the commanded steering angle (left = rotated to the
# left), plus a forward arrow scaled with the speed. Compare with the real
# front wheels.
# =============================================================================
class WheelPanel:
    def __init__(self, ax):
        self.ax = ax
        ax.set_xlim(-0.35, 0.35)
        ax.set_ylim(-0.32, 0.42)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title('Wheels (seen from behind)', fontsize=14, fontweight='bold')
        hl, hw = BODY_LENGTH / 2.0, BODY_WIDTH / 2.0
        ax.add_patch(patches.Rectangle((-hw, -hl), 2 * hw, 2 * hl, facecolor='0.85',
                                       edgecolor='0.3', lw=1.5))
        self.wheel_w, self.wheel_l = 0.05, 0.11
        self.axle_y = hl * 0.62
        self.front = []
        for sx in (-1, 1):
            cx = sx * (hw + 0.03)
            ax.add_patch(patches.Polygon(self._rect(cx, -self.axle_y, 0.0), closed=True,
                                         facecolor='black'))
            p = patches.Polygon(self._rect(cx, self.axle_y, 0.0), closed=True, facecolor='black')
            ax.add_patch(p)
            self.front.append((p, cx))
        ax.text(0, hl + 0.02, 'FRONT', ha='center', va='bottom', fontsize=11, fontweight='bold')
        ax.text(0, -hl - 0.02, 'REAR', ha='center', va='top', fontsize=11, fontweight='bold')
        self.arrow = patches.FancyArrowPatch((0, 0), (0, 0.001), arrowstyle='-|>',
                                             mutation_scale=25, lw=3, color='green')
        ax.add_patch(self.arrow)

    def _rect(self, cx, cy, ang):
        c, s = math.cos(ang), math.sin(ang)
        hw, hl = self.wheel_w / 2.0, self.wheel_l / 2.0
        return [(cx + c * x - s * y, cy + s * x + c * y)
                for x, y in ((-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl))]

    # =========================================================================
    # [UI]     update(delta, v, max_speed)
    # -------------------------------------------------------------------------
    # Rotates the front wheels by delta (positive = left = counter-clockwise
    # with forward up) and scales the green forward arrow with v.
    # =========================================================================
    def update(self, delta, v, max_speed):
        for p, cx in self.front:
            p.set_xy(self._rect(cx, self.axle_y, delta))
        length = 0.25 * (v / max_speed if max_speed > 0 else 0.0)
        self.arrow.set_positions((0, -0.05), (0, -0.05 + max(length, 0.001)))
        self.arrow.set_visible(v > 1e-4)


# =============================================================================
# [LIFE]   main(args=None)
# -----------------------------------------------------------------------------
# Spins the node in a daemon thread and builds the GUI; on exit always sends
# a zero Twist.
# UI ELEMENTS CREATED HERE:
#   ax_path    : behind view (+X forward UP, +Y left LEFT) on the lab grid
#                (red axes, 1 m grid): grey dashed test path, orange expected
#                trail, RobotGlyph whose arrow points along the last traced
#                segment (start = straight up), green dot = start, red x = end;
#                the path is centred on the lab grid (auto_place) or drawn at
#                start_x / start_y
#   ax_wheel   : WheelPanel (front-wheel steering + speed arrow)
#   info       : text: phase / countdown, path, time, speed, steering
#                (LEFT / RIGHT, deg), omega, SATURATED warning
#   radio      : RadioButtons straight | circle | figure8 | drawn
#   btn_start  : button 'Start'  -> node.start()
#   btn_stop   : button 'STOP'   -> node.stop()
#   ani        : FuncAnimation, 10 Hz -> update_plot
# =============================================================================
def main(args=None):
    init_gui_ros(args)  # Ctrl-C closes the window; cleanup below still runs
    node = LimoTestSuiteNode()
    spin_thread = start_spin_thread(node)

    fig = plt.figure(figsize=(15, 9))
    fig.canvas.manager.set_window_title('LIMO Test Suite')
    ax_path = fig.add_axes([0.05, 0.08, 0.55, 0.85])
    ax_wheel = fig.add_axes([0.66, 0.50, 0.30, 0.43])
    ax_info = fig.add_axes([0.63, 0.22, 0.35, 0.26])
    ax_info.axis('off')
    behind = node.behind_view
    style_lab_axes(ax_path, node.lab_x, node.lab_y,
                   'LIMO Test Suite (view from behind)' if behind else 'LIMO Test Suite',
                   behind_view=behind)
    wheels = WheelPanel(ax_wheel)
    info = ax_info.text(0.0, 1.0, '', va='top', ha='left', fontsize=14, family='monospace')

    path_line, = ax_path.plot([], [], '--', color='0.35', lw=2.5, label='Test path')
    trail_line, = ax_path.plot([], [], '-', color='darkorange', lw=3, label='Expected motion')
    start_dot, = ax_path.plot([], [], 'o', color='green', ms=12, label='Start', zorder=7)
    end_dot, = ax_path.plot([], [], 'x', color='red', ms=14, mew=3, label='End', zorder=7)
    robot = RobotGlyph(ax_path, label='LIMO (expected)')
    ax_path.legend(loc='upper left', fontsize=11)
    offset = {'xy': node.placement(node.path), 'origin': None, 'legend_done': False}

    # =========================================================================
    # [UI]     lab(x, y, yaw=0.0) -> (u, v, yaw_view)  (nested)
    # -------------------------------------------------------------------------
    # Path-frame point to canvas: a drawn path goes back to where it was drawn
    # (its 'origin'), a built-in path is shifted by the placement offset.
    # =========================================================================
    def lab(x, y, yaw=0.0):
        if offset['origin'] is not None:
            x0, y0, yaw0 = offset['origin']
            c, sn = math.cos(yaw0), math.sin(yaw0)
            return to_view(x0 + c * x - sn * y, y0 + sn * x + c * y, yaw + yaw0, behind)
        return to_view(x + offset['xy'][0], y + offset['xy'][1], yaw, behind)

    radio_ax = fig.add_axes([0.66, 0.05, 0.12, 0.15])
    radio = RadioButtons(radio_ax, PATH_NAMES, active=PATH_NAMES.index(node.name))
    for lbl in radio.labels:
        lbl.set_fontsize(13)
    btn_start = Button(fig.add_axes([0.81, 0.12, 0.15, 0.07]), 'Start', color='palegreen')
    btn_stop = Button(fig.add_axes([0.81, 0.04, 0.15, 0.07]), 'STOP', color='salmon')
    for b in (btn_start, btn_stop):
        b.label.set_fontsize(15)
        b.label.set_fontweight('bold')
    radio.on_clicked(lambda name: node.select(name))
    btn_start.on_clicked(lambda _e: node.start())
    btn_stop.on_clicked(lambda _e: node.stop())
    drawn_cache = {'path': None}

    # =========================================================================
    # [UI]     update_plot(frame_idx)  (nested, 10 Hz)
    # -------------------------------------------------------------------------
    # Redraws path (when the selection changed), expected trail, robot glyph
    # (heading = direction of the last traced segment), wheel panel and info.
    # =========================================================================
    def update_plot(_frame_idx):
        st = node.snapshot()
        p = st['path']
        if p is not drawn_cache['path']:
            drawn_cache['path'] = p
            offset['xy'] = node.placement(p)
            offset['origin'] = p.get('origin') if p is not None else None
            if p is None:
                path_line.set_data([], [])
                end_dot.set_data([], [])
            else:
                u, v, _ = lab(p['x'], p['y'])
                path_line.set_data(u, v)
                eu, ev, _ = lab(p['x'][-1], p['y'][-1])
                end_dot.set_data([eu], [ev])
            su, sv, _ = lab(0.0, 0.0)
            start_dot.set_data([su], [sv])
        tr = np.array(st['trail'])
        u, v, _ = lab(tr[:, 0], tr[:, 1])
        trail_line.set_data(u, v)
        # Heading: from the last traced point (>= 2 cm back) to the current one
        ex, ey, eth = st['pose']
        heading = eth
        for px, py in reversed(st['trail'][:-1]):
            if math.hypot(ex - px, ey - py) >= 0.02:
                heading = math.atan2(ey - py, ex - px)
                break
        robot.update(*lab(ex, ey, heading))
        if not offset['legend_done']:
            ax_path.legend(loc='upper left', fontsize=11)   # glyph is visible now
            offset['legend_done'] = True
        wheels.update(st['delta'], st['v'], node.speed)

        deg = math.degrees(st['delta'])
        side = 'LEFT ' if deg > 0.5 else 'RIGHT' if deg < -0.5 else 'STRAIGHT'
        if st['phase'] == 'COUNTDOWN':
            phase = f"STARTING IN {max(0.0, node.start_delay - st['t']):.1f} s"
        elif st['phase'] == 'RUNNING':
            phase = f"RUNNING  {st['t']:5.1f} / {node.duration():.1f} s"
        elif p is None:
            phase = 'waiting for /reference_path'
        else:
            phase = st['phase'] + f"   (duration {node.duration():.1f} s)"
        sat = abs(st['delta']) >= node.max_steer - 1e-6 and st['v'] > 0
        info.set_text(
            f"{phase}\n"
            f"path    : {st['name']}\n"
            f"speed   : {st['v']:.2f} m/s forward\n"
            f"steering: {side} {abs(deg):4.1f} deg\n"
            f"omega   : {st['omega']:+.2f} rad/s"
            + ("\n!! STEERING SATURATED" if sat else ""))
        info.set_color('red' if sat else 'black')

    ani = animation.FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)  # noqa: F841

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
