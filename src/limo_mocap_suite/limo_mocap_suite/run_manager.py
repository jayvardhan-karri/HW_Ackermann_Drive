# #############################################################################
# [FILE]   run_manager.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/run_manager.py
# ROLE  : Everything a controller does besides computing steering, shared by all
#         four: reach the path start, count laps, watch the errors, obey the
#         kill button, log each run to its own folder, and wait for the next path.
#
# Phases : WAIT_PATH -> APPROACH (only if not already on the start) -> TRACK -> WAIT_PATH
# Always : KILL; no odometry for odom_timeout -> zero command until it returns.
# Guard  : optional extension (guard:=true), off by default so the controller alone
#          decides what happens when the robot drifts. Watches the live error:
#            |cte| > slow_cte    SLOW    speed reduced, down to 40 %
#            |cte| > rejoin_cte  REJOIN  plan a drivable way back onto the path
#            |cte| > abort_cte   STOP    also: left the lab, too many rejoins,
#                                        no odometry for odom_abort, approach too slow
#          A CBF detour around an obstacle is not drift: REJOIN waits until it ends.
# Log    : <log_dir>/<stamp>_<controller>_<path_name>/telemetry.csv + summary.json
# Topics : publishes /cmd_vel, /run_status (JSON, 5 Hz), /reference_path_viz;
#          subscribes /run_kill (latched Bool)
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
# #############################################################################

import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from limo_mocap_suite import approach_planner
from limo_mocap_suite.lap_stats import LapStats

SLOW_MIN = 0.4       # smallest speed factor of the guard
PREFIX_LEN = 1.5     # metres of the real path kept after an approach (for the MPC horizon)


# =============================================================================
# [CORE]   class RunManager
# -----------------------------------------------------------------------------
# Used by a controller's control_loop:
#     tick = self.run.tick(x, y, yaw, lookahead)    # None: nothing to do now
#     cte, herr, ref_pt, idx, dt = tick
#     ... controller computes v, steer ...
#     v, steer = self.run.shape(v, steer, idx)      # guard speed
#     v, steer, clear, status = self.safety.apply(...)
#     self.run.command(v, steer, clear, status, solve_ms)
# =============================================================================
class RunManager:
    # =========================================================================
    # [LIFE]   __init__(node, name, traj, wheelbase, max_steer)
    # -------------------------------------------------------------------------
    # Declares the parameters, creates the topics and sets the idle state.
    # on_start() is called at each new run.
    # =========================================================================
    def __init__(self, node, name, traj, wheelbase, max_steer, on_start=None):
        self.node, self.name, self.traj = node, name, traj
        self.wheelbase, self.max_steer = wheelbase, max_steer
        self.on_start = on_start
        defaults = {
            'log_dir': 'evaluation', 'path_name': 'drawn', 'run_duration': 0.0,
            'laps': 1, 'exit_on_finish': False, 'guard': False, 'approach': True,
            'approach_radius': 0.60, 'approach_speed': 0.25, 'join_tol': 0.15, 'join_heading_deg': 30.0,
            'slow_cte': 0.15, 'rejoin_cte': 0.35, 'abort_cte': 1.00, 'max_rejoins': 3,
            'odom_timeout': 0.5, 'odom_abort': 3.0, 'obstacle_margin': 0.30,
            'guard_lab_x': [-0.5, 8.0], 'guard_lab_y': [-1.0, 6.0],
        }
        for key, value in defaults.items():
            if not node.has_parameter(key):
                node.declare_parameter(key, value)
        self.p = {key: node.get_parameter(key).value for key in defaults}
        self.guard_on = bool(self.p['guard'])

        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = node.create_publisher(String, '/run_status', 10)
        self.viz_pub = node.create_publisher(Path, '/reference_path_viz', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool, '/run_kill', self._kill_cb, latched)
        node.create_timer(0.2, self.publish_status)
        node.create_timer(2.0, self._print_status)

        self.killed = False
        self.phase = 'WAIT_PATH'
        self.result = None
        self.reason = ''
        self.guard = 'OK'
        self.last_odom = None
        self.main = None
        self.csv_file = None
        self.stats = None
        self.exit_requested = False
        self.stop_ticks = 0
        self.cte = self.herr = self.steer = self.steer_ff = 0.0
        self.prev_time = None
        self.lap = 1
        self.run_dir = ''
        self.lap_rows = []
        self.last_rms = None
        node.get_logger().info(f"Ready: waiting for a path. Runs are logged under {self.p['log_dir']}/")

    # -------------------------------------------------------------------------
    # [ROS-IO] inputs
    # -------------------------------------------------------------------------
    def note_odom(self):
        self.last_odom = time.monotonic()

    def _kill_cb(self, msg):
        self.killed = bool(msg.data)
        if self.killed:
            self.cmd_pub.publish(Twist())
            self.node.get_logger().error('KILL received: robot stopped')
            if self.csv_file is not None:
                self._finish('KILLED', 'kill button')
        else:
            self.traj.new_path = False      # a path from before the kill must not start by itself
            self.node.get_logger().warn('Re-armed: send a new path to run again')

    # =========================================================================
    # [CORE]   tick(x, y, yaw, lookahead) -> (cte, herr, ref_pt, idx, dt) or None
    # -------------------------------------------------------------------------
    # One control tick up to the point where the controller computes steering.
    # None: the controller must not command anything now (idle, killed, finished,
    # odometry lost). Those cases send the zero command themselves.
    # =========================================================================
    def tick(self, x, y, yaw, lookahead):
        now = time.monotonic()
        if self.exit_requested:
            raise SystemExit
        if self.killed or self.last_odom is None:
            if self.killed:
                self.cmd_pub.publish(Twist())
            return None
        if self.traj.new_path:
            self._start_run(x, y, yaw, now)
        if self.csv_file is None:
            if self.stop_ticks > 0:      # a few zeros after a run, then silence
                self.cmd_pub.publish(Twist())
                self.stop_ticks -= 1
            return None
        dt = max(now - self.prev_time, 1e-3)
        self.prev_time = now

        age = now - self.last_odom
        if age > self.p['odom_timeout']:
            self.cmd_pub.publish(Twist())
            self.guard = 'HOLD'
            if self.guard_on and age > self.p['odom_abort']:
                self._finish('ABORTED', f'no odometry for {age:.1f} s')
            return None
        if self.p['run_duration'] > 0.0 and now - self.t_start >= self.p['run_duration']:
            self._finish('TIME_LIMIT', f"run_duration {self.p['run_duration']} s reached")
            return None

        lab_x, lab_y = self.p['guard_lab_x'], self.p['guard_lab_y']
        if self.guard_on and not (lab_x[0] <= x <= lab_x[1] and lab_y[0] <= y <= lab_y[1]):
            self._finish('ABORTED', f'left the lab at ({x:.2f}, {y:.2f})')
            return None

        if self.guard_on and self.phase == 'APPROACH' and now > self.approach_deadline:
            self._finish('ABORTED', 'could not reach the path in time')
            return None
        cte, herr, ref_pt, idx = self.traj.compute_tracking_errors(x, y, yaw, lookahead_pts=lookahead)
        if self.phase == 'APPROACH' and idx >= self.join_at:
            self._join_path(idx - self.join_at + self.join_idx)
            cte, herr, ref_pt, idx = self.traj.compute_tracking_errors(x, y, yaw, lookahead_pts=lookahead)
        if self.phase == 'TRACK' and self._update_progress(idx):
            return None
        if self.traj.goal_reached:
            self._finish('FINISHED', 'end of path')
            return None
        self.cte, self.herr, self.x, self.y, self.yaw, self.idx, self.dt = cte, herr, x, y, yaw, idx, dt
        if self._update_guard(cte, herr, now):
            return None
        return cte, herr, ref_pt, idx, dt

    # =========================================================================
    # [CORE]   shape(v, steer, idx) -> (v, steer)
    # -------------------------------------------------------------------------
    # Applies the guard's speed factor and remembers what the geometry needs here.
    # =========================================================================
    def shape(self, v, steer, idx):
        self.steer_ctrl = steer
        self.steer_ff = math.atan(self.wheelbase * self.traj.waypoints[idx, 4])
        return v * self.guard_factor, float(np.clip(steer, -self.max_steer, self.max_steer))

    # =========================================================================
    # [CORE]   command(v, steer, clearance, status, solve_ms=0.0)
    # -------------------------------------------------------------------------
    # Publishes the Twist, feeds the lap statistics and writes the CSV row.
    # =========================================================================
    def command(self, v, steer, clearance, status, solve_ms=0.0):
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        self.steer = steer
        self.cbf_status = status
        if status == 'CBF_ACTIVE':
            self.last_cbf = time.monotonic()
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(v / self.wheelbase * math.tan(steer)) if abs(v) > 1e-4 else 0.0
        self.cmd_pub.publish(twist)
        if self.phase == 'TRACK':
            self.stats.record(self.steer_ff, steer, self.cte, self.dt,
                              score=self.lap > 1 or self.progress * self.spacing > 1.0)
        kappa = self.traj.waypoints[self.idx, 4]
        self.writer.writerow([
            f'{time.monotonic() - self.t_start:.3f}', self.phase, self.lap,
            f'{self.x:.4f}', f'{self.y:.4f}', f'{self.yaw:.4f}', f'{self.node.v_actual:.4f}',
            f'{v:.4f}', f'{twist.angular.z:.4f}', f'{steer:.4f}', f'{self.steer_ff:.4f}',
            f'{self.steer_ctrl:.4f}',
            f'{self.traj.waypoints[self.idx, 0]:.4f}', f'{self.traj.waypoints[self.idx, 1]:.4f}',
            f'{self.traj.waypoints[self.idx, 2]:.4f}', f'{kappa:.4f}',
            f'{self.idx * self.spacing:.3f}', f'{self.cte:.4f}', f'{self.herr:.4f}',
            f'{clearance:.3f}', status, self.guard, f'{solve_ms:.3f}',
        ])

    # =========================================================================
    # [LIFE]   _start_run(x, y, yaw, now)
    # -------------------------------------------------------------------------
    # A new path arrived: close any run in progress, open a new log, keep the path
    # as the main table, and plan the approach if the robot is not on its start.
    # =========================================================================
    def _start_run(self, x, y, yaw, now):
        self.traj.new_path = False
        if self.csv_file is not None:
            self._finish('REPLACED', 'a new path arrived')
        self.exit_requested = False
        self.main = self.traj.waypoints.copy()
        self.main_closed = self.traj.is_closed
        seg = np.hypot(np.diff(self.main[:, 0]), np.diff(self.main[:, 1]))
        self.spacing = max(float(np.median(seg)), 1e-3)
        self.length = float(seg.sum())
        self.laps = int(self.p['laps']) if self.main_closed else 1
        if not self.main_closed and int(self.p['laps']) > 1:
            self.node.get_logger().warn('Open path: it can only be driven once; draw a closed path for laps')
        self.stats = LapStats(self.max_steer)
        self.lap = 1
        self.lap_rows = []
        self.progress = 0
        self.rejoins = 0
        self.lap_rejoins = 0
        self.last_cbf = -1e9
        self.last_plan = -1e9
        self.guard = 'OK'
        self.guard_factor = 1.0
        self.result = None
        self.reason = ''
        self.approach_time = 0.0
        self.heading_to_start = True
        self.steer_ctrl = 0.0
        self.steer_ff = 0.0
        self.t_start = now
        self.t_lap = now
        self.prev_time = now
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(self.p['log_dir'], f"{stamp}_{self.name}_{self.p['path_name']}")
        os.makedirs(self.run_dir, exist_ok=True)
        self.csv_file = open(os.path.join(self.run_dir, 'telemetry.csv'), 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['t', 'phase', 'lap', 'x', 'y', 'yaw', 'v_actual', 'v_cmd', 'omega_cmd', 'steer_rad',
                              'steer_ff', 'steer_ctrl', 'x_ref', 'y_ref', 'yaw_ref', 'kappa',
                              's', 'cte', 'heading_err', 'min_clearance', 'safety_status', 'guard', 'solve_ms'])
        if self.on_start:
            self.on_start()
        self._publish_viz()

        kappa_max = math.tan(self.max_steer) / self.wheelbase
        peak = float(np.max(np.abs(self.main[:, 4])))
        if peak > kappa_max:
            self.node.get_logger().warn(
                f'Path is tighter than the robot can turn: curvature {peak:.2f} 1/m, limit {kappa_max:.2f} 1/m')

        sx, sy, syaw = self.main[0, :3]
        dist = math.hypot(sx - x, sy - y)
        dyaw = abs(math.atan2(math.sin(syaw - yaw), math.cos(syaw - yaw)))
        on_start = dist <= self.p['join_tol'] and dyaw <= math.radians(self.p['join_heading_deg'])
        if on_start or not bool(self.p['approach']):
            self._join_path(0)
            self.node.get_logger().info(f'Run started on the path: {self.run_dir}')
        else:
            self._plan_way_back((x, y, yaw), 0, to_start=True)
            if self.csv_file is not None:
                self.node.get_logger().info(
                    f'Run started, {dist:.2f} m from the start: driving there first. {self.run_dir}')

    # =========================================================================
    # [UTIL]   _allowed(table) -> bool
    # -------------------------------------------------------------------------
    # A way back must stay in the lab and clear of the known obstacles.
    # =========================================================================
    def _allowed(self, table):
        lab_x, lab_y = self.p['guard_lab_x'], self.p['guard_lab_y']
        if (table[:, 0].min() < lab_x[0] or table[:, 0].max() > lab_x[1] or
                table[:, 1].min() < lab_y[0] or table[:, 1].max() > lab_y[1]):
            return False
        safety = getattr(self.node, 'safety', None)
        if safety is None:
            return True
        flat = safety.param_obstacles + safety.topic_obstacles
        margin = self.p['obstacle_margin']
        c, s = math.cos(self.node.yaw0), math.sin(self.node.yaw0)
        for i in range(0, len(flat) - 2, 3):
            dx, dy = flat[i] - self.node.x0, flat[i + 1] - self.node.y0
            ox, oy = c * dx + s * dy, -s * dx + c * dy
            if np.min(np.hypot(table[:, 0] - ox, table[:, 1] - oy)) < flat[i + 2] + margin:
                return False
        return True

    # =========================================================================
    # [CORE]   _plan_way_back(pose, idx, to_start=False)
    # -------------------------------------------------------------------------
    # Dubins path to the path start, or to a point ahead on the path. It becomes
    # the table the controller tracks, followed by a piece of the real path so the
    # MPC horizons see what comes next. The controller does not know the difference.
    # =========================================================================
    def _plan_way_back(self, pose, idx, to_start=False):
        radius = float(self.p['approach_radius'])
        if to_start:
            got = approach_planner.plan(pose, tuple(self.main[0, :3]), radius, allowed=self._allowed)
            join = 0
            table = None if got is None else got[0]
        else:
            got = approach_planner.plan_rejoin(pose, self.main, idx, self.main_closed, radius, allowed=self._allowed)
            table, join = (None, 0) if got is None else got
        if table is None and self.guard_on:
            self._finish('ABORTED', 'no drivable way back inside the lab')
            return
        if table is None:       # no guard: the lab and obstacle checks are only a preference
            self.node.get_logger().warn('No way to the path inside the lab and clear of obstacles: taking the shortest')
            table = approach_planner.plan(pose, tuple(self.main[0, :3]), radius)[0]
            join = 0
        v = float(self.p['approach_speed'])
        way = np.column_stack((table[:, 0], table[:, 1], table[:, 2], np.full(len(table), v), table[:, 3]))
        n_prefix = int(PREFIX_LEN / self.spacing)
        order = np.arange(join, join + n_prefix)
        order = order % len(self.main) if self.main_closed else order[order < len(self.main)]
        self.heading_to_start = to_start
        self.last_plan = time.monotonic()
        self.approach_deadline = time.monotonic() + 15.0 + 3.0 * len(way) * 0.01 / max(v, 0.05)
        self.join_at = len(way)
        self.join_idx = join
        self.traj.load_table(np.vstack((way, self.main[order])), closed=False, start_idx=0)
        self.phase = 'APPROACH'

    # =========================================================================
    # [CORE]   _join_path(idx)
    # -------------------------------------------------------------------------
    # The robot is on the path at main-table index idx: track the real path now.
    # =========================================================================
    def _join_path(self, idx):
        idx = int(idx) % len(self.main)
        was_tracking = self.phase == 'TRACK' or not self.heading_to_start
        self.traj.load_table(self.main, closed=self.main_closed, start_idx=idx)
        if not was_tracking:
            self.approach_time = time.monotonic() - self.t_start
            self.t_lap = time.monotonic()
            self.idx_prev = idx
        else:   # rejoined further on: the part skipped still counts as driven
            self.progress += (idx - self.idx_prev) % len(self.main)
            self.idx_prev = idx
        self.phase = 'TRACK'

    # =========================================================================
    # [CORE]   _update_progress(idx) -> True if the run just finished
    # -------------------------------------------------------------------------
    # Laps are counted by forward progress along the path, so stopping or wobbling
    # at the start line cannot count as a lap.
    # =========================================================================
    def _update_progress(self, idx):
        if not self.main_closed:
            self.idx_prev = idx
            return False
        n = len(self.main)
        d = (idx - self.idx_prev + n // 2) % n - n // 2
        self.idx_prev = idx
        if d > 0:
            self.progress += d
        if self.progress < self.lap * n:
            return False
        self._end_lap()
        if self.laps > 0 and self.lap >= self.laps:
            self._finish('FINISHED', f'{self.laps} laps done')
            return True
        self.lap += 1
        return False

    def _end_lap(self):
        stats = self.stats.end_lap()
        now = time.monotonic()
        stats.update(lap=self.lap, lap_time=now - self.t_lap, rejoins=self.lap_rejoins)
        self.t_lap = now
        self.lap_rejoins = 0
        self.lap_rows.append(stats)
        self.last_rms = stats['rms_cte']
        self.node.get_logger().info(
            f"LAP {self.lap}: RMS {100 * stats['rms_cte']:.1f} cm, max {100 * stats['max_cte']:.1f} cm, "
            f"at steering limit {100 * stats['sat_share']:.0f}%, {stats['lap_time']:.1f} s")

    # =========================================================================
    # [CORE]   _update_guard(cte, herr, now) -> True if the run was stopped
    # -------------------------------------------------------------------------
    # See the file header. Sets self.guard and self.guard_factor.
    # =========================================================================
    def _update_guard(self, cte, herr, now):
        if not self.guard_on:
            self.guard = 'OFF'
            return False
        e = abs(cte)
        detour = now - self.last_cbf < 3.0
        if e > self.p['abort_cte']:
            self._finish('ABORTED', f'off the path by {e:.2f} m')
            return True
        drifting = e > self.p['rejoin_cte'] or (abs(herr) > math.radians(75.0) and e > self.p['slow_cte'])
        if drifting and not detour and now - self.last_plan > 2.0:
            self.rejoins += 1
            self.lap_rejoins += 1
            if self.rejoins > self.p['max_rejoins']:
                self._finish('ABORTED', f"more than {self.p['max_rejoins']} rejoins")
                return True
            self.node.get_logger().warn(f'DRIFT: {e:.2f} m off, planning a way back ({self.rejoins})')
            if self.phase == 'APPROACH' and self.heading_to_start:
                self._plan_way_back((self.x, self.y, self.yaw), 0, to_start=True)
            elif self.phase == 'APPROACH':      # off the way back: from where it was heading
                self._plan_way_back((self.x, self.y, self.yaw), self.join_idx)
            else:
                self.idx_prev = self.traj.last_idx
                self._plan_way_back((self.x, self.y, self.yaw), self.traj.last_idx)
            if self.csv_file is None:
                return True
        if self.phase == 'APPROACH':      # already slow; slower would only make it take forever
            self.guard_factor = 1.0
            self.guard = 'REJOIN' if self.rejoins > 0 else 'OK'
            return False
        span = max(self.p['rejoin_cte'] - self.p['slow_cte'], 1e-3)
        over = (e - self.p['slow_cte']) / span
        self.guard_factor = 1.0 if over <= 0.0 else max(SLOW_MIN, 1.0 - (1.0 - SLOW_MIN) * over)
        self.guard = 'SLOW' if over > 0.0 else 'OK'
        return False

    # =========================================================================
    # [LOG]    _finish(result, reason)
    # -------------------------------------------------------------------------
    # Stops the robot, closes the log, writes summary.json. With exit_on_finish
    # the node leaves on its next tick; otherwise it waits for the next path.
    # =========================================================================
    def _finish(self, result, reason):
        if self.csv_file is None:
            return
        self.cmd_pub.publish(Twist())
        self.stop_ticks = 10
        self.result, self.reason = result, reason
        partial = self.phase == 'TRACK' and self.stats.ticks > 50 and result != 'FINISHED'
        if partial:
            self._end_lap()
            self.lap_rows[-1]['partial'] = True
        self.csv_file.close()
        self.csv_file = None
        summary = {
            'controller': self.name, 'path_name': self.p['path_name'], 'obstacle_mode': getattr(self.node, 'obstacle_mode', 'cbf'), 'result': result, 'reason': reason,
            'guard': self.guard_on, 'laps_requested': self.laps, 'closed_path': self.main_closed,
            'path_length_m': self.length, 'path_peak_curvature': float(np.max(np.abs(self.main[:, 4]))),
            'curvature_limit': math.tan(self.max_steer) / self.wheelbase, 'max_steer': self.max_steer,
            'approach_time_s': self.approach_time, 'rejoins': self.rejoins,
            'duration_s': time.monotonic() - self.t_start, 'laps': self.lap_rows,
            'obstacles': self._obstacles_flat(),
        }
        with open(os.path.join(self.run_dir, 'summary.json'), 'w') as f:
            json.dump(summary, f, indent=2)
        if result in ('FINISHED', 'REPLACED'):
            self.node.get_logger().info(f'RUN {result}: {reason}. Log: {self.run_dir}')
        else:
            self.node.get_logger().error(f'RUN {result}: {reason}. Log: {self.run_dir}')
        self.phase = 'WAIT_PATH'
        self.traj.path_ready = False
        self.publish_status()
        if bool(self.p['exit_on_finish']) and result != 'REPLACED':
            self.exit_requested = True

    def _obstacles_flat(self):
        safety = getattr(self.node, 'safety', None)
        return [] if safety is None else list(safety.param_obstacles) + list(safety.topic_obstacles)

    # =========================================================================
    # [LIFE]   shutdown()
    # -------------------------------------------------------------------------
    # Ctrl-C: zero command, and the run in progress is logged as STOPPED.
    # =========================================================================
    def shutdown(self):
        if self.csv_file is not None:
            self._finish('STOPPED', 'node shut down')
        if rclpy.ok():
            self.cmd_pub.publish(Twist())

    # -------------------------------------------------------------------------
    # [ROS-IO] outputs for people and the live view
    # -------------------------------------------------------------------------
    def publish_status(self):
        msg = String()
        running = self.csv_file is not None
        msg.data = json.dumps({
            'controller': self.name, 'phase': 'KILLED' if self.killed else self.phase,
            'lap': self.lap if running else 0, 'laps': self.laps if self.main is not None else 0,
            'cte': self.cte, 'herr': self.herr, 'steer': self.steer, 'steer_ff': self.steer_ff,
            'max_steer': self.max_steer, 'guard': self.guard if running else '',
            'guard_on': self.guard_on, 'slow_cte': self.p['slow_cte'], 'rejoin_cte': self.p['rejoin_cte'],
            'rms_last': self.last_rms,
            'result': self.result, 'reason': self.reason, 'run_dir': self.run_dir,
        })
        self.status_pub.publish(msg)

    def _print_status(self):
        if self.csv_file is not None:
            self.node.get_logger().info(
                f'{self.phase} lap {self.lap}/{self.laps} | CTE {self.cte:+.3f} m | '
                f'steer {math.degrees(self.steer):+.0f} deg | guard {self.guard}')

    def _publish_viz(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for pt in self.main[::5]:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(pt[0]), float(pt[1])
            msg.poses.append(ps)
        self.viz_pub.publish(msg)
