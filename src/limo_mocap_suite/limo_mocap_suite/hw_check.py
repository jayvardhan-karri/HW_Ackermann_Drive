# #############################################################################
# [FILE]   hw_check.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/hw_check.py
# ROLE  : Lab check, to run before any tracking. Finds what no desk test can:
#         a mirrored mocap frame, swapped markers, a wrong namespace, a robot that
#         keeps driving when commands stop, and how the real steering compares with
#         the model. It reports and suggests; it changes nothing.
#
# Console script : ros2 run limo_mocap_suite hw_check   (normally: hw_check.launch.py)
# Subscribes     : /hardware_odom, /run_kill (latched)
# Publishes      : /cmd_vel, only with drive:=true
# Stage 1, robot still (always)  : mocap rate and gaps, jitter, is anything on /cmd_vel
# Stage 2, slow drive (drive:=true), all at `speed`, with zero commands between:
#   straight : speed gain; is yaw the driving direction; steering trim; start lag
#   arc, full lock, left then right: does a left command turn left; curvature gain
#              each way; ref_offset; real tightest radius each way
#   silence  : commands stop with no zero sent: does the robot stop by itself?
# Any FAIL, KILL, lost odometry or leaving max_range stops the robot at once.
# Report : <log_dir>/hw_check_<stamp>/report.json, and printed.
# Needs phasespace_bridge running (it publishes /hardware_odom from the mocap
# markers; frame is ROS right-handed: +x forward, +y left). Parameters: drive
# (False), speed, wheelbase, ref_offset, max_steer, still_time, start_delay,
# max_range, odom_timeout, log_dir. Findings are PASS / WARN / FAIL / INFO.
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
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from limo_mocap_suite.ros_compat import init_ros

# Durations of the drive segments
SEGMENT_STRAIGHT = 3.0  # s
SEGMENT_ARC = 3.5
SEGMENT_FULL = 4.0
SETTLE = 1.5            # s to wait for speed and steering before measuring
ARC_STEER = 0.25        # rad: well inside the limit
# Limits for the still-robot checks of Stage 1
MIN_RATE = 50.0         # Hz of odometry worth warning below
MAX_GAP = 0.20          # s: longest allowed pause between odometry messages
MAX_POS_JITTER = 0.005  # m: standard deviation of the position while still
MAX_YAW_JITTER = math.radians(1.0)


# =============================================================================
# [LIFE]   class Stop(Exception)
# -----------------------------------------------------------------------------
# Raised to end the drive at once (KILL, odometry lost, bad result). run()
# catches it, stops the robot and reports the message as a FAIL.
# =============================================================================
class Stop(Exception):
    """The robot must stop now; the reason is the message."""


# =============================================================================
# [LIFE]   class OutOfRoom(Stop)
# -----------------------------------------------------------------------------
# The robot left max_range. Not a robot fault: reported as a WARN (incomplete),
# not a FAIL.
# =============================================================================
class OutOfRoom(Stop):
    """Nothing is wrong with the robot; the checks after this point were not done."""


# =============================================================================
# [UTIL]   wrap(a) -> rad in (-pi, pi]
# -----------------------------------------------------------------------------
# Angle a (rad) wrapped, so a difference of two yaws never jumps by 2 pi.
# =============================================================================
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# =============================================================================
# [CORE]   measure(samples) -> dict or None
# -----------------------------------------------------------------------------
# samples: rows (t s, x m, y m, yaw rad), from one steady stretch of driving. Returns the
# speed (m/s), the curvature kappa = dyaw/ds of the path driven (1/m, left
# positive), dyaw (total turn, rad) and beta: the driving direction
# of the tracked point relative to the robot's yaw (rad). None: too little data or motion.
# =============================================================================
def measure(samples):
    if len(samples) < 10:
        return None
    a = np.array(samples)
    # Distance over 0.2 s steps: adding up every frame would also add up the mocap noise
    every = max(1, int(round(0.2 / max(float(np.median(np.diff(a[:, 0]))), 1e-3))))
    coarse = a[::every]
    if len(coarse) < 2:
        return None
    # ds = path length driven, span = time
    ds = float(np.hypot(np.diff(coarse[:, 1]), np.diff(coarse[:, 2])).sum())
    span = float(coarse[-1, 0] - coarse[0, 0])
    if ds < 0.03 or span <= 0.0:
        return None
    # unwrap: yaw across +-pi must not look like a full turn
    turned = float(np.unwrap(coarse[:, 3])[-1] - np.unwrap(coarse[:, 3])[0])
    yaw = np.unwrap(a[:, 3])
    # A chord is less noisy than single steps, and on an arc its direction is the
    # tangent at the middle of the arc, so it is compared with the yaw at the middle.
    half = len(a) // 2
    betas = []
    for i in range(half):
        j = i + half
        course = math.atan2(a[j, 2] - a[i, 2], a[j, 1] - a[i, 1])
        betas.append(wrap(course - yaw[(i + j) // 2]))
    # Circular mean of beta (arctan2 of mean sin, mean cos)
    return {'speed': ds / span, 'kappa': turned / ds,
            'beta': float(np.arctan2(np.mean(np.sin(betas)), np.mean(np.cos(betas)))), 'dyaw': turned}


# =============================================================================
# [CORE]   analyse(results, wheelbase, ref_offset, speed, arc_steer, max_steer)
#              -> (list of (level, name, text), dict of suggestions)
# -----------------------------------------------------------------------------
# r: dict segment name -> measure() result (straight, left, right, full_left,
# full_right, coast). wheelbase, ref_offset in m; speed in m/s; arc_steer and
# max_steer in rad. Turns the measured segments into findings. Level: PASS, WARN
# or FAIL. The dict holds suggested parameter values (ref_offset, the real tightest
# radius and steering limit); nothing is changed by itself.
# =============================================================================
def analyse(r, wheelbase, ref_offset, speed, arc_steer, max_steer):
    out, tips = [], {}
    add = lambda level, name, text: out.append((level, name, text))  # noqa: E731  (appends one finding)
    straight = r.get('straight')
    if straight:
        gain = straight['speed'] / speed
        add('PASS' if 0.8 <= gain <= 1.2 else 'WARN', 'Speed',
            f"{straight['speed']:.3f} m/s for {speed:.3f} commanded ({100 * gain:.0f}%)")
        if abs(straight['kappa']) > 0.05:
            trim = math.atan(wheelbase * straight['kappa'])    # steering angle that gives this curvature
            side = 'left' if trim > 0 else 'right'
            add('WARN', 'Trim', f"pulls {side} on a zero steering command: {math.degrees(trim):+.1f} deg. "
                'The PID and MPC integrals can absorb this; LQR and NMPC will hold a small offset')
        else:
            add('PASS', 'Trim', f"drives straight (curvature {straight['kappa']:+.3f} 1/m)")
    expected = math.tan(arc_steer) / wheelbase    # bicycle model: kappa = tan(steer) / L
    gains = {}
    for name, label in (('left', 'Left'), ('right', 'Right')):
        m = r.get(name)
        if not m:
            continue
        gains[name] = abs(m['kappa']) / expected
        add('PASS' if 0.8 <= gains[name] <= 1.2 else 'WARN', f'{label} arc',
            f"curvature {abs(m['kappa']):.2f} 1/m, model expects {expected:.2f} ({100 * gains[name]:.0f}%)")
    if len(gains) == 2 and abs(gains['left'] - gains['right']) > 0.1:
        add('WARN', 'Left vs right', f"{100 * gains['left']:.0f}% left, {100 * gains['right']:.0f}% right: "
            'the steering is not symmetric')
    if r.get('left') and r.get('right'):
        # Beta is atan(a kappa) for the tracked point a ahead of the rear axle. Using left
        # and right together cancels any constant error in the markers' direction.
        estimates = [math.tan(r[s]['beta']) / r[s]['kappa'] for s in ('left', 'right')]
        a = 0.5 * sum(estimates)
        if abs(a - ref_offset) > 0.03:
            add('WARN', 'ref_offset', f'measures about {a:.2f} m, configured {ref_offset:.2f} m. '
                'Rough estimate: confirm with a tape measure')
            tips['ref_offset'] = round(a, 2)
        else:
            add('PASS', 'ref_offset', f'measures about {a:.2f} m, configured {ref_offset:.2f} m')
    for name, label in (('full_left', 'Left'), ('full_right', 'Right')):
        m = r.get(name)
        if m and abs(m['kappa']) > 1e-3:
            radius = 1.0 / abs(m['kappa'])
            model = wheelbase / math.tan(max_steer)
            add('PASS' if abs(radius - model) < 0.08 else 'WARN', f'{label} tightest radius',
                f'{radius:.2f} m; the controllers assume {model:.2f} m')
    radii = [1.0 / abs(r[k]['kappa']) for k in ('full_left', 'full_right') if r.get(k) and abs(r[k]['kappa']) > 1e-3]
    if radii:
        tips['tightest_radius_m'] = round(max(radii), 3)
        tips['real_steer_limit_rad'] = round(math.atan(wheelbase / max(radii)), 3)
    if 'coast' in r:
        moved, stopped = r['coast']['moved'], r['coast']['stopped']
        if stopped:
            add('PASS', 'Commands stop', f'robot stopped by itself after {moved:.2f} m')
        else:
            add('WARN', 'Commands stop', f'robot kept driving ({moved:.2f} m in {r["coast"]["time"]:.1f} s) with no '
                'commands. If the WiFi or this computer dies mid-run it will not stop: keep the hardware stop in reach')
    return out, tips


# =============================================================================
# [ROS-IO] class HwCheck(Node)
# -----------------------------------------------------------------------------
# The lab-check node. The ROS callbacks only store odometry; run() (in the
# main thread) drives the stages while rclpy.spin runs in another thread.
# self.lock guards the odometry samples shared by the two threads.
# =============================================================================
class HwCheck(Node):
    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Declares the parameters listed in the file header, creates the /cmd_vel
    # publisher and subscribes to /run_kill (latched) and /hardware_odom.
    # =========================================================================
    def __init__(self):
        super().__init__('hw_check')
        defaults = {'drive': False, 'speed': 0.15, 'wheelbase': 0.20, 'ref_offset': 0.10, 'max_steer': 0.4084,
                    'still_time': 5.0, 'start_delay': 5.0, 'max_range': 3.0, 'odom_timeout': 0.5,
                    'log_dir': 'evaluation'}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/run_kill', self._kill, latched)
        self.create_subscription(Odometry, '/hardware_odom', self._odom, 50)
        self.samples = []       # rows (time.monotonic(), x m, y m, yaw rad)
        self.last_odom = None   # time of the newest sample, to detect lost odometry
        self.killed = False
        self.lock = threading.Lock()

    # =========================================================================
    # [ROS-IO] _kill(msg: Bool)  (helper: callback of /run_kill)
    # -------------------------------------------------------------------------
    # Remembers the KILL state; check() raises Stop while it is set.
    # =========================================================================
    def _kill(self, msg):
        self.killed = bool(msg.data)

    # =========================================================================
    # [ROS-IO] _odom(msg: Odometry)  (helper: callback of /hardware_odom)
    # -------------------------------------------------------------------------
    # Stores (monotonic time, x, y, yaw) of every odometry message. Yaw comes
    # from the quaternion (yaw-only rotation).
    # =========================================================================
    def _odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        now = time.monotonic()
        with self.lock:
            self.samples.append((now, p.position.x, p.position.y, yaw))
            self.last_odom = now

    # =========================================================================
    # [UTIL]   take(t0, t1=None) -> list of samples
    # -------------------------------------------------------------------------
    # Samples with t0 <= time <= t1 (time.monotonic() seconds; t1 None = no end).
    # =========================================================================
    def take(self, t0, t1=None):
        with self.lock:
            return [s for s in self.samples if s[0] >= t0 and (t1 is None or s[0] <= t1)]

    # =========================================================================
    # [UTIL]   latest() -> newest sample (t, x, y, yaw) or None
    # =========================================================================
    def latest(self):
        with self.lock:
            return self.samples[-1] if self.samples else None

    # =========================================================================
    # [ROS-IO] send(v, steer)
    # -------------------------------------------------------------------------
    # One command, as the controllers send it: v in m/s, steer in rad (centre
    # steering angle), omega = v tan(steer) / L. Zero omega when v is ~0.
    # =========================================================================
    def send(self, v, steer=0.0):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(v / self.p['wheelbase'] * math.tan(steer)) if abs(v) > 1e-4 else 0.0
        self.cmd_pub.publish(t)

    # =========================================================================
    # [ROS-IO] stop(seconds=0.2)
    # -------------------------------------------------------------------------
    # Sends zero commands at 50 Hz for the given time, so one lost message
    # cannot leave the robot moving.
    # =========================================================================
    def stop(self, seconds=0.2):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(0.0)
            time.sleep(0.02)

    # =========================================================================
    # [CORE]   check()
    # -------------------------------------------------------------------------
    # Safety test, called every 20 ms while driving. Raises Stop on KILL or
    # when odometry is older than odom_timeout; raises OutOfRoom when the robot
    # is further than max_range (m) from where the drive started (self.origin).
    # =========================================================================
    def check(self):
        if self.killed:
            raise Stop('KILL pressed')
        if self.last_odom is None or time.monotonic() - self.last_odom > self.p['odom_timeout']:
            raise Stop('odometry lost')
        now = self.latest()
        if math.hypot(now[1] - self.origin[0], now[2] - self.origin[1]) > self.p['max_range']:
            raise OutOfRoom(f"reached max_range ({self.p['max_range']} m) from where it started, so the rest was not "
                            'measured. Start from somewhere with more room, or raise max_range')

    # =========================================================================
    # [CORE]   drive_for(seconds, v, steer=0.0) -> (samples, t_start)
    # -------------------------------------------------------------------------
    # Holds the command v (m/s), steer (rad) for the time (50 Hz, checking the
    # safety test each step), then stops for 1 s. Returns the samples recorded
    # after SETTLE, so speed and steering have settled, and the start time.
    # =========================================================================
    def drive_for(self, seconds, v, steer=0.0):
        """Holds a command for the given time. Returns the samples after the settling time."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.send(v, steer)
            self.check()
            time.sleep(0.02)
        got = self.take(t0 + SETTLE)
        self.stop(1.0)
        return got, t0

    # =========================================================================
    # [CORE]   still_stage() -> list of findings
    # -------------------------------------------------------------------------
    # Robot stationary: is mocap good, and is anything listening on /cmd_vel?
    # Waits up to 15 s for odometry, records still_time seconds, then checks
    # rate, gaps, position and heading jitter, and drift. Returns findings
    # (level, name, text). Nothing is published.
    # =========================================================================
    def still_stage(self):
        log = self.get_logger()
        found = []
        log.info('Waiting for /hardware_odom ...')
        t0 = time.monotonic()
        while self.latest() is None:
            if time.monotonic() - t0 > 15.0:
                return [('FAIL', 'Mocap', 'no /hardware_odom in 15 s. Is the bridge running, the same '
                         'ROS_DOMAIN_ID set, and are the robot markers (front_id, rear_id) tracked?')]
            time.sleep(0.1)
        log.info(f"Keep the robot still for {self.p['still_time']:.0f} s ...")
        t0 = time.monotonic()
        time.sleep(float(self.p['still_time']))
        got = np.array(self.take(t0))
        if len(got) < 5:
            return [('FAIL', 'Mocap', 'odometry stopped while the robot was still: markers occluded?')]
        gaps = np.diff(got[:, 0])
        rate = 1.0 / float(np.median(gaps))
        found.append(('PASS' if rate >= MIN_RATE else 'WARN', 'Mocap rate', f'{rate:.0f} Hz'))
        found.append(('PASS' if gaps.max() <= MAX_GAP else 'WARN', 'Mocap gaps', f'longest {gaps.max():.3f} s'))
        pos = float(max(got[:, 1].std(), got[:, 2].std()))
        found.append(('PASS' if pos <= MAX_POS_JITTER else 'WARN', 'Position jitter', f'{1000 * pos:.1f} mm'))
        yaw = float(np.unwrap(got[:, 3]).std())
        found.append(('PASS' if yaw <= MAX_YAW_JITTER else 'WARN', 'Heading jitter', f'{math.degrees(yaw):.2f} deg'))
        moved = float(np.hypot(got[:, 1] - got[0, 1], got[:, 2] - got[0, 2]).max())
        if moved > 0.02:
            found.append(('WARN', 'Still?', f'the pose moved {100 * moved:.1f} cm while it should be still'))
        found.append(('INFO', 'Pose', f'x {got[-1, 1]:.2f} m, y {got[-1, 2]:.2f} m, '
                      f'heading {math.degrees(got[-1, 3]):+.0f} deg'))
        n = self.cmd_pub.get_subscription_count()
        found.append(('PASS' if n else 'FAIL', 'Command topic',
                      f'{n} subscriber(s)' if n else 'nothing subscribes to the command topic. Is the robot on, '
                      'on this ROS_DOMAIN_ID, and is robot_ns right? Check: ros2 topic list | grep cmd_vel'))
        return found

    # =========================================================================
    # [CORE]   drive_stage(results)
    # -------------------------------------------------------------------------
    # The slow drive. Raises Stop at the first sign that driving on is unsafe:
    # tracking with a mirrored frame or a reversed heading would run away.
    # Order: straight, left, full left, right, full right, then the commands-stop
    # test. results: dict filled with measure() output per segment.
    # =========================================================================
    def drive_stage(self, results):
        log = self.get_logger()
        v = float(self.p['speed'])
        limit = float(self.p['max_steer'])
        first = self.latest()
        self.origin = (first[1], first[2])
        log.warn(f"The robot drives in {self.p['start_delay']:.0f} s, up to {self.p['max_range']} m from here at "
                 f'{v} m/s. Stop it with KILL in the live view or Ctrl-C here.')
        time.sleep(float(self.p['start_delay']))
        self.check()

        log.info('Straight ...')
        got, t0 = self.drive_for(SEGMENT_STRAIGHT, v)
        m = measure(got)
        if m is None:
            raise Stop('the robot did not move on a forward command: wrong topic, or speed below what the motors manage')
        results['straight'] = m
        moving = [s for s in self.take(t0) if math.hypot(s[1] - first[1], s[2] - first[2]) > 0.01]
        if moving:
            results['start_lag'] = moving[0][0] - t0
        if abs(m['beta']) > math.radians(60.0):
            raise Stop(f"the robot drove {math.degrees(m['beta']):+.0f} deg away from its yaw: the markers do not point "
                       'along the robot. If about 180, front_id and rear_id are swapped')

        # Both left turns, then both right: the robot curls back rather than wandering off
        for name, steer, seconds in (('left', ARC_STEER, SEGMENT_ARC), ('full_left', limit, SEGMENT_FULL),
                                     ('right', -ARC_STEER, SEGMENT_ARC), ('full_right', -limit, SEGMENT_FULL)):
            log.info(f"{name.replace('_', ' ').capitalize()} ...")
            got, _ = self.drive_for(seconds, v, steer)
            m = measure(got)
            if m is None:
                raise Stop(f'the robot did not move during {name}')
            results[name] = m
            if m['kappa'] * steer < 0.0:
                raise Stop(f"a {name.split('_')[-1]} command turned the other way (yaw changed "
                           f"{math.degrees(m['dyaw']):+.0f} deg): the mocap frame is mirrored. "
                           'Tracking would run away. Fix the frame first')

        # Last test: drive, then send NOTHING (not even zeros) and see whether it stops.
        log.info('Commands stop: does the robot stop by itself? ...')
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            self.send(v)
            self.check()
            time.sleep(0.02)
        t1 = time.monotonic()
        wait = 1.5
        while time.monotonic() - t1 < wait:      # no command sent here, on purpose
            self.check()
            time.sleep(0.02)
        self.stop(1.0)
        got = self.take(t1 + 0.5, t1 + wait)     # after the time a robot needs to brake
        # speed in the last part (m/s); stopped if below a quarter of the command
        a = np.array(got)
        speed = 0.0
        if len(a) > 2:
            speed = float(np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])).sum()) / max(float(a[-1, 0] - a[0, 0]), 1e-3)
        whole = np.array(self.take(t1, t1 + wait))
        moved = float(np.hypot(np.diff(whole[:, 1]), np.diff(whole[:, 2])).sum()) if len(whole) > 2 else 0.0
        results['coast'] = {'moved': moved, 'time': wait, 'stopped': speed < 0.25 * v}

    # =========================================================================
    # [LIFE]   run() -> exit code
    # -------------------------------------------------------------------------
    # Runs the stages and writes the report, whatever stopped them.
    # Returns 1 if there is any FAIL, else 0 (the console script's exit code).
    # The stop-then-analyse order matters: the robot is stopped before the
    # results are worked out.
    # =========================================================================
    def run(self):
        log = self.get_logger()
        found = self.still_stage()
        results, tips, stopped, out_of_room = {}, {}, None, None
        blocked = any(level == 'FAIL' for level, _, _ in found)
        if self.p['drive'] and blocked:
            stopped = 'not driving: fix the FAIL above first'
        elif self.p['drive']:
            try:
                self.drive_stage(results)
            except OutOfRoom as e:
                out_of_room = str(e)
            except Stop as e:
                stopped = str(e)
            finally:
                self.stop(0.5)
            more, tips = analyse(results, self.p['wheelbase'], self.p['ref_offset'], self.p['speed'],
                                 ARC_STEER, self.p['max_steer'])
            found += more
            if 'start_lag' in results:
                found.append(('INFO', 'Start lag', f"{results['start_lag']:.2f} s from command to 1 cm of motion"))
        if stopped:
            found.append(('FAIL', 'Stopped', stopped))
        if out_of_room:
            found.append(('WARN', 'Incomplete', out_of_room))
        stamp = time.strftime('%Y%m%d_%H%M%S')
        out = os.path.join(self.p['log_dir'], f'hw_check_{stamp}')
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, 'report.json'), 'w') as f:
            json.dump({'settings': self.p, 'findings': found, 'measured': results, 'suggestions': tips}, f, indent=2)
        lines = ['', '=' * 72, 'LIMO hardware check']
        lines += [f'  {level:5s} {name}: {text}' for level, name, text in found]
        if tips:
            lines.append('  Measured values to consider:')
            lines += [f'        {k} = {v}' for k, v in tips.items()]
        fails = sum(1 for level, _, _ in found if level == 'FAIL')
        warns = sum(1 for level, _, _ in found if level == 'WARN')
        if fails:
            verdict = f'{fails} FAIL: do not run tracking until this is fixed.'
        elif not self.p['drive']:
            verdict = 'Still checks done. Now run it with drive:=true, with clear space around the robot.'
        elif out_of_room and 'right' not in results:
            verdict = 'Incomplete: left and right were not both checked. Do not run tracking yet.'
        else:
            verdict = f'OK to run tracking ({warns} warning(s) to read).'
        lines += [f'  {verdict}', f'  Report: {out}/report.json', '=' * 72]
        log.info('\n'.join(lines))
        return 1 if fails else 0


# =============================================================================
# [LIFE]   main(args=None)
# -----------------------------------------------------------------------------
# Entry point. Spins the node in a thread, runs the check in the main thread,
# always sends zero commands at the end, and exits with run()'s code.
# =============================================================================
def main(args=None):
    init_ros(args)
    node = HwCheck()
    node.origin = (0.0, 0.0)    # replaced by the real start position when driving starts
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop(0.3)
        if rclpy.ok():
            rclpy.shutdown()    # ends the spin thread, which must finish before the node goes
        spin.join(timeout=2.0)
        node.destroy_node()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
