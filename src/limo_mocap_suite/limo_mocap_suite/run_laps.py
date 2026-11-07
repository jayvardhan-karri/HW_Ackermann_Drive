# #############################################################################
# [FILE]   run_laps.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/run_laps.py
# ROLE  : Runs the lap test: for each track radius, for each controller in turn,
#         start it, give it the capsule track (it drives itself to the race start
#         along a Dubins path), let it do its laps, wait buffer_time with the robot
#         stopped, then the next one. Finally builds the report (report_lap).
#         Only one controller runs at a time: each is a child process that
#         exits by itself (exit_on_finish) when its run is over.
#
# Console script : ros2 run limo_mocap_suite run_laps
# controllers    : 'pid,lqr,mpc,nmpc'; an entry 'mpc:constraint' runs that controller
#                  with that obstacle_mode (default for all: the obstacle_mode parameter)
# Normally       : record_lap.launch.py (one controller, 3 laps),
#                  compare_lap.launch.py (all four in turn),
#                  obstacle_lap.launch.py (add-on), all through run_laps.launch.py
# Log            : <log_dir>/<session_name>_<stamp>/R1/<run>/; log_dir '' = the
#                  workspace's evaluation/ folder (the one send_logs.sh sends)
# Publishes      : /reference_path, /cmd_vel (zero only, if a controller goes silent)
# Subscribes     : /run_status, /run_kill (latched), /hardware_odom
# Per run: obstacle mode 'cbf' (default) or 'mpc:constraint' (per entry, see
# controllers). Start side: start_side 'auto' uses track_tools.pick_side() at the
# moment each run starts, from the robot's y.
# Stops the whole test after a kill or a safety stop: the robot never starts moving
# again by itself after one of those. A run over its time limit (a robot that cannot
# get round a tight track) is recorded, and the test goes on.
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
import os
import signal
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from limo_mocap_suite.ros_compat import init_ros
from limo_mocap_suite.robot_topics import parse_controller_params
from limo_mocap_suite.track_tools import (LAB_X, LAB_Y, TRACK_STRAIGHT, TRACK_X, TRACK_Y, capsule,
                                          capsule_message, pick_side)

PACKAGE = 'limo_mocap_suite'


# =============================================================================
# [LIFE]   class LapRunner(Node)
# -----------------------------------------------------------------------------
# The test supervisor. It does not drive: it starts a controller process, sends
# the track, watches /run_status, and moves on when the run ends. If a controller
# goes silent it publishes zero speed. The spin thread (see main) runs the
# callbacks while run() blocks in its loops.
# =============================================================================
class LapRunner(Node):
    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Declares all test parameters (self.p), creates the topics and the state
    # the callbacks fill: killed, status (last /run_status), odom time, robot y.
    # =========================================================================
    def __init__(self):
        super().__init__('run_laps')
        defaults = {
            'controllers': 'pid,lqr,mpc,nmpc', 'laps': 3, 'guard': False, 'v_nominal': 0.30,
            'track_straight': TRACK_STRAIGHT, 'track_radii': '1.0', 'track_x': TRACK_X, 'track_y': TRACK_Y,
            'start_side': 'auto', 'direction': 'ccw', 'start_at': 'middle', 'buffer_time': 10.0, 'session_name': 'run_laps',
            'obstacle_mode': 'cbf', 'controller_params': '',
            'lab_x': list(LAB_X), 'lab_y': list(LAB_Y), 'log_dir': '', 'start_delay': 5.0,
            'cmd_topic': '/cmd_vel', 'imu_topic': '/imu', 'ref_offset': 0.10, 'wheelbase': 0.20,
            'max_steer': 0.4084, 'cbf_max_steer': 0.0, 'approach_radius': 0.60, 'approach_speed': 0.25, 'approach_spill': 0.30,
            'slow_cte': 0.15, 'rejoin_cte': 0.35, 'abort_cte': 1.00,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.path_pub = self.create_publisher(Path, '/reference_path', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/run_kill', self._kill_cb, latched)
        self.create_subscription(String, '/run_status', self._status_cb, 10)
        self.create_subscription(Odometry, '/hardware_odom', self._odom_cb, 10)
        self.killed = False
        self.status = None
        self.status_time = 0.0
        self.odom = None
        self.robot_y = None
        self.child = None

    # =========================================================================
    # [ROS-IO] _kill_cb(msg: Bool)  (helper: callback of /run_kill, used only in this class)
    # -------------------------------------------------------------------------
    # Remembers the KILL state; run_one() and buffer() look at it.
    # =========================================================================
    def _kill_cb(self, msg):
        self.killed = bool(msg.data)

    # =========================================================================
    # [ROS-IO] _status_cb(msg: String)  (helper: callback of /run_status, used only in this class)
    # -------------------------------------------------------------------------
    # Keeps the controller's last status (JSON) and when it arrived (s).
    # =========================================================================
    def _status_cb(self, msg):
        self.status = json.loads(msg.data)
        self.status_time = time.monotonic()

    # =========================================================================
    # [ROS-IO] _odom_cb(msg: Odometry)  (helper: callback of /hardware_odom, used only in this class)
    # -------------------------------------------------------------------------
    # Keeps the robot's y (m), for start_side auto, and that odometry is alive.
    # =========================================================================
    def _odom_cb(self, msg):
        self.odom = time.monotonic()
        self.robot_y = msg.pose.pose.position.y

    # =========================================================================
    # [LIFE]   start_controller(name, session, label, limit, mode=None)
    # -------------------------------------------------------------------------
    # Starts <name>_tracker as a child process with this test's settings. It leaves
    # by itself when its run is over (exit_on_finish).
    # session: log folder for the run; label: run name (e.g. R1); limit: time
    # limit in s (run_duration); mode: obstacle_mode for this run, None = the
    # obstacle_mode parameter.
    # HOW: builds the command line, turns every setting into -p key:=value and
    # remaps /cmd_vel and /imu to the configured topics. rezero_odom is off
    # because the track is in lab coordinates.
    # =========================================================================
    def start_controller(self, name, session, label, limit, mode=None):
        exe = os.path.join(get_package_prefix(PACKAGE), 'lib', PACKAGE, f'{name}_tracker')
        p = self.p
        params = {
            'laps': p['laps'], 'guard': p['guard'], 'run_duration': limit, 'v_nominal': p['v_nominal'], 'exit_on_finish': True,
            'log_dir': session, 'path_name': label, 'rezero_odom': False, 'ref_offset': p['ref_offset'],
            'wheelbase': p['wheelbase'], 'max_steer': p['max_steer'], 'cbf_max_steer': p['cbf_max_steer'],
            'approach_radius': p['approach_radius'], 'approach_speed': p['approach_speed'], 'approach_spill': p['approach_spill'],
            'slow_cte': p['slow_cte'], 'rejoin_cte': p['rejoin_cte'], 'abort_cte': p['abort_cte'],
            'guard_lab_x': p['lab_x'], 'guard_lab_y': p['lab_y'], 'obstacle_mode': mode or p['obstacle_mode'],
        }
        params.update(parse_controller_params(p['controller_params']))   # lab overrides, e.g. q_cte:=300
        args = [exe, '--ros-args', '-r', f"/cmd_vel:={p['cmd_topic']}", '-r', f"/imu:={p['imu_topic']}"]
        for key, value in params.items():
            text = str(value).lower() if isinstance(value, bool) else str(value)
            args += ['-p', f'{key}:={text}']
        self.status = None
        self.child = subprocess.Popen(args, stdin=subprocess.DEVNULL)

    # =========================================================================
    # [LIFE]   stop_controller()
    # -------------------------------------------------------------------------
    # Ends the child process: SIGINT (it logs the run as STOPPED and sends zero
    # commands), kill after 5 s if it does not leave. Then one zero command.
    # =========================================================================
    def stop_controller(self):
        if self.child is not None and self.child.poll() is None:
            self.child.send_signal(signal.SIGINT)
            try:
                self.child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.child.kill()
        self.child = None
        if rclpy.ok():
            self.cmd_pub.publish(Twist())

    # =========================================================================
    # [CORE]   run_one(name, session, track, label, limit, mode=None) -> (result, reason)
    # -------------------------------------------------------------------------
    # Waits for the controller to be ready, hands over the track, then watches it.
    # A controller that stops reporting gets a zero command and is stopped.
    # result: FINISHED, TIME_LIMIT, ABORTED, KILLED ... (as the run manager reports).
    # HOW: loop 1 waits up to 60 s for a /run_status from this controller. Loop 2
    # publishes the track every 3 s while the phase is WAIT_PATH and the run has
    # not started; once APPROACH or TRACK was seen it stops sending. It ends when
    # the child exits; no status for 1.5 s counts as stopped reporting.
    # =========================================================================
    def run_one(self, name, session, track, label, limit, mode=None):
        log = self.get_logger()
        self.start_controller(name, session, label, limit, mode)
        t0 = time.monotonic()
        while not (self.status and self.status['controller'] == name):
            if self.killed:
                return 'KILLED', 'kill button'
            if self.child.poll() is not None or time.monotonic() - t0 > 60.0:
                return 'ABORTED', f'{name} did not start'
            time.sleep(0.1)
        sent = 0.0
        started = False
        while self.child.poll() is None:
            now = time.monotonic()
            st = self.status
            if self.killed:
                return 'KILLED', 'kill button'
            if not started and st['phase'] == 'WAIT_PATH' and st['result'] is None and now - sent > 3.0:
                self.path_pub.publish(track)
                sent = now
            if st['phase'] in ('APPROACH', 'TRACK'):
                started = True
            if now - self.status_time > 1.5:
                self.cmd_pub.publish(Twist())
                log.error(f'{name} stopped reporting: robot stopped')
                return 'ABORTED', f'{name} stopped reporting'
            time.sleep(0.1)
        st = self.status
        if st['controller'] == name and st['result']:
            return st['result'], st['reason']
        return 'ABORTED', f'{name} exited early (code {self.child.returncode})'

    # =========================================================================
    # [UTIL]   log_root() -> folder the test folders go in
    # -------------------------------------------------------------------------
    # log_dir if set; otherwise <workspace>/evaluation, found from the install
    # folder, so the logs land in the same place whatever folder it is started from.
    # =========================================================================
    def log_root(self):
        if self.p['log_dir']:
            return os.path.abspath(os.path.expanduser(self.p['log_dir']))
        install = os.path.dirname(get_package_prefix(PACKAGE))
        if os.path.basename(install) != 'install':      # merged or unusual layout
            install = get_package_prefix(PACKAGE)
        if os.path.basename(install) == 'install':
            return os.path.join(os.path.dirname(install), 'evaluation')
        return os.path.abspath('evaluation')

    # =========================================================================
    # [LIFE]   buffer(seconds, next_name)
    # -------------------------------------------------------------------------
    # Pause between controllers (buffer_time, s) with the robot held at zero: a
    # zero command every 0.2 s. KILL ends it.
    # =========================================================================
    def buffer(self, seconds, next_name):
        log = self.get_logger()
        log.warn(f'Pause {seconds:.0f} s, robot stopped. Next: {next_name}, which drives itself to the start. '
                 'KILL in the live view cancels the rest.')
        end = time.monotonic() + seconds
        while time.monotonic() < end and not self.killed:
            self.cmd_pub.publish(Twist())
            time.sleep(0.2)

    # =========================================================================
    # [CORE]   run()
    # -------------------------------------------------------------------------
    # The whole test. Writes settings.json, results.json and calls report_lap.
    # HOW: 1. check every track fits the lab, before anything moves. 2. wait for
    # /hardware_odom, then start_delay seconds of warning. 3. for each radius,
    # for each controller entry: buffer pause (not before the first), pick the
    # start side, build the track, run_one(), stop the controller. 4. stop at the
    # first result that is not FINISHED or TIME_LIMIT (kill, safety stop). 5. run
    # the report on the session folder.
    # The time limit for a run is 60 s plus 5 times the time needed for all laps
    # at 40 % of v_nominal, so a robot that cannot get round a tight track ends.
    # =========================================================================
    def run(self):
        log = self.get_logger()
        stamp = time.strftime('%Y%m%d_%H%M%S')
        session = os.path.join(self.log_root(), f"{self.p['session_name']}_{stamp}")
        os.makedirs(session, exist_ok=True)
        with open(os.path.join(session, 'settings.json'), 'w') as f:
            json.dump(self.p, f, indent=2)
        lab_x, lab_y = self.p['lab_x'], self.p['lab_y']
        radii = [float(r) for r in str(self.p['track_radii']).split(',') if r.strip()]
        straight, cx, cy = self.p['track_straight'], self.p['track_x'], self.p['track_y']
        side, way, at = self.p['start_side'], self.p['direction'], self.p['start_at']
        for r in radii:          # fails here, before anything moves, if a track does not fit the lab
            capsule_message(straight, r, cx, cy, self.get_clock().now().to_msg(), lab_x, lab_y, 'top', way, at)
        names = [n.strip().lower() for n in self.p['controllers'].split(',') if n.strip()]
        log.info(f"Lap test: radii {radii}, {names}, {self.p['laps']} laps each, start {side} {at} {way}, "
                 f"guard={self.p['guard']}. Log: {session}")
        while self.odom is None:
            log.info('Waiting for /hardware_odom (is the robot tracked?) ...', throttle_duration_sec=3.0)
            time.sleep(0.2)
        log.warn(f"The robot will start moving in {self.p['start_delay']:.0f} s. Stop it with KILL in the live view.")
        time.sleep(float(self.p['start_delay']))
        results = {}
        stop = False
        for radius in radii:
            label = f'R{radius:g}'
            folder = os.path.join(session, label)
            os.makedirs(folder, exist_ok=True)
            length = len(capsule(straight, radius, cx, cy)[0]) * 0.01     # points x 1 cm spacing = track length, m
            limit = 60.0 + 5.0 * int(self.p['laps']) * length / max(float(self.p['v_nominal']) * 0.4, 0.05)
            for i, entry in enumerate(names):
                name, _, mode = entry.partition(':')     # 'mpc:constraint' -> name 'mpc', obstacle_mode 'constraint'
                if i > 0:
                    self.buffer(float(self.p['buffer_time']), entry)
                if self.killed:
                    results[f'{label}_{entry}'] = ('KILLED', 'kill button')
                    stop = True
                    break
                log.info(f'--- {label} {entry}: starting ---')
                chosen = pick_side(side, self.robot_y, cy)     # auto: top, or bottom if the robot is nearer it
                log.info(f'Start: middle of the {chosen} straight' if at == 'middle' else
                         f'Start: beginning of the {chosen} straight')
                track = capsule_message(straight, radius, cx, cy, self.get_clock().now().to_msg(), lab_x, lab_y,
                                        chosen, way, at)
                got = self.run_one(name, folder, track, label, limit, mode or None)
                results[f'{label}_{entry}'] = got
                self.stop_controller()
                log.info(f'--- {label} {entry}: {got[0]} ({got[1]}) ---')
                if got[0] not in ('FINISHED', 'TIME_LIMIT'):
                    log.error('Lap test stopped here. Check the robot, then run it again with '
                              'track_radii:=... controllers:=... for the ones left.')
                    stop = True
                    break
            if stop:
                break
        with open(os.path.join(session, 'results.json'), 'w') as f:
            json.dump(results, f, indent=2)
        exe = os.path.join(get_package_prefix(PACKAGE), 'lib', PACKAGE, 'report_lap')
        code = subprocess.call([exe, session])
        if code == 0:
            log.info(f"Report: {os.path.join(session, 'report.md')}")
        else:
            log.error(f'Report failed (code {code}); the logs are in {session}')


# =============================================================================
# [LIFE]   main(args=None)  -> console script run_laps
# -----------------------------------------------------------------------------
# Spins the node in a thread so its callbacks work while run() blocks.
# A plain rclpy.spin thread left running at exit made the process abort with
# "terminate called without an active exception"; the executor is stopped and
# the thread joined before the node goes away.
# =============================================================================
def main(args=None):
    init_ros(args)
    node = LapRunner()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_controller()
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
