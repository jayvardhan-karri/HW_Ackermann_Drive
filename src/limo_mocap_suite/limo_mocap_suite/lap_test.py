# #############################################################################
# [FILE]   lap_test.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/lap_test.py
# ROLE  : Runs the lap test: for each track radius, for each controller in turn,
#         start it, give it the capsule track (it drives itself to the start), let
#         it do its laps, then the next one. Finally builds the report.
#
# Console script : ros2 run limo_mocap_suite lap_test   (normally: lap_test.launch.py)
# Publishes      : /reference_path, /cmd_vel (zero only, if a controller goes silent)
# Subscribes     : /run_status, /run_kill (latched), /hardware_odom
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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from limo_mocap_suite.ros_compat import init_ros
from limo_mocap_suite.track_tools import capsule, capsule_message

PACKAGE = 'limo_mocap_suite'


class LapTest(Node):
    def __init__(self):
        super().__init__('lap_test')
        defaults = {
            'controllers': 'pid,lqr,mpc,nmpc', 'laps': 3, 'guard': False, 'v_nominal': 0.30,
            'track_straight': 3.0, 'track_radii': '1.0,0.5,0.4', 'track_x': 3.75, 'track_y': 2.5,
            'lab_x': [-0.5, 8.0], 'lab_y': [-1.0, 6.0], 'log_dir': 'evaluation', 'start_delay': 5.0,
            'cmd_topic': '/cmd_vel', 'imu_topic': '/imu', 'ref_offset': 0.10, 'wheelbase': 0.20,
            'max_steer': 0.4084, 'cbf_max_steer': 0.0, 'approach_radius': 0.60, 'approach_speed': 0.25,
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
        self.child = None

    def _kill_cb(self, msg):
        self.killed = bool(msg.data)

    def _status_cb(self, msg):
        self.status = json.loads(msg.data)
        self.status_time = time.monotonic()

    def _odom_cb(self, msg):
        self.odom = time.monotonic()

    # =========================================================================
    # [LIFE]   start_controller(name, session)
    # -------------------------------------------------------------------------
    # Starts <name>_tracker as a child process with this test's settings. It leaves
    # by itself when its run is over (exit_on_finish).
    # =========================================================================
    def start_controller(self, name, session, label, limit):
        exe = os.path.join(get_package_prefix(PACKAGE), 'lib', PACKAGE, f'{name}_tracker')
        p = self.p
        params = {
            'laps': p['laps'], 'guard': p['guard'], 'run_duration': limit, 'v_nominal': p['v_nominal'], 'exit_on_finish': True,
            'log_dir': session, 'path_name': label, 'rezero_odom': False, 'ref_offset': p['ref_offset'],
            'wheelbase': p['wheelbase'], 'max_steer': p['max_steer'], 'cbf_max_steer': p['cbf_max_steer'],
            'approach_radius': p['approach_radius'], 'approach_speed': p['approach_speed'],
            'slow_cte': p['slow_cte'], 'rejoin_cte': p['rejoin_cte'], 'abort_cte': p['abort_cte'],
            'guard_lab_x': p['lab_x'], 'guard_lab_y': p['lab_y'],
        }
        args = [exe, '--ros-args', '-r', f"/cmd_vel:={p['cmd_topic']}", '-r', f"/imu:={p['imu_topic']}"]
        for key, value in params.items():
            text = str(value).lower() if isinstance(value, bool) else str(value)
            args += ['-p', f'{key}:={text}']
        self.status = None
        self.child = subprocess.Popen(args, stdin=subprocess.DEVNULL)

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
    # [CORE]   run_one(name, session, track) -> (result, reason)
    # -------------------------------------------------------------------------
    # Waits for the controller to be ready, hands over the track, then watches it.
    # A controller that stops reporting gets a zero command and is stopped.
    # =========================================================================
    def run_one(self, name, session, track, label, limit):
        log = self.get_logger()
        self.start_controller(name, session, label, limit)
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

    def run(self):
        log = self.get_logger()
        stamp = time.strftime('%Y%m%d_%H%M%S')
        session = os.path.abspath(os.path.join(self.p['log_dir'], f'lap_test_{stamp}'))
        os.makedirs(session, exist_ok=True)
        with open(os.path.join(session, 'settings.json'), 'w') as f:
            json.dump(self.p, f, indent=2)
        lab_x, lab_y = self.p['lab_x'], self.p['lab_y']
        radii = [float(r) for r in str(self.p['track_radii']).split(',') if r.strip()]
        straight, cx, cy = self.p['track_straight'], self.p['track_x'], self.p['track_y']
        tracks = {r: capsule_message(straight, r, cx, cy, self.get_clock().now().to_msg(), lab_x, lab_y) for r in radii}
        names = [n.strip().lower() for n in self.p['controllers'].split(',') if n.strip()]
        log.info(f"Lap test: radii {radii}, {names}, {self.p['laps']} laps each, "
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
            length = len(capsule(straight, radius, cx, cy)[0]) * 0.01
            limit = 60.0 + 5.0 * int(self.p['laps']) * length / max(float(self.p['v_nominal']) * 0.4, 0.05)
            for name in names:
                if self.killed:
                    results[f'{label}_{name}'] = ('KILLED', 'kill button')
                    stop = True
                    break
                log.info(f'--- {label} {name}: starting ---')
                got = self.run_one(name, folder, tracks[radius], label, limit)
                results[f'{label}_{name}'] = got
                self.stop_controller()
                log.info(f'--- {label} {name}: {got[0]} ({got[1]}) ---')
                if got[0] not in ('FINISHED', 'TIME_LIMIT'):
                    log.error('Lap test stopped here. Check the robot, then run it again with '
                              'track_radii:=... controllers:=... for the ones left.')
                    stop = True
                    break
                time.sleep(2.0)
            if stop:
                break
        with open(os.path.join(session, 'results.json'), 'w') as f:
            json.dump(results, f, indent=2)
        exe = os.path.join(get_package_prefix(PACKAGE), 'lib', PACKAGE, 'lap_report')
        code = subprocess.call([exe, session])
        if code == 0:
            log.info(f"Report: {os.path.join(session, 'report.md')}")
        else:
            log.error(f'Report failed (code {code}); the logs are in {session}')


def main(args=None):
    init_ros(args)
    node = LapTest()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_controller()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
