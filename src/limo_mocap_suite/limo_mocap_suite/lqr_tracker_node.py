# #############################################################################
# [FILE]   lqr_tracker_node.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/lqr_tracker_node.py
# ROLE  : Continuous-time LQR tracker, gain re-solved every step.
#
# Console script : ros2 run limo_mocap_suite lqr_tracker
# Law            : steer = atan(L*kappa) + k_cte*cte + k_h*heading_err
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
import scipy.linalg

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from limo_mocap_suite.trajectory_subscriber import TrajectorySubscriber
from limo_mocap_suite.obstacle_cbf import ObstacleCbfLayer
from limo_mocap_suite.run_manager import RunManager
from limo_mocap_suite.ros_compat import init_ros


# =============================================================================
# [CORE]   class LqrTrackerNode(Node)
# -----------------------------------------------------------------------------
# ROS 2 node 'lqr_tracker_node': path in, /cmd_vel out.
# =============================================================================
class LqrTrackerNode(Node):
    """Hardware-transferable Kinematic LQR Trajectory Tracker (Algebraic Riccati Equation)."""

    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Builds the LQR node:
    # - declares/reads ROS parameters (v_nominal, wheelbase,
    #   max_steer, q_cte, q_heading, r_steer, lookahead_pts, rezero_odom, ref_offset)
    # - creates TrajectorySubscriber (listens to /reference_path) and
    #   ObstacleCbfLayer (CBF)
    # - initialises pose and controller memory
    # - creates RunManager (approach, laps, guard, kill, logging, /cmd_vel)
    # - subscribers: /hardware_odom (Odometry), /hardware_obstacles (latched),
    #   /imu (Imu)
    # - timers     : control_loop @ 50 Hz (0.02 s)
    # =========================================================================
    def __init__(self):
        super().__init__('lqr_tracker_node')

        self.declare_parameter('v_nominal', 0.40)
        self.declare_parameter('wheelbase', 0.20)
        self.declare_parameter('ref_offset', 0.10)  # marker midpoint ahead of the rear axle (m)
        self.declare_parameter('max_steer', 0.4084)  # centre angle at the driver's 28 deg inner-wheel limit (R 0.46 m)
        self.declare_parameter('q_cte', 450.0)
        self.declare_parameter('q_heading', 35.0)
        self.declare_parameter('r_steer', 0.5)
        self.declare_parameter('lookahead_pts', 8)
        self.declare_parameter('rezero_odom', False)  # False: track the path in the absolute odom / MoCap frame

        v_nom = float(self.get_parameter('v_nominal').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.q_cte = float(self.get_parameter('q_cte').value)
        self.q_heading = float(self.get_parameter('q_heading').value)
        self.r_steer = float(self.get_parameter('r_steer').value)
        self.lookahead_pts = int(self.get_parameter('lookahead_pts').value)
        self.rezero_odom = bool(self.get_parameter('rezero_odom').value)

        self.traj = TrajectorySubscriber(self, v_nominal=v_nom,
                                         ref_offset=float(self.get_parameter('ref_offset').value))
        self._gain_cache = {}
        self.safety = ObstacleCbfLayer(self, self.wheelbase, self.max_steer, self.traj.ref_offset)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.x0 = None
        self.y0 = None
        self.yaw0 = None
        self.v_actual = 0.0
        self.odom_received = False


        self.run = RunManager(self, 'lqr', self.traj, self.wheelbase, self.max_steer)


        self.create_subscription(Odometry, '/hardware_odom', self.odom_cb, 20)
        self.create_subscription(Imu, '/imu', self.imu_cb, 20)

        self.control_timer = self.create_timer(0.02, self.control_loop)


    # =========================================================================
    # [UTIL]   quat_to_yaw(q) -> float
    # -------------------------------------------------------------------------
    # Converts a geometry_msgs Quaternion into a planar yaw angle (rad).
    # Uses the standard ZYX formula: atan2(2(wz+xy), 1-2(y^2+z^2)).
    # =========================================================================
    @staticmethod
    def quat_to_yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # =========================================================================
    # [ROS-IO] odom_cb(msg: Odometry)
    # -------------------------------------------------------------------------
    # Callback for /hardware_odom. Stores the robot pose used by control_loop.
    # 1. Reads raw (x, y, yaw) from the message.
    # 2. On the FIRST message, latches it as the origin (x0, y0, yaw0)
    #    (origin = 0 when the parameter rezero_odom is False).
    # 3. Expresses every later pose in that frame (translate + rotate), so by
    #    default the drawn path is RELATIVE to where the robot started.
    # 4. Sets odom_received = True (gate for control_loop).
    # Also caches v_actual = twist.linear.x for telemetry.
    # =========================================================================
    def odom_cb(self, msg: Odometry):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        raw_yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        if self.x0 is None:
            self.x0, self.y0, self.yaw0 = (raw_x, raw_y, raw_yaw) if self.rezero_odom else (0.0, 0.0, 0.0)
        dx = raw_x - self.x0
        dy = raw_y - self.y0
        c0, s0 = math.cos(self.yaw0), math.sin(self.yaw0)
        self.x = c0 * dx + s0 * dy
        self.y = -s0 * dx + c0 * dy
        self.yaw = math.atan2(math.sin(raw_yaw - self.yaw0), math.cos(raw_yaw - self.yaw0))
        self.v_actual = msg.twist.twist.linear.x
        self.odom_received = True
        self.run.note_odom()

    # =========================================================================
    # [ROS-IO] imu_cb(msg: Imu)
    # -------------------------------------------------------------------------
    # Callback for /imu. Intentionally empty (placeholder for yaw-rate
    #   damping).
    # =========================================================================
    def imu_cb(self, msg: Imu):
        pass

    # =========================================================================
    # [CORE]   _solve_lqr_gain(v, kappa) -> [k_cte, k_heading]
    # -------------------------------------------------------------------------
    # Linear error model e = [cte, heading_err], u = steer:
    #   A = [[0, v], [0, 0]],  B = [[0], [v*(1+(L*kappa)^2)/L]]
    # Solves the continuous algebraic Riccati equation (scipy) with
    # Q = diag(q_cte, q_heading), R = r_steer, returns K = R^-1 B^T P.
    # Falls back to K = [2.5, 1.2] if the solver throws.
    # =========================================================================
    def _solve_lqr_gain(self, v: float, kappa: float):
        """
        Computes LQR gain K = [k_cte, k_heading] using continuous ARE for error state:
          e_dot = A e + B u
        """
        L = self.wheelbase
        sec2 = 1.0 + (L * kappa) ** 2

        # State: [e_cte, e_heading]^T, Input: steering delta
        A = np.array([
            [0.0, v],
            [0.0, 0.0]
        ], dtype=np.float64)
        B = np.array([
            [0.0],
            [v * sec2 / L]
        ], dtype=np.float64)

        Q = np.diag([self.q_cte, self.q_heading])
        R = np.array([[self.r_steer]], dtype=np.float64)

        try:
            P = scipy.linalg.solve_continuous_are(A, B, Q, R)
            K = np.linalg.inv(R) @ (B.T @ P)
            return K[0]  # [k_cte, k_heading]
        except Exception:
            return np.array([2.5, 1.2])

    # =========================================================================
    # [CORE]   control_loop()
    # -------------------------------------------------------------------------
    # 50 Hz (0.02 s) timer callback. RunManager.tick() decides whether there is anything
    # to drive (it handles waiting, the approach, laps, the guard and the kill).
    # 1. RunManager.tick() -> cte, heading_err, ref point (None: nothing to do).
    # 2. K = solve_lqr_gain_cached(v, kappa); steer = ff + K.e; same speed law as PID.
    # 3. RunManager.shape(): guard speed factor.
    # 4. ObstacleCbfLayer.apply(): closest safe (v, steer) on the known obstacles.
    # 5. RunManager.command(): publish /cmd_vel, lap statistics, CSV row.
    # =========================================================================
    def control_loop(self):
        tick = self.run.tick(self.x, self.y, self.yaw, self.lookahead_pts)
        if tick is None:
            return
        cte, heading_err, ref_pt, idx, _dt = tick
        _x_ref, _y_ref, _yaw_ref, v_ref, kappa_ref = ref_pt

        K = self.solve_lqr_gain_cached(v_ref, kappa_ref)
        ff_steer = math.atan(self.wheelbase * kappa_ref)
        lqr_steer = ff_steer + (K[0] * cte + K[1] * heading_err)
        steer_cmd = float(np.clip(lqr_steer, -self.max_steer, self.max_steer))
        v_cmd = v_ref * max(0.4, 1.0 - 0.5 * abs(steer_cmd) / self.max_steer)

        v_cmd, steer_cmd = self.run.shape(v_cmd, steer_cmd, idx)
        v_safe, steer_safe, clearance, status = self.safety.apply(
            v_cmd, steer_cmd, self.x, self.y, self.yaw, self.x0, self.y0, self.yaw0
        )
        self.run.command(v_safe, steer_safe, clearance, status)


    # =========================================================================
    # [UTIL]   solve_lqr_gain_cached(v, kappa)
    # -------------------------------------------------------------------------
    # Memoises _solve_lqr_gain on (v rounded to 1 mm/s, kappa rounded to
    # 0.01 1/m), so repeated operating points skip the Riccati solve. The
    # cache is cleared once it holds 5000 entries.
    # =========================================================================
    def solve_lqr_gain_cached(self, v, kappa):
        key = (round(v, 3), round(kappa, 2))
        if key not in self._gain_cache:
            if len(self._gain_cache) >= 5000:
                self._gain_cache.clear()
            self._gain_cache[key] = self._solve_lqr_gain(*key)
        return self._gain_cache[key]

# =============================================================================
# [LIFE]   main(args=None)
# -----------------------------------------------------------------------------
# Console-script entry point (lqr_tracker).
# init rclpy -> construct node -> spin until Ctrl-C / SystemExit ->
# stop robot, destroy node, shutdown rclpy.
# =============================================================================
def main(args=None):
    # Plain Python SIGINT handling: Ctrl-C raises KeyboardInterrupt but keeps the
    # ROS context alive, so the shutdown code below can still publish.
    init_ros(args)  # Foxy + Humble: context stays alive on Ctrl-C
    node = LqrTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.run.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
