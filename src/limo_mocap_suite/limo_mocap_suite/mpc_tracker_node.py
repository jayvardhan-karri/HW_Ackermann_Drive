# #############################################################################
# [FILE]   mpc_tracker_node.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/mpc_tracker_node.py
# ROLE  : LTV error-state MPC solved by backward Riccati recursion.
#
# Console script : ros2 run limo_mocap_suite mpc_tracker
# State          : z = [cte, d(cte)/dt, heading_err, previous steer deviation]
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

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from limo_mocap_suite.trajectory_subscriber import TrajectorySubscriber
from limo_mocap_suite.obstacle_cbf import ObstacleCbfLayer
from limo_mocap_suite.mpc_obstacles import (LinearObstacleMpc, clearance, sample_reference, speed_for_plan,
                                            world_obstacles)
from limo_mocap_suite.run_manager import RunManager
from limo_mocap_suite.ros_compat import init_ros


# =============================================================================
# [CORE]   class MpcTrackerNode(Node)
# -----------------------------------------------------------------------------
# ROS 2 node 'mpc_tracker_node': path in (/reference_path), /cmd_vel out.
# Linear time-varying MPC on the error state z = [cte, d(cte)/dt, heading_err,
# previous steering deviation]. The finite-horizon problem is solved in closed
# form by a backward Riccati recursion (no QP solver), every 20 ms.
# Two obstacle modes (parameter obstacle_mode):
#   cbf        MPC first, then ObstacleCbfLayer filters the command (default)
#   constraint obstacles inside the optimisation (LinearObstacleMpc, mpc_obstacles.py)
# Sign convention: cte > 0 = path on the robot's left; steer > 0 = turn left.
# =============================================================================
class MpcTrackerNode(Node):
    """
    Hardware-transferable LTV Riccati Kinematic Error-State MPC Tracker.
    Solves the finite-horizon discrete Riccati recursion on the 4D augmented Frenet
    state x_e = [e_cte, d(e_cte)/dt, e_heading, u_{k-1}]^T at 50 Hz (< 0.5 ms).
    """

    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Builds the MPC node:
    # - declares/reads ROS parameters, defaults: v_nominal 0.40 m/s, wheelbase 0.20 m,
    #   ref_offset 0.10 m, max_steer 0.4084 rad, horizon_steps 16, mpc_dt 0.04 s
    #   (horizon 0.64 s), q_cte 520, q_dcte 35, q_heading 42, r_steer 0.28,
    #   rd_steer 0.45, lookahead_pts 9, rezero_odom False, obstacle_mode 'cbf'
    # - creates TrajectorySubscriber (listens to /reference_path) and
    #   ObstacleCbfLayer (CBF; it also declares robot_radius and safety_margin,
    #   which give keep_out = robot_radius + safety_margin)
    # - initialises pose and controller memory (prev_steer, prev_cte, int_cte)
    # - creates LinearObstacleMpc (used only in constraint mode)
    # - creates RunManager (approach, laps, guard, kill, logging, /cmd_vel);
    #   its on_start callback is _reset_memory
    # - subscribers: /hardware_odom (Odometry), /hardware_obstacles (latched, made by
    #   ObstacleCbfLayer), /imu (Imu)
    # - timers     : control_loop @ 50 Hz (0.02 s)
    # =========================================================================
    def __init__(self):
        super().__init__('mpc_tracker_node')

        self.declare_parameter('v_nominal', 0.40)
        self.declare_parameter('wheelbase', 0.20)
        self.declare_parameter('ref_offset', 0.10)  # marker midpoint ahead of the rear axle (m)
        self.declare_parameter('max_steer', 0.4084)  # centre angle at the driver's 28 deg inner-wheel limit (R 0.46 m)
        self.declare_parameter('horizon_steps', 16)
        self.declare_parameter('mpc_dt', 0.04)
        self.declare_parameter('q_cte', 520.0)
        self.declare_parameter('q_dcte', 35.0)
        self.declare_parameter('q_heading', 42.0)
        self.declare_parameter('r_steer', 0.28)
        self.declare_parameter('rd_steer', 0.45)
        self.declare_parameter('lookahead_pts', 9)
        self.declare_parameter('rezero_odom', False)  # False: track the path in the absolute odom / MoCap frame

        v_nom = float(self.get_parameter('v_nominal').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.N = int(self.get_parameter('horizon_steps').value)
        self.mpc_dt = float(self.get_parameter('mpc_dt').value)
        self.q_cte = float(self.get_parameter('q_cte').value)
        self.q_dcte = float(self.get_parameter('q_dcte').value)
        self.q_heading = float(self.get_parameter('q_heading').value)
        self.r_steer = float(self.get_parameter('r_steer').value)
        self.rd_steer = float(self.get_parameter('rd_steer').value)
        self.lookahead_pts = int(self.get_parameter('lookahead_pts').value)
        self.rezero_odom = bool(self.get_parameter('rezero_odom').value)

        self.traj = TrajectorySubscriber(self, v_nominal=v_nom,
                                         ref_offset=float(self.get_parameter('ref_offset').value))
        self.safety = ObstacleCbfLayer(self, self.wheelbase, self.max_steer, self.traj.ref_offset)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.x0 = None
        self.y0 = None
        self.yaw0 = None
        self.v_actual = 0.0
        self.odom_received = False

        self.prev_steer = 0.0
        self.prev_cte = 0.0
        self.int_cte = 0.0

        self.declare_parameter('obstacle_mode', 'cbf')  # cbf: filter after the MPC; constraint: inside it
        self.obstacle_mode = str(self.get_parameter('obstacle_mode').value)
        self.keep_out = float(self.get_parameter('robot_radius').value) + float(self.get_parameter('safety_margin').value)
        self.obstacle_mpc = LinearObstacleMpc(self.wheelbase, self.max_steer, self.keep_out)
        self.run = RunManager(self, 'mpc', self.traj, self.wheelbase, self.max_steer,
                              on_start=self._reset_memory)


        self.create_subscription(Odometry, '/hardware_odom', self.odom_cb, 20)
        self.create_subscription(Imu, '/imu', self.imu_cb, 20)

        self.control_timer = self.create_timer(0.02, self.control_loop)


    # =========================================================================
    # [UTIL]   quat_to_yaw(q) -> yaw [rad]
    # -------------------------------------------------------------------------
    # Converts a geometry_msgs Quaternion (x, y, z, w) into a planar yaw (rad).
    # Standard ZYX formula: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2)).
    # Static: no node state is used. Result is in (-pi, pi].
    # =========================================================================
    @staticmethod
    def quat_to_yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # =========================================================================
    # [ROS-IO] odom_cb(msg: Odometry)
    # -------------------------------------------------------------------------
    # Callback for /hardware_odom. Stores the robot pose used by control_loop.
    # Pose is the TRACKED POINT (marker midpoint, ref_offset ahead of the rear axle).
    # 1. Reads raw (x, y, yaw) from the message (m, m, rad).
    # 2. On the FIRST message, latches the origin (x0, y0, yaw0). The origin is the
    #    first pose if rezero_odom is True, else (0, 0, 0): the absolute odom / MoCap
    #    frame, which is the default.
    # 3. Expresses every pose in that frame: translate by (x0, y0), rotate by -yaw0.
    #    With the default origin this changes nothing.
    # 4. Sets odom_received = True and tells RunManager (note_odom) that odom is alive.
    # Also caches v_actual = twist.linear.x (m/s) for telemetry.
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
        # rotate the offset by -yaw0 (world frame -> controller frame)
        self.x = c0 * dx + s0 * dy
        self.y = -s0 * dx + c0 * dy
        # wrap the yaw difference to (-pi, pi]
        self.yaw = math.atan2(math.sin(raw_yaw - self.yaw0), math.cos(raw_yaw - self.yaw0))
        self.v_actual = msg.twist.twist.linear.x
        self.odom_received = True
        self.run.note_odom()

    # =========================================================================
    # [ROS-IO] imu_cb(msg: Imu)
    # -------------------------------------------------------------------------
    # Callback for /imu. Intentionally empty: placeholder for yaw-rate damping.
    # The message is received but not used by the control law.
    # =========================================================================
    def imu_cb(self, msg: Imu):
        pass

    # =========================================================================
    # [CORE]   _solve_ltv_mpc(cte, d_cte, heading_err, nearest_idx) -> (v, steer)
    #          (helper of control_loop)
    # -------------------------------------------------------------------------
    # Inputs: cte (m), d_cte (m/s), heading_err (rad), nearest path index.
    # Outputs: speed v (m/s) and steering (rad) for this tick.
    # 1. Sample N reference points ahead (row k = [x, y, yaw, v, kappa]), advancing
    #    v_k*dt of arc length per step. Row 0 is the nearest point.
    # 2. Backward Riccati recursion k = N-1..0 on z = [cte, d_cte, heading_err, prev_du],
    #    with step-dependent A_k, B_k built from v_k, kappa_k. Cost: Q = diag(q_cte,
    #    q_dcte, q_heading, rd_steer), terminal P = 3Q, input weight R_eff = r_steer +
    #    rd_steer. Per step: S = R_eff + B^T P B, K_k = B^T P A / S,
    #    P <- Q + A^T P (A - B K_k). Only K_0 is used (receding horizon).
    # 3. Feed-forward = weighted mean of atan(L*kappa_k) over the first 5 steps
    #    (weights 0.45, 0.25, 0.15, 0.10, 0.05).
    # 4. du = -K_0 . z;  steer = ff + du + 0.06*int(cte); clipped to +-max_steer.
    #    z[3] = previous steering minus ff, so the steering rate is penalised.
    # 5. Speed: same heuristic profile as PID (fair comparison).
    # =========================================================================
    def _solve_ltv_mpc(self, cte: float, d_cte: float, heading_err: float, nearest_idx: int):
        """
        Solves the finite-horizon LTV-MPC problem over N steps using backward Riccati
        recursion on the 4-state Frenet error model:
          z_k = [e_cte, d(e_cte)/dt, e_heading, du_{k-1}]^T
        """
        dt = self.mpc_dt
        L = self.wheelbase

        # 1. Sample time-varying reference horizon (v_k, kappa_k) along trajectory
        # rows: [x, y, yaw, v, kappa]
        ref_horizon = np.zeros((self.N, 5), dtype=np.float64)
        curr_idx = float(nearest_idx)
        for k in range(self.N):
            idx_int = self.traj.index(round(curr_idx))
            pt_k = self.traj.waypoints[idx_int]
            nxt_pt = self.traj.waypoints[self.traj.index(idx_int + 1)]
            ds = max(math.hypot(nxt_pt[0] - pt_k[0], nxt_pt[1] - pt_k[1]), 1e-3)
            ref_horizon[k] = pt_k
            # index step = (distance driven in dt) / (waypoint spacing)
            curr_idx += (pt_k[3] * dt) / ds

        # 2. Backward Riccati recursion on z_k = [cte, d_cte, heading_err, prev_du]^T
        Q = np.diag([self.q_cte, self.q_dcte, self.q_heading, self.rd_steer])
        # terminal cost: 3x the stage cost
        P = Q * 3.0

        K_seq = np.zeros((self.N, 4), dtype=np.float64)
        ff_steers = np.zeros(self.N, dtype=np.float64)

        for k in range(self.N - 1, -1, -1):
            # floor the speed (m/s): the model degenerates near 0
            vk = max(float(ref_horizon[k, 3]), 0.15)
            kk = float(ref_horizon[k, 4])
            ff_steers[k] = math.atan(L * kk)
            sec2 = 1.0 + (L * kk) ** 2
            # heading change per rad of steering per step: v*dt*sec^2/L
            b_yaw = (vk * dt * sec2) / L

            # z_next = A z + B du. The 0.35 and 0.20 are empirical damping gains of the
            # lateral rows (not derived); the last row is zero: prev_du becomes the new du.
            A = np.array([
                [1.0, 0.35 * dt, vk * dt, 0.0],
                [0.0, 0.20,      vk,      0.0],
                [0.0, 0.0,       1.0,     0.0],
                [0.0, 0.0,       0.0,     0.0]
            ], dtype=np.float64)
            B = np.array([
                # sign: steering left reduces "path minus robot" errors
                [-0.5 * vk * dt * b_yaw],
                [-vk * b_yaw / dt],
                [-b_yaw],
                [1.0]
            ], dtype=np.float64)

            # scalar input weight: size + rate of change lumped together
            R_eff = self.r_steer + self.rd_steer
            S = float(R_eff + (B.T @ P @ B)[0, 0])
            K_k = (B.T @ P @ A) / S
            K_seq[k] = K_k[0]
            P = Q + A.T @ P @ (A - B @ K_k)

        # 3. Preview feedforward over upcoming horizon + optimal Riccati state feedback K_0
        weights = np.array([0.45, 0.25, 0.15, 0.10, 0.05])
        d_ff_preview = float(np.dot(weights, ff_steers[:5]))

        prev_du = self.prev_steer - d_ff_preview
        state_vec = np.array([cte, d_cte, heading_err, prev_du], dtype=np.float64)
        du_opt = -float(np.dot(K_seq[0], state_vec))

        # 0.06 = small integral gain (int_cte is clamped to +-0.5 m*s)
        steer_opt = d_ff_preview + du_opt + 0.06 * self.int_cte
        steer_opt = float(np.clip(steer_opt, -self.max_steer, self.max_steer))

        # Match speed profile with PID for exact apples-to-apples speed comparison
        v_ref_0 = float(ref_horizon[0, 3])
        v_opt = float(v_ref_0 * max(0.4, 1.0 - 0.5 * abs(steer_opt) / self.max_steer))

        return v_opt, steer_opt

    # =========================================================================
    # [CORE]   control_loop()
    # -------------------------------------------------------------------------
    # 50 Hz (0.02 s) timer callback. RunManager.tick() decides whether there is anything
    # to drive (it handles waiting, the approach, laps, the guard and the kill).
    # 1. RunManager.tick() -> cte (m), heading_err (rad), nearest index, dt (s).
    #    None means nothing to do this tick.
    # 2. d(cte)/dt (backward difference), integral of cte clamped to +-0.5 m*s, then
    #    _solve_ltv_mpc() (timed in ms, logged as solve_ms).
    #    In obstacle_mode 'constraint' it calls _constraint_step() instead.
    # 3. RunManager.shape(): guard speed factor.
    # 4. ObstacleCbfLayer.apply(): closest safe (v, steer) on the known obstacles.
    # 5. RunManager.command(): publish /cmd_vel, lap statistics, CSV row, then
    #    prev_steer = the steering actually sent (after shape and CBF).
    # =========================================================================
    def control_loop(self):
        tick = self.run.tick(self.x, self.y, self.yaw, self.lookahead_pts)
        if tick is None:
            return
        cte, heading_err, _ref_pt, nearest_idx, dt = tick

        d_cte = (cte - self.prev_cte) / dt
        self.prev_cte = cte
        self.int_cte = float(np.clip(self.int_cte + cte * dt, -0.5, 0.5))

        if self.obstacle_mode == 'constraint':
            self._constraint_step(cte, heading_err, nearest_idx)
            return

        t0 = time.perf_counter()
        v_mpc, steer_mpc = self._solve_ltv_mpc(cte, d_cte, heading_err, nearest_idx)
        solve_ms = (time.perf_counter() - t0) * 1000.0

        v_mpc, steer_mpc = self.run.shape(v_mpc, steer_mpc, nearest_idx)
        v_safe, steer_safe, clearance, status = self.safety.apply(
            v_mpc, steer_mpc, self.x, self.y, self.yaw, self.x0, self.y0, self.yaw0
        )
        self.run.command(v_safe, steer_safe, clearance, status, solve_ms)
        self.prev_steer = self.run.steer

    # =========================================================================
    # [CORE]   _constraint_step(cte, heading_err, nearest_idx)
    #          (helper of control_loop)
    # -------------------------------------------------------------------------
    # obstacle_mode:=constraint: obstacles as bounds in the QP (mpc_obstacles.py).
    # The CBF filter is not used. Steps:
    # 1. Obstacles into the controller frame (world_obstacles), 3 s reference (sample_reference).
    # 2. LinearObstacleMpc.solve() -> steering (rad) and slack (m; > 0 = no safe plan).
    # 3. Speed from the reference with the PID speed law, then RunManager.shape().
    # 4. speed_for_plan(): slows or stops if the slack says the plan is not safe.
    # 5. RunManager.command() and prev_steer, as in control_loop.
    # =========================================================================
    def _constraint_step(self, cte, heading_err, nearest_idx):
        obstacles = world_obstacles(self.safety, self.x0, self.y0, self.yaw0)
        ref = sample_reference(self.traj, nearest_idx)
        t0 = time.perf_counter()
        steer, slack = self.obstacle_mpc.solve(cte, heading_err, ref, self.prev_steer, obstacles)
        solve_ms = (time.perf_counter() - t0) * 1000.0
        v = float(self.traj.waypoints[nearest_idx, 3]) * max(0.4, 1.0 - 0.5 * abs(steer) / self.max_steer)
        v, steer = self.run.shape(v, steer, nearest_idx)
        clear = clearance(self.x, self.y, obstacles)
        v, status = speed_for_plan(v, slack, clear, self.keep_out)
        self.run.command(v, steer, clear, status, solve_ms)
        self.prev_steer = self.run.steer

    # =========================================================================
    # [LIFE]   _reset_memory()  (helper: only used in this class, as RunManager on_start)
    # -------------------------------------------------------------------------
    # Start of a run: clear the integral, the previous cte and the previous steering,
    # and the obstacle MPC state (committed passing sides, warm start).
    # =========================================================================
    def _reset_memory(self):
        self.int_cte = 0.0
        self.prev_cte = 0.0
        self.prev_steer = 0.0
        self.obstacle_mpc.reset()


# =============================================================================
# [LIFE]   main(args=None)
# -----------------------------------------------------------------------------
# Console-script entry point (mpc_tracker).
# init rclpy -> construct MpcTrackerNode -> spin until Ctrl-C / SystemExit ->
# stop robot (node.run.shutdown), destroy node, shutdown rclpy.
# init_ros keeps the ROS context alive on Ctrl-C, so the stop command can still
# be published in the finally block.
# =============================================================================
def main(args=None):
    # Plain Python SIGINT handling: Ctrl-C raises KeyboardInterrupt but keeps the
    # ROS context alive, so the shutdown code below can still publish.
    init_ros(args)  # Foxy + Humble: context stays alive on Ctrl-C
    node = MpcTrackerNode()
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
