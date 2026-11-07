# #############################################################################
# [FILE]   nmpc_tracker_node.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/nmpc_tracker_node.py
# ROLE  : Nonlinear MPC on the bicycle model, solved with SciPy SLSQP.
#
# Console script : ros2 run limo_mocap_suite nmpc_tracker
# Decision vars  : steering sequence u[0..N-1] (speed is NOT optimised)
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
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from limo_mocap_suite.trajectory_subscriber import TrajectorySubscriber
from limo_mocap_suite.obstacle_cbf import ObstacleCbfLayer
from limo_mocap_suite.mpc_obstacles import (NonlinearObstacleMpc, clearance, sample_reference, speed_for_plan,
                                            world_obstacles)
from limo_mocap_suite.run_manager import RunManager
from limo_mocap_suite.ros_compat import init_ros


# =============================================================================
# [CORE]   class NmpcTrackerNode(Node)
# -----------------------------------------------------------------------------
# ROS 2 node 'nmpc_tracker_node': path in (/reference_path), /cmd_vel out.
# Nonlinear MPC on the kinematic bicycle model, solved each tick by SciPy SLSQP.
# Decision variables: steering sequence u[0..N-1] (rad). Speed is not optimised;
# it follows the reference profile. Runs at 25 Hz because SLSQP is heavier.
# Two obstacle modes (parameter obstacle_mode):
#   cbf        NMPC first, then ObstacleCbfLayer filters the command (default)
#   constraint obstacles as solver constraints (NonlinearObstacleMpc, mpc_obstacles.py)
# Sign convention: cte > 0 = path on the robot's left; steer > 0 = turn left.
# =============================================================================
class NmpcTrackerNode(Node):
    """Hardware-transferable Full Nonlinear MPC (SLSQP)."""

    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Builds the NMPC node:
    # - declares/reads ROS parameters, defaults: v_nominal 0.40 m/s, wheelbase 0.20 m,
    #   ref_offset 0.10 m, max_steer 0.4084 rad, horizon_steps 10, mpc_dt 0.08 s
    #   (horizon 0.8 s), q_cte 40, q_heading 8, r_steer 0.8 (not used), rd_steer 2.5,
    #   rezero_odom False, obstacle_mode 'cbf'
    # - creates TrajectorySubscriber (listens to /reference_path) and
    #   ObstacleCbfLayer (CBF; it also declares robot_radius and safety_margin,
    #   which give keep_out = robot_radius + safety_margin)
    # - initialises pose and controller memory (u_guess warm start, prev_steer)
    # - creates NonlinearObstacleMpc (used only in constraint mode)
    # - creates RunManager (approach, laps, guard, kill, logging, /cmd_vel);
    #   its on_start callback is _reset_memory
    # - subscribers: /hardware_odom (Odometry), /hardware_obstacles (latched, made by
    #   ObstacleCbfLayer)
    # - timers     : control_loop @ 25 Hz (0.04 s)
    # =========================================================================
    def __init__(self):
        super().__init__('nmpc_tracker_node')

        self.declare_parameter('v_nominal', 0.40)
        self.declare_parameter('wheelbase', 0.20)
        self.declare_parameter('ref_offset', 0.10)  # marker midpoint ahead of the rear axle (m)
        self.declare_parameter('max_steer', 0.4084)  # centre angle at the driver's 28 deg inner-wheel limit (R 0.46 m)
        self.declare_parameter('horizon_steps', 10)
        self.declare_parameter('mpc_dt', 0.08)
        self.declare_parameter('q_cte', 40.0)
        self.declare_parameter('q_heading', 8.0)
        self.declare_parameter('r_steer', 0.8)  # reserved: not used in _cost_function (kept to match the sim tuning)
        self.declare_parameter('rd_steer', 2.5)
        self.declare_parameter('rezero_odom', False)  # False: track the path in the absolute odom / MoCap frame

        v_nom = float(self.get_parameter('v_nominal').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.N = int(self.get_parameter('horizon_steps').value)
        self.mpc_dt = float(self.get_parameter('mpc_dt').value)
        self.q_cte = float(self.get_parameter('q_cte').value)
        self.q_heading = float(self.get_parameter('q_heading').value)
        self.r_steer = float(self.get_parameter('r_steer').value)
        self.rd_steer = float(self.get_parameter('rd_steer').value)
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

        self.u_guess = np.zeros(self.N, dtype=np.float64)
        self.prev_steer = 0.0


        self.declare_parameter('obstacle_mode', 'cbf')  # cbf: filter after the MPC; constraint: inside it
        self.obstacle_mode = str(self.get_parameter('obstacle_mode').value)
        self.keep_out = float(self.get_parameter('robot_radius').value) + float(self.get_parameter('safety_margin').value)
        self.obstacle_mpc = NonlinearObstacleMpc(self.wheelbase, self.max_steer, self.traj.ref_offset, self.keep_out)
        self.run = RunManager(self, 'nmpc', self.traj, self.wheelbase, self.max_steer,
                              on_start=self._reset_memory)


        self.create_subscription(Odometry, '/hardware_odom', self.odom_cb, 20)
        
        # We run NMPC slightly slower (25Hz) because SLSQP is computationally heavier
        self.control_timer = self.create_timer(0.04, self.control_loop)

    # =========================================================================
    # [UTIL]   quat_to_yaw(q) -> yaw [rad]
    # -------------------------------------------------------------------------
    # Converts a geometry_msgs Quaternion (x, y, z, w) into a planar yaw (rad).
    # Standard ZYX formula: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2)).
    # Static: no node state is used. Result is in (-pi, pi].
    # =========================================================================
    @staticmethod
    def quat_to_yaw(q):
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
        rx, ry = msg.pose.pose.position.x, msg.pose.pose.position.y
        ryaw = self.quat_to_yaw(msg.pose.pose.orientation)
        if self.x0 is None:
            self.x0, self.y0, self.yaw0 = (rx, ry, ryaw) if self.rezero_odom else (0.0, 0.0, 0.0)
        dx, dy = rx - self.x0, ry - self.y0
        c0, s0 = math.cos(self.yaw0), math.sin(self.yaw0)
        # rotate the offset by -yaw0 (world frame -> controller frame)
        self.x = c0 * dx + s0 * dy
        self.y = -s0 * dx + c0 * dy
        # wrap the yaw difference to (-pi, pi]
        self.yaw = math.atan2(math.sin(ryaw - self.yaw0), math.cos(ryaw - self.yaw0))
        self.v_actual = msg.twist.twist.linear.x
        self.odom_received = True
        self.run.note_odom()

    # =========================================================================
    # [CORE]   _cost_function(u_steer, x0, y0, yaw0, ref_horizon) -> float
    #          (helper of _solve_nmpc; the objective SLSQP minimises)
    # -------------------------------------------------------------------------
    # Inputs: candidate steering sequence u_steer (N values, rad); current pose of the
    # tracked point x0, y0 (m), yaw0 (rad); ref_horizon (N x 5: x, y, yaw, v, kappa).
    # Output: scalar cost (no unit).
    # 1. Rear axle start = tracked point moved back by ref_offset along the heading.
    # 2. For k = 0..N-1 (Euler, dt = mpc_dt, speed v_k from the reference):
    #      x += v cos(yaw) dt;  y += v sin(yaw) dt;  yaw += (v/L) tan(u_k) dt
    # 3. Errors at the tracked point (ref_offset ahead of the rear axle) against ref k:
    #      cte = lateral offset in the path frame
    #      heading_err = wrap(path_yaw - atan(ref_offset*kappa) - yaw)
    # 4. cost += q_cte*cte^2 + q_heading*heading_err^2 + rd_steer*(u_k - u_{k-1})^2,
    #    with u_{-1} = prev_steer. There is no terminal cost; r_steer is not used.
    # =========================================================================
    def _cost_function(self, u_steer, x0, y0, yaw0, ref_horizon):
        cost = 0.0
        L = self.wheelbase
        dt = self.mpc_dt
        # The bicycle model holds for the rear axle; errors are taken at the tracked point ahead of it
        lo = self.traj.ref_offset
        yaw = yaw0
        # rear axle position
        x, y = x0 - lo * math.cos(yaw0), y0 - lo * math.sin(yaw0)
        
        for k in range(self.N):
            vk = ref_horizon[k, 3]
            dk = u_steer[k]
            
            x += vk * math.cos(yaw) * dt
            y += vk * math.sin(yaw) * dt
            yaw += (vk / L) * math.tan(dk) * dt
            
            xr, yr, th_r, vr, kr = ref_horizon[k]
            dx, dy = xr - (x + lo * math.cos(yaw)), yr - (y + lo * math.sin(yaw))
            # path-frame lateral offset, + = path on the left
            cte = -math.sin(th_r) * dx + math.cos(th_r) * dy
            # yaw the robot must have so the tracked point follows the path
            th_des = th_r - math.atan(lo * kr)
            herr = math.atan2(math.sin(th_des - yaw), math.cos(th_des - yaw))
            
            d_prev = self.prev_steer if k == 0 else u_steer[k-1]
            slew = dk - d_prev
            
            cost += self.q_cte * (cte**2) + self.q_heading * (herr**2) + self.rd_steer * (slew**2)
            
        return cost

    # =========================================================================
    # [CORE]   _solve_nmpc(nearest_idx) -> (v, steer)
    #          (helper of control_loop)
    # -------------------------------------------------------------------------
    # Output: speed v (m/s) and steering (rad) for this tick.
    # 1. Sample N reference points ahead (same scheme as the LTV-MPC, advancing
    #    v_k*dt of arc length per step). Row k is the point one step on, because cost
    #    term k scores the state AFTER step k.
    # 2. Warm start: shift the last solution left, append atan(L*kappa_N).
    # 3. scipy.optimize.minimize(_cost_function, SLSQP), bounds +-max_steer on every
    #    u_k, options maxiter 6, ftol 1e-3 (few iterations on purpose: real-time).
    # 4. Apply u[0]; speed = v_ref * max(0.4, 1 - 0.5*|u0|/max_steer), the shared
    #    heuristic profile. The full solution is kept as the next warm start.
    # =========================================================================
    def _solve_nmpc(self, nearest_idx):
        dt = self.mpc_dt
        
        ref_horizon = np.zeros((self.N, 5), dtype=np.float64)
        curr_idx = float(nearest_idx)
        for k in range(self.N):
            idx_int = self.traj.index(round(curr_idx))
            pt_k = self.traj.waypoints[idx_int]
            nxt_pt = self.traj.waypoints[self.traj.index(idx_int + 1)]
            ds = max(math.hypot(nxt_pt[0] - pt_k[0], nxt_pt[1] - pt_k[1]), 1e-3)
            curr_idx += (pt_k[3] * dt) / ds
            # Cost term k scores the state AFTER step k, so compare it with the point one step on
            ref_horizon[k] = self.traj.waypoints[self.traj.index(round(curr_idx))]
            
        bounds = [(-self.max_steer, self.max_steer) for _ in range(self.N)]
        
        # Warm start
        self.u_guess[:-1] = self.u_guess[1:]
        self.u_guess[-1] = math.atan(self.wheelbase * ref_horizon[-1, 4])
        
        res = minimize(
            self._cost_function,
            self.u_guess,
            args=(self.x, self.y, self.yaw, ref_horizon),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 6, 'ftol': 1e-3}
        )
        
        self.u_guess = res.x
        v_opt = ref_horizon[0, 3] * max(0.4, 1.0 - 0.5 * abs(res.x[0]) / self.max_steer)
        return float(v_opt), float(res.x[0])

    # =========================================================================
    # [CORE]   control_loop()
    # -------------------------------------------------------------------------
    # 25 Hz (0.04 s) timer callback. RunManager.tick() decides whether there is anything
    # to drive (it handles waiting, the approach, laps, the guard and the kill).
    # 1. RunManager.tick() -> nearest index (the cte / heading values are not used here;
    #    the cost function computes its own errors). None: nothing to do.
    # 2. _solve_nmpc() (timed in ms, logged as solve_ms) gives speed and steering
    #    directly. In obstacle_mode 'constraint' it calls _constraint_step() instead.
    # 3. RunManager.shape(): guard speed factor.
    # 4. ObstacleCbfLayer.apply(): closest safe (v, steer) on the known obstacles.
    # 5. RunManager.command(): publish /cmd_vel, lap statistics, CSV row, then
    #    prev_steer = the steering actually sent (after shape and CBF).
    # =========================================================================
    def control_loop(self):
        tick = self.run.tick(self.x, self.y, self.yaw, 5)
        if tick is None:
            return
        _cte, _heading_err, _ref_pt, nearest_idx, _dt = tick

        if self.obstacle_mode == 'constraint':
            self._constraint_step(nearest_idx)
            return

        t0 = time.perf_counter()
        v_nmpc, steer_nmpc = self._solve_nmpc(nearest_idx)
        solve_ms = (time.perf_counter() - t0) * 1000.0

        v_nmpc, steer_nmpc = self.run.shape(v_nmpc, steer_nmpc, nearest_idx)
        v_safe, steer_safe, clearance, status = self.safety.apply(
            v_nmpc, steer_nmpc, self.x, self.y, self.yaw, self.x0, self.y0, self.yaw0
        )
        self.run.command(v_safe, steer_safe, clearance, status, solve_ms)
        self.prev_steer = self.run.steer

    # =========================================================================
    # [CORE]   _constraint_step(nearest_idx)
    #          (helper of control_loop)
    # -------------------------------------------------------------------------
    # obstacle_mode:=constraint: obstacles as constraints in the solver
    # (mpc_obstacles.py). The CBF filter is not used. Steps:
    # 1. Obstacles into the controller frame (world_obstacles), 3 s reference (sample_reference).
    # 2. NonlinearObstacleMpc.solve() -> steering (rad) and shortfall (m; how far the
    #    plan's closest approach is under the keep-out distance).
    # 3. Speed from the reference with the PID speed law, then RunManager.shape().
    # 4. speed_for_plan(): slows or stops if the plan is not safe.
    # 5. RunManager.command() and prev_steer, as in control_loop.
    # =========================================================================
    def _constraint_step(self, nearest_idx):
        obstacles = world_obstacles(self.safety, self.x0, self.y0, self.yaw0)
        ref = sample_reference(self.traj, nearest_idx)
        t0 = time.perf_counter()
        steer, short = self.obstacle_mpc.solve(self.x, self.y, self.yaw, ref, self.prev_steer, obstacles)
        solve_ms = (time.perf_counter() - t0) * 1000.0
        v = float(self.traj.waypoints[nearest_idx, 3]) * max(0.4, 1.0 - 0.5 * abs(steer) / self.max_steer)
        v, steer = self.run.shape(v, steer, nearest_idx)
        clear = clearance(self.x, self.y, obstacles)
        v, status = speed_for_plan(v, short, clear, self.keep_out)
        self.run.command(v, steer, clear, status, solve_ms)
        self.prev_steer = self.run.steer

    # =========================================================================
    # [LIFE]   _reset_memory()  (helper: only used in this class, as RunManager on_start)
    # -------------------------------------------------------------------------
    # Start of a run: clear the warm start (u_guess), the previous steering,
    # and the obstacle MPC state (warm start, obstacles already seen).
    # =========================================================================
    def _reset_memory(self):
        self.u_guess = np.zeros(self.N, dtype=np.float64)
        self.prev_steer = 0.0
        self.obstacle_mpc.reset()


# =============================================================================
# [LIFE]   main(args=None)
# -----------------------------------------------------------------------------
# Console-script entry point (nmpc_tracker).
# init rclpy -> construct NmpcTrackerNode -> spin until Ctrl-C / SystemExit ->
# stop robot (node.run.shutdown), destroy node, shutdown rclpy.
# init_ros keeps the ROS context alive on Ctrl-C, so the stop command can still
# be published in the finally block.
# =============================================================================
def main(args=None):
    # Plain Python SIGINT handling: Ctrl-C raises KeyboardInterrupt but keeps the
    # ROS context alive, so the shutdown code below can still publish.
    init_ros(args)  # Foxy + Humble: context stays alive on Ctrl-C
    node = NmpcTrackerNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit): pass
    finally:
        node.run.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__': main()
