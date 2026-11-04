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
# #############################################################################

import csv
import math
import os
import time
import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu

from limo_mocap_suite.trajectory_subscriber import TrajectorySubscriber
from limo_mocap_suite.obstacle_cbf import ObstacleCbfLayer
from limo_mocap_suite.ros_compat import init_ros


# =============================================================================
# [CORE]   class NmpcTrackerNode(Node)
# -----------------------------------------------------------------------------
# ROS 2 node 'nmpc_tracker_node': path in, /cmd_vel out.
# =============================================================================
class NmpcTrackerNode(Node):
    """Hardware-transferable Full Nonlinear MPC (SLSQP)."""

    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Builds the NMPC node:
    # - declares/reads ROS parameters (profile, v_nominal, wheelbase,
    #   max_steer, horizon_steps, mpc_dt, q_cte, q_heading, r_steer, rd_steer,
    #   log_dir, run_duration, rezero_odom, ref_offset)
    # - creates TrajectorySubscriber (listens to /reference_path) and
    #   ObstacleCbfLayer (CBF)
    # - initialises pose and controller memory
    # - opens CSV telemetry file <log_dir>/nmpc_<profile>_telemetry.csv
    # - publishers : /cmd_vel (Twist), /reference_path_viz (Path, viz copy)
    # - subscribers: /hardware_odom (Odometry), /hardware_obstacles (latched)
    # - timers     : control_loop @ 25 Hz (0.04 s), print_status @ 0.5 Hz
    # =========================================================================
    def __init__(self):
        super().__init__('nmpc_tracker_node')

        self.declare_parameter('profile', 'figure8')
        self.declare_parameter('v_nominal', 0.40)
        self.declare_parameter('wheelbase', 0.20)
        self.declare_parameter('ref_offset', 0.10)  # marker midpoint ahead of the rear axle (m)
        self.declare_parameter('max_steer', 0.48869)
        self.declare_parameter('horizon_steps', 10)
        self.declare_parameter('mpc_dt', 0.08)
        self.declare_parameter('q_cte', 40.0)
        self.declare_parameter('q_heading', 8.0)
        self.declare_parameter('r_steer', 0.8)  # reserved: not used in _cost_function (kept to match the sim tuning)
        self.declare_parameter('rd_steer', 2.5)
        self.declare_parameter('log_dir', 'evaluation')
        self.declare_parameter('rezero_odom', True)  # False: track the path in the absolute odom / MoCap frame
        self.declare_parameter('run_duration', 25.0)

        profile = self.get_parameter('profile').value
        v_nom = float(self.get_parameter('v_nominal').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.N = int(self.get_parameter('horizon_steps').value)
        self.mpc_dt = float(self.get_parameter('mpc_dt').value)
        self.q_cte = float(self.get_parameter('q_cte').value)
        self.q_heading = float(self.get_parameter('q_heading').value)
        self.r_steer = float(self.get_parameter('r_steer').value)
        self.rd_steer = float(self.get_parameter('rd_steer').value)
        self.run_duration = float(self.get_parameter('run_duration').value)
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

        self.start_time = None
        self.prev_time = None
        self.cte_history = []
        self.heading_err_history = []
        self.solve_times_ms = []

        log_dir = self.get_parameter('log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, f'nmpc_{profile}_telemetry.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            't', 'x', 'y', 'yaw', 'v_actual', 'v_cmd', 'omega_cmd', 'steer_rad',
            'x_ref', 'y_ref', 'yaw_ref', 'cte', 'heading_err', 'min_clearance', 'safety_status', 'solve_ms'
        ])

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/reference_path_viz', 1)  # separate topic: avoids re-ingesting the decimated copy

        self.create_subscription(Odometry, '/hardware_odom', self.odom_cb, 20)
        
        # We run NMPC slightly slower (25Hz) because SLSQP is computationally heavier
        self.control_timer = self.create_timer(0.04, self.control_loop)
        self.log_timer = self.create_timer(2.0, self.print_status)
        self.get_logger().info(f'NMPC Tracker initialized | logging to {self.csv_path}')

    # =========================================================================
    # [UTIL]   quat_to_yaw(q) -> float
    # -------------------------------------------------------------------------
    # Converts a geometry_msgs Quaternion into a planar yaw angle (rad).
    # Uses the standard ZYX formula: atan2(2(wz+xy), 1-2(y^2+z^2)).
    # =========================================================================
    @staticmethod
    def quat_to_yaw(q):
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
        rx, ry = msg.pose.pose.position.x, msg.pose.pose.position.y
        ryaw = self.quat_to_yaw(msg.pose.pose.orientation)
        if self.x0 is None:
            self.x0, self.y0, self.yaw0 = (rx, ry, ryaw) if self.rezero_odom else (0.0, 0.0, 0.0)
        dx, dy = rx - self.x0, ry - self.y0
        c0, s0 = math.cos(self.yaw0), math.sin(self.yaw0)
        self.x = c0 * dx + s0 * dy
        self.y = -s0 * dx + c0 * dy
        self.yaw = math.atan2(math.sin(ryaw - self.yaw0), math.cos(ryaw - self.yaw0))
        self.v_actual = msg.twist.twist.linear.x
        self.odom_received = True

    # =========================================================================
    # [CORE]   _cost_function(u_steer, x0, y0, yaw0, ref_horizon) -> float
    # -------------------------------------------------------------------------
    # Objective evaluated by SLSQP. Forward-simulates the nonlinear bicycle
    # model of the rear axle (Euler, dt = mpc_dt), measures errors at the tracked
    # point ref_offset ahead of it, under the candidate steering sequence, and sums
    # q_cte*cte^2 + q_heading*heading_err^2 + rd_steer*(u_k - u_{k-1})^2.
    # =========================================================================
    def _cost_function(self, u_steer, x0, y0, yaw0, ref_horizon):
        cost = 0.0
        L = self.wheelbase
        dt = self.mpc_dt
        # The bicycle model holds for the rear axle; errors are taken at the tracked point ahead of it
        lo = self.traj.ref_offset
        yaw = yaw0
        x, y = x0 - lo * math.cos(yaw0), y0 - lo * math.sin(yaw0)
        
        for k in range(self.N):
            vk = ref_horizon[k, 3]
            dk = u_steer[k]
            
            x += vk * math.cos(yaw) * dt
            y += vk * math.sin(yaw) * dt
            yaw += (vk / L) * math.tan(dk) * dt
            
            xr, yr, th_r, vr, kr = ref_horizon[k]
            dx, dy = xr - (x + lo * math.cos(yaw)), yr - (y + lo * math.sin(yaw))
            cte = -math.sin(th_r) * dx + math.cos(th_r) * dy
            th_des = th_r - math.atan(lo * kr)
            herr = math.atan2(math.sin(th_des - yaw), math.cos(th_des - yaw))
            
            d_prev = self.prev_steer if k == 0 else u_steer[k-1]
            slew = dk - d_prev
            
            cost += self.q_cte * (cte**2) + self.q_heading * (herr**2) + self.rd_steer * (slew**2)
            
        return cost

    # =========================================================================
    # [CORE]   _solve_nmpc(nearest_idx) -> (v, steer)
    # -------------------------------------------------------------------------
    # 1. Sample N reference points ahead (same scheme as the LTV-MPC).
    # 2. Warm start: shift last solution left, append atan(L*kappa_N).
    # 3. scipy.optimize.minimize(SLSQP, bounds +/-max_steer, maxiter 6).
    # 4. Apply u[0]; speed from the shared heuristic profile.
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
    # 25 Hz (0.04 s) timer callback. One full control step:
    # 1. Gate: return until odometry AND a reference path have arrived.
    # 2. First tick: start the clock, publish_ref_path(), return.
    # 3. If run_duration elapsed or an open path's end is reached:
    #    stop_robot(), log final metrics, raise SystemExit.
    # 4. TrajectorySubscriber.compute_tracking_errors() -> cte, heading_err,
    #   ref point.
    # 5. _solve_nmpc() (timed in ms) gives speed and steering directly.
    # 6. Speed: v_cmd = v_ref * max(0.4, 1 - 0.5*|steer|/max_steer).
    # 7. ObstacleCbfLayer.apply(): closest safe (v, steer) to the command (CBF on
    #    the known obstacles); re-clip.
    # 8. Ackermann conversion: omega = v * tan(steer) / L; publish Twist on
    #   /cmd_vel.
    # 9. Append errors to history and write one CSV telemetry row.
    # =========================================================================
    def control_loop(self):
        if not self.odom_received:
            return

        if not getattr(self.traj, 'path_ready', False):
            return

        now = time.monotonic()
        if self.start_time is None:
            self.start_time = now
            self.publish_ref_path()
            return

        elapsed = now - self.start_time
        if self.traj.goal_reached or (self.run_duration > 0.0 and elapsed >= self.run_duration):
            self.stop_robot()
            rms_cte = math.sqrt(np.mean(np.square(self.cte_history))) if self.cte_history else 0.0
            max_cte = float(np.max(np.abs(self.cte_history))) if self.cte_history else 0.0
            rms_head = math.sqrt(np.mean(np.square(self.heading_err_history))) if self.heading_err_history else 0.0
            mean_solve = float(np.mean(self.solve_times_ms)) if self.solve_times_ms else 0.0
            self.get_logger().info(
                f'=== NMPC RUN COMPLETE ({elapsed:.1f}s) | RMS CTE: {rms_cte:.4f}m | Max CTE: {max_cte:.4f}m | RMS Heading Err: {rms_head:.4f}rad | Avg Solve: {mean_solve:.2f}ms ==='
            )
            raise SystemExit

        cte, heading_err, ref_pt, nearest_idx = self.traj.compute_tracking_errors(self.x, self.y, self.yaw, lookahead_pts=5)
        
        t0 = time.perf_counter()
        v_nmpc, steer_nmpc = self._solve_nmpc(nearest_idx)
        solve_ms = (time.perf_counter() - t0) * 1000.0

        v_safe, steer_safe, min_dist, status = self.safety.apply(
            v_nmpc, steer_nmpc, self.x, self.y, self.yaw, self.x0, self.y0, self.yaw0
        )
        steer_safe = float(np.clip(steer_safe, -self.max_steer, self.max_steer))
        self.prev_steer = steer_safe
        omega_cmd = (v_safe / self.wheelbase) * math.tan(steer_safe) if abs(v_safe) > 1e-4 else 0.0

        twist = Twist()
        twist.linear.x, twist.angular.z = float(v_safe), float(omega_cmd)
        self.cmd_pub.publish(twist)

        self.cte_history.append(cte)
        self.heading_err_history.append(heading_err)
        self.solve_times_ms.append(solve_ms)
        self.csv_writer.writerow([
            f'{elapsed:.3f}', f'{self.x:.4f}', f'{self.y:.4f}', f'{self.yaw:.4f}',
            f'{self.v_actual:.4f}', f'{v_safe:.4f}', f'{omega_cmd:.4f}', f'{steer_safe:.4f}',
            f'{ref_pt[0]:.4f}', f'{ref_pt[1]:.4f}', f'{ref_pt[2]:.4f}',
            f'{cte:.4f}', f'{heading_err:.4f}', f'{min_dist:.3f}', status, f'{solve_ms:.3f}'
        ])

    # =========================================================================
    # [LOG]    print_status()
    # -------------------------------------------------------------------------
    # 2 s timer. Prints elapsed time, pose, latest CTE, running RMS CTE and
    # heading error plus last solve time to the ROS logger.
    # Does nothing until odometry and at least one control step exist.
    # =========================================================================
    def print_status(self):
        if not self.odom_received or not self.cte_history: return
        rms_cte = math.sqrt(np.mean(np.square(self.cte_history)))
        self.get_logger().info(f'NMPC | CTE={self.cte_history[-1]:+6.3f}m (RMS={rms_cte:.3f}m) | solve={self.solve_times_ms[-1]:.1f}ms')

    # =========================================================================
    # [ROS-IO] publish_ref_path()
    # -------------------------------------------------------------------------
    # Publishes every 5th stored waypoint as a nav_msgs/Path on
    # /reference_path_viz (frame 'odom') once, at the first control tick, for
    # visualisation (RViz). Kept off /reference_path so no controller
    # re-ingests the decimated copy.
    # =========================================================================
    def publish_ref_path(self):
        path_msg = Path()
        path_msg.header.frame_id, path_msg.header.stamp = 'odom', self.get_clock().now().to_msg()
        for pt in self.traj.waypoints[::5]:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x, ps.pose.position.y = float(pt[0]), float(pt[1])
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

    # =========================================================================
    # [LIFE]   stop_robot()
    # -------------------------------------------------------------------------
    # Publishes a zero Twist on /cmd_vel (robot halts; skipped if the ROS
    # context is already shut down) and closes the CSV file.
    # =========================================================================
    def stop_robot(self):
        if rclpy.ok():
            self.cmd_pub.publish(Twist())
        if not self.csv_file.closed:
            self.csv_file.close()

# =============================================================================
# [LIFE]   main(args=None)
# -----------------------------------------------------------------------------
# Console-script entry point (nmpc_tracker).
# init rclpy -> construct node -> spin until Ctrl-C / SystemExit ->
# stop robot, destroy node, shutdown rclpy.
# =============================================================================
def main(args=None):
    # Plain Python SIGINT handling: Ctrl-C raises KeyboardInterrupt but keeps the
    # ROS context alive, so the shutdown code below can still publish.
    init_ros(args)  # Foxy + Humble: context stays alive on Ctrl-C
    node = NmpcTrackerNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit): pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__': main()
