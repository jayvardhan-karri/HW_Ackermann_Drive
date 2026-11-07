# #############################################################################
# [FILE]   pid_tracker_node.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/pid_tracker_node.py
# ROLE  : Kinematic Ackermann PID tracker with curvature feed-forward.
#
# Console script : ros2 run limo_mocap_suite pid_tracker
# Law            : steer = atan(L*kappa) + Kp*cte + Ki*int(cte) + Kd*d(cte)/dt
#   + Kh*heading_err
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
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from limo_mocap_suite.trajectory_subscriber import TrajectorySubscriber
from limo_mocap_suite.obstacle_cbf import ObstacleCbfLayer
from limo_mocap_suite.run_manager import RunManager
from limo_mocap_suite.ros_compat import init_ros


# =============================================================================
# [CORE]   class PidTrackerNode(Node)
# -----------------------------------------------------------------------------
# ROS 2 node 'pid_tracker_node': path in (/reference_path), /cmd_vel out.
# Kinematic Ackermann PID with curvature feed-forward. Chain every tick:
#   RunManager.tick -> PID law -> RunManager.shape -> ObstacleCbfLayer.apply
#   -> RunManager.command.
# Memory: integral_cte (m*s, clamped +-0.5) and prev_cte (m, for the derivative).
# Sign convention: cte > 0 = path on the robot's left; steer > 0 = turn left.
# =============================================================================
class PidTrackerNode(Node):
    """Hardware-transferable Kinematic Ackermann PID Trajectory Tracker."""

    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Builds the PID node:
    # - declares/reads ROS parameters, defaults: v_nominal 0.40 m/s, kp_cte 1.8,
    #   ki_cte 0.05, kd_cte 0.45, kp_heading 1.15, wheelbase 0.20 m, ref_offset 0.10 m,
    #   max_steer 0.4084 rad, lookahead_pts 10, rezero_odom False
    # - creates TrajectorySubscriber (listens to /reference_path) and
    #   ObstacleCbfLayer (CBF)
    # - initialises pose and controller memory (integral_cte, prev_cte)
    # - creates RunManager (approach, laps, guard, kill, logging, /cmd_vel);
    #   its on_start callback is _reset_memory
    # - subscribers: /hardware_odom (Odometry), /hardware_obstacles (latched, made by
    #   ObstacleCbfLayer), /imu (Imu)
    # - timers     : control_loop @ 50 Hz (0.02 s)
    # =========================================================================
    def __init__(self):
        super().__init__('pid_tracker_node')

        # Declare configurable parameters (identical for sim and physical LIMO)
        self.declare_parameter('v_nominal', 0.40)
        self.declare_parameter('kp_cte', 1.8)
        self.declare_parameter('ki_cte', 0.05)
        self.declare_parameter('kd_cte', 0.45)
        self.declare_parameter('kp_heading', 1.15)
        self.declare_parameter('wheelbase', 0.20)
        self.declare_parameter('ref_offset', 0.10)  # marker midpoint ahead of the rear axle (m)
        self.declare_parameter('max_steer', 0.4084)  # centre angle at the driver's 28 deg inner-wheel limit (R 0.46 m)
        self.declare_parameter('lookahead_pts', 10)
        self.declare_parameter('rezero_odom', False)  # False: track the path in the absolute odom / MoCap frame

        v_nom = float(self.get_parameter('v_nominal').value)
        self.kp_cte = float(self.get_parameter('kp_cte').value)
        self.ki_cte = float(self.get_parameter('ki_cte').value)
        self.kd_cte = float(self.get_parameter('kd_cte').value)
        self.kp_heading = float(self.get_parameter('kp_heading').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.lookahead_pts = int(self.get_parameter('lookahead_pts').value)
        self.rezero_odom = bool(self.get_parameter('rezero_odom').value)

        self.traj = TrajectorySubscriber(self, v_nominal=v_nom,
                                         ref_offset=float(self.get_parameter('ref_offset').value))
        self.safety = ObstacleCbfLayer(self, self.wheelbase, self.max_steer, self.traj.ref_offset)

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.x0 = None
        self.y0 = None
        self.yaw0 = None
        self.v_actual = 0.0
        self.odom_received = False

        # PID memory
        self.integral_cte = 0.0
        self.prev_cte = 0.0

        self.run = RunManager(self, 'pid', self.traj, self.wheelbase, self.max_steer,
                              on_start=self._reset_memory)

        # Publishers & Subscribers (matched to limo_base hardware topics)

        self.create_subscription(Odometry, '/hardware_odom', self.odom_cb, 20)
        self.create_subscription(Imu, '/imu', self.imu_cb, 20)

        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz (matches limo_base)


    # =========================================================================
    # [UTIL]   quat_to_yaw(q) -> yaw [rad]
    # -------------------------------------------------------------------------
    # Converts a geometry_msgs Quaternion (x, y, z, w) into a planar yaw (rad).
    # Standard ZYX formula: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2)).
    # Static: no node state is used. Result is in (-pi, pi].
    # =========================================================================
    @staticmethod
    def quat_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

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
        pass  # Available for dynamic yaw-rate damping if needed

    # =========================================================================
    # [CORE]   control_loop()
    # -------------------------------------------------------------------------
    # 50 Hz (0.02 s) timer callback. RunManager.tick() decides whether there is anything
    # to drive (it handles waiting, the approach, laps, the guard and the kill).
    # 1. RunManager.tick() -> cte (m), heading_err (rad), lookahead ref point, nearest
    #    index, dt (s). None means nothing to do this tick.
    # 2. PID with curvature feed-forward (kappa and v_ref come from the lookahead point):
    #      steer = atan(L*kappa) + kp_cte*cte + ki_cte*int(cte) + kd_cte*d(cte)/dt
    #              + kp_heading*heading_err,  clipped to +-max_steer (rad)
    #    int(cte) is clamped to +-0.5 (anti-windup); d(cte)/dt is a backward difference.
    #    Speed: v = v_ref * max(0.4, 1 - 0.5*|steer|/max_steer), i.e. full speed on
    #    straights, half speed at full lock (the 0.4 floor is never reached).
    # 3. RunManager.shape(): guard speed factor.
    # 4. ObstacleCbfLayer.apply(): closest safe (v, steer) on the known obstacles.
    # 5. RunManager.command(): publish /cmd_vel, lap statistics, CSV row.
    # =========================================================================
    def control_loop(self):
        tick = self.run.tick(self.x, self.y, self.yaw, self.lookahead_pts)
        if tick is None:
            return
        cte, heading_err, ref_pt, _idx, dt = tick
        _x_ref, _y_ref, _yaw_ref, v_ref, kappa_ref = ref_pt

        # integral of cte (m*s), clamped so it cannot wind up
        self.integral_cte = np.clip(self.integral_cte + cte * dt, -0.5, 0.5)
        d_cte = (cte - self.prev_cte) / dt
        self.prev_cte = cte

        # bicycle model: steering angle that follows curvature kappa
        ff_steer = math.atan(self.wheelbase * kappa_ref)
        pid_steer = (
            ff_steer
            + self.kp_cte * cte
            + self.ki_cte * self.integral_cte
            + self.kd_cte * d_cte
            + self.kp_heading * heading_err
        )
        steer_cmd = float(np.clip(pid_steer, -self.max_steer, self.max_steer))
        v_cmd = v_ref * max(0.4, 1.0 - 0.5 * abs(steer_cmd) / self.max_steer)

        v_cmd, steer_cmd = self.run.shape(v_cmd, steer_cmd, _idx)
        v_safe, steer_safe, clearance, status = self.safety.apply(
            v_cmd, steer_cmd, self.x, self.y, self.yaw, self.x0, self.y0, self.yaw0
        )
        self.run.command(v_safe, steer_safe, clearance, status)

    # =========================================================================
    # [LIFE]   _reset_memory()  (helper: only used in this class, as RunManager on_start)
    # -------------------------------------------------------------------------
    # Called at the start of each run: forgets the previous run's PID state
    # (integral_cte and prev_cte set to 0), so an old integral cannot kick the new run.
    # =========================================================================
    def _reset_memory(self):
        self.integral_cte = 0.0
        self.prev_cte = 0.0

# =============================================================================
# [LIFE]   main(args=None)
# -----------------------------------------------------------------------------
# Console-script entry point (pid_tracker).
# init rclpy -> construct PidTrackerNode -> spin until Ctrl-C / SystemExit ->
# stop robot (node.run.shutdown), destroy node, shutdown rclpy.
# init_ros keeps the ROS context alive on Ctrl-C, so the stop command can still
# be published in the finally block.
# =============================================================================
def main(args=None):
    # Plain Python SIGINT handling: Ctrl-C raises KeyboardInterrupt but keeps the
    # ROS context alive, so the shutdown code below can still publish.
    init_ros(args)  # Foxy + Humble: context stays alive on Ctrl-C
    node = PidTrackerNode()
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
