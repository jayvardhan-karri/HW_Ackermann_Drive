# #############################################################################
# [FILE]   sim_limo.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/sim_limo.py
# ROLE  : A stand-in robot for rehearsing without the lab or Gazebo: bicycle model
#         that takes /cmd_vel and publishes /hardware_odom in real time, as the
#         bridge would. Its steering is deliberately imperfect (gain, trim, lag,
#         rate limit) because a perfect robot leaves nothing to adapt to.
#
# Console script : ros2 run limo_mocap_suite sim_limo
# Also           : run_laps.launch.py sim:=true
# Model          : bicycle model, rear axle position (x, y), yaw; the odometry is
#                  published for the tracked point, ref_offset ahead of the axle
# obstacles  : flat [x, y, r, ...], published latched on /hardware_obstacles as the
#              bridge would, so the obstacle add-on can be rehearsed (none)
# Not used on the real robot.
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
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32MultiArray

from limo_mocap_suite.obstacle_cbf import declare_flexible_parameter, parse_obstacles
from limo_mocap_suite.ros_compat import init_ros


# =============================================================================
# [LIFE]   class SimLimo(Node)
# -----------------------------------------------------------------------------
# The stand-in robot: reads /cmd_vel, moves a bicycle model every 10 ms,
# publishes /hardware_odom (and /hardware_obstacles once, if obstacles are set).
# Imperfections, so the controllers have something to correct: steering gain
# curv_gain, offset bias (rad), first-order lag tau (s), rate limit rate (rad/s),
# transport delay delay (s). Units: m, rad, s.
# =============================================================================
class SimLimo(Node):
    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Declares the parameters, puts the robot at (x, y, yaw), creates the
    # topics and the 100 Hz timer that calls step().
    # =========================================================================
    def __init__(self):
        super().__init__('sim_limo')
        for name, value in (('x', 1.0), ('y', 0.5), ('yaw', 0.0), ('wheelbase', 0.20), ('ref_offset', 0.10),
                            ('max_steer', 0.4084), ('curv_gain', 0.89), ('bias', 0.02), ('tau', 0.12),
                            ('rate', 1.5), ('delay', 0.06), ('drift_at', 0.0), ('drift_steer', 0.0),
                            ('drift_for', 0.0), ('freeze_at', 0.0)):
            self.declare_parameter(name, value)
        g = lambda n: float(self.get_parameter(n).value)  # noqa: E731   short parameter reader
        self.L, self.a, self.limit = g('wheelbase'), g('ref_offset'), g('max_steer')
        self.gain, self.bias, self.tau, self.rate, self.dt = g('curv_gain'), g('bias'), g('tau'), g('rate'), 0.01
        self.drift = (g('drift_at'), g('drift_steer'), g('drift_for'))
        self.freeze_at = g('freeze_at')
        self.t = 0.0
        self.moved = 0.0
        self.yaw = g('yaw')
        self.x = g('x') - self.a * math.cos(self.yaw)   # rear axle; odometry reports the tracked point
        self.y = g('y') - self.a * math.sin(self.yaw)
        self.v = 0.0
        self.steer_cmd = 0.0
        self.steer = 0.0
        self.delay = deque([0.0] * max(1, int(g('delay') / self.dt)))   # one slot per tick of delay
        self.pub = self.create_publisher(Odometry, '/hardware_odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(self.dt, self.step)
        obstacles = parse_obstacles(declare_flexible_parameter(self, 'obstacles'))
        if obstacles:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.obstacle_pub = self.create_publisher(Float32MultiArray, '/hardware_obstacles', latched)
            self.obstacle_pub.publish(Float32MultiArray(data=[float(v) for v in obstacles]))

    # =========================================================================
    # [ROS-IO] cmd_cb(msg: Twist)
    # -------------------------------------------------------------------------
    # Keeps the commanded speed (no reverse) and steering for step().
    # HOW: steering from the bicycle model, steer = atan(omega * wheelbase / v).
    # Below 1 mm/s it is undefined, so the last steering is kept.
    # =========================================================================
    def cmd_cb(self, msg):
        self.v = max(0.0, msg.linear.x)
        self.steer_cmd = math.atan(msg.angular.z * self.L / self.v) if self.v > 1e-3 else self.steer_cmd

    # =========================================================================
    # [CORE]   step()  <- 100 Hz
    # -------------------------------------------------------------------------
    # Steering delay, lag, rate limit, gain and trim, then one bicycle step.
    # HOW: take the commanded steering from the delay queue; move the actual
    # steering toward it with time constant tau, capped at rate (rad/s) and at the
    # limit; the wheel angle really turned is atan(gain * tan(steer + bias));
    # integrate x, y, yaw with the midpoint heading (rear-axle bicycle model);
    # publish the pose of the tracked point, ref_offset ahead of the axle.
    # drift_*: test hook, forces the steering for a while after driving drift_at
    # metres, as if a wheel slipped. freeze_at: stops odometry, as if MoCap lost it.
    # =========================================================================
    def step(self):
        self.t += self.dt
        self.delay.append(self.steer_cmd)
        want = self.delay.popleft()
        at, steer, span = self.drift
        if at > 0.0 and at <= self.moved < at + span:     # drift test hook active
            want = steer
        rate = (want - self.steer) / max(self.tau, self.dt)
        if self.rate > 0.0:
            rate = max(-self.rate, min(self.rate, rate))
        self.steer = max(-self.limit, min(self.limit, self.steer + rate * self.dt))
        eff = math.atan(self.gain * math.tan(self.steer + self.bias))
        w = self.v / self.L * math.tan(eff)
        ym = self.yaw + 0.5 * w * self.dt
        self.x += self.v * math.cos(ym) * self.dt
        self.y += self.v * math.sin(ym) * self.dt
        self.yaw += w * self.dt
        self.moved += self.v * self.dt
        if 0.0 < self.freeze_at <= self.moved:     # odometry lost test hook
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self.x + self.a * math.cos(self.yaw)
        msg.pose.pose.position.y = self.y + self.a * math.sin(self.yaw)
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = self.v
        self.pub.publish(msg)


# =============================================================================
# [LIFE]   main(args=None)  -> console script sim_limo
# -----------------------------------------------------------------------------
# Creates the node and spins it until Ctrl-C, then shuts down cleanly.
# =============================================================================
def main(args=None):
    init_ros(args)
    node = SimLimo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
