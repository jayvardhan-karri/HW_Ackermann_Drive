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
# Also           : lap_test.launch.py sim:=true
# Not used on the real robot.
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
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from limo_mocap_suite.ros_compat import init_ros


class SimLimo(Node):
    def __init__(self):
        super().__init__('sim_limo')
        for name, value in (('x', 1.0), ('y', 0.5), ('yaw', 0.0), ('wheelbase', 0.20), ('ref_offset', 0.10),
                            ('max_steer', 0.4084), ('curv_gain', 0.89), ('bias', 0.02), ('tau', 0.12),
                            ('rate', 1.5), ('delay', 0.06), ('drift_at', 0.0), ('drift_steer', 0.0),
                            ('drift_for', 0.0), ('freeze_at', 0.0)):
            self.declare_parameter(name, value)
        g = lambda n: float(self.get_parameter(n).value)  # noqa: E731
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
        self.delay = deque([0.0] * max(1, int(g('delay') / self.dt)))
        self.pub = self.create_publisher(Odometry, '/hardware_odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(self.dt, self.step)

    def cmd_cb(self, msg):
        self.v = max(0.0, msg.linear.x)
        self.steer_cmd = math.atan(msg.angular.z * self.L / self.v) if self.v > 1e-3 else self.steer_cmd

    # =========================================================================
    # [CORE]   step()  <- 100 Hz
    # -------------------------------------------------------------------------
    # Steering delay, lag, rate limit, gain and trim, then one bicycle step.
    # drift_*: test hook, forces the steering for a while after driving drift_at
    # metres, as if a wheel slipped. freeze_at: stops odometry, as if MoCap lost it.
    # =========================================================================
    def step(self):
        self.t += self.dt
        self.delay.append(self.steer_cmd)
        want = self.delay.popleft()
        at, steer, span = self.drift
        if at > 0.0 and at <= self.moved < at + span:
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
        if 0.0 < self.freeze_at <= self.moved:
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
