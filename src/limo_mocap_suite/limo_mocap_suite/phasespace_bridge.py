# #############################################################################
# [FILE]   phasespace_bridge.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/phasespace_bridge.py
# ROLE  : Hardware abstraction: PhaseSpace markers -> Odometry + obstacle map.
#
# Console script : ros2 run limo_mocap_suite phasespace_bridge
# Subscribes     : /phasespace/markers  (phasespace_msgs/Markers, depth 1)
# Publishes      : /hardware_odom (nav_msgs/Odometry)
#                  /hardware_obstacles (std_msgs/Float32MultiArray, latched:
#                  flat [x, y, r, ...]) for the controllers' CBF and the GUIs
# Heading        : vector from marker rear_id (1) to marker front_id (0)
# Parameters     : front_id (0), rear_id (1): marker ids (the Marker.id field)
#                  obstacles: flat [x, y, r, ...] in the PhaseSpace frame (none)
#                  obstacle_markers: [(0,1,2,3),(4,5,6,7)]: 4 markers on the rim
#                  of each cylinder; centre and radius come live from them (none)
# phasespace_msgs comes from the lab's phasespace_bringup workspace, which must
# be built and sourced on the machine running this node.
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
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32MultiArray
from phasespace_msgs.msg import Markers

from limo_mocap_suite.obstacle_cbf import (
    circle_from_markers, declare_flexible_parameter, parse_marker_groups, parse_obstacles)

# =============================================================================
# [ROS-IO] class PhaseSpaceBridgeNode(Node)
# -----------------------------------------------------------------------------
# Turns two MoCap markers (front + rear of the LIMO) into a 2-D pose and
# velocity, and publishes the obstacle map.
# =============================================================================
class PhaseSpaceBridgeNode(Node):
    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Reads parameters (no obstacles by default), creates publishers / subscriber,
    # publishes the fixed obstacles on the latched /hardware_obstacles topic (late
    # subscribers still receive it) and clears the pose state (pose_valid = False
    # until the first fully tracked marker frame).
    # =========================================================================
    def __init__(self):
        super().__init__('phasespace_bridge_node')
        
        self.declare_parameter('front_id', 0)
        self.declare_parameter('rear_id', 1)

        self.front_id = self.get_parameter('front_id').value
        self.rear_id = self.get_parameter('rear_id').value

        self.fixed_obstacles = parse_obstacles(declare_flexible_parameter(self, 'obstacles'))
        self.marker_groups = parse_marker_groups(declare_flexible_parameter(self, 'obstacle_markers'))
        self.marker_circles = {}    # group index -> last known (x, y, r); kept if occluded
        self.published_obstacles = None

        self.odom_pub = self.create_publisher(Odometry, '/hardware_odom', 10)

        # Latched (transient local) so GUIs started later still get the map
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.obstacle_pub = self.create_publisher(Float32MultiArray, '/hardware_obstacles', latched)
        self.publish_obstacles()
        if self.marker_groups:
            self.create_timer(5.0, self.warn_missing_obstacles)

        # Depth 1: only the newest MoCap frame matters
        self.create_subscription(Markers, '/phasespace/markers', self.marker_cb, 1)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_valid = False
        self.last_time = self.get_clock().now()

        self.get_logger().info(
            f'PhaseSpace Bridge ready | front_id={self.front_id} rear_id={self.rear_id} '
            f'| {len(self.fixed_obstacles) // 3} fixed obstacle(s), '
            f'{len(self.marker_groups)} from markers {self.marker_groups}')

    # =========================================================================
    # [UTIL]   euler_to_quat(yaw) -> [x, y, z, w]
    # -------------------------------------------------------------------------
    # Yaw-only rotation to quaternion: [0, 0, sin(yaw/2), cos(yaw/2)].
    # =========================================================================
    def euler_to_quat(self, yaw):
        return [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]

    # =========================================================================
    # [CORE]   marker_cb(msg)
    # -------------------------------------------------------------------------
    # Callback for /phasespace/markers. Pose estimation from two markers:
    # 0. Update the obstacles that come from markers (even if the robot is lost).
    # 1. Find the front / rear markers by their id field. Skip the frame if either
    #    is missing, has cond <= 0 (not tracked) or sits exactly at (0, 0, 0).
    # 2. position = midpoint of the markers; yaw = atan2(fy-ry, fx-rx).
    # 3. Finite-difference world velocity, projected on the previous heading
    #    -> longitudinal speed v_long (0 on the first valid frame).
    # 4. Publish Odometry (frame 'odom', child 'base_footprint').
    # =========================================================================
    def marker_cb(self, msg: Markers):
        tracked = {m.id: m for m in msg.markers
                   if m.cond > 0.0 and not (m.x == 0.0 and m.y == 0.0 and m.z == 0.0)}
        self.update_marker_obstacles(tracked)
        if self.front_id not in tracked or self.rear_id not in tracked:
            return  # Robot markers not tracked in this frame
        f, r = tracked[self.front_id], tracked[self.rear_id]
        fx, fy = f.x, f.y
        rx, ry = r.x, r.y

        new_x = (fx + rx) / 2.0
        new_y = (fy + ry) / 2.0
        new_yaw = math.atan2(fy - ry, fx - rx)

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if not self.pose_valid:
            # First valid frame: no previous pose to difference against
            self.robot_x, self.robot_y, self.robot_yaw = new_x, new_y, new_yaw
            self.pose_valid = True
            dt = 0.0

        v_x = (new_x - self.robot_x) / dt if dt > 0 else 0.0
        v_y = (new_y - self.robot_y) / dt if dt > 0 else 0.0
        v_long = v_x * math.cos(self.robot_yaw) + v_y * math.sin(self.robot_yaw)

        self.robot_x, self.robot_y, self.robot_yaw = new_x, new_y, new_yaw

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        
        q = self.euler_to_quat(self.robot_yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v_long
        
        self.odom_pub.publish(odom)

    # =========================================================================
    # [CORE]   update_marker_obstacles(tracked)
    # -------------------------------------------------------------------------
    # For each 4-marker group fully tracked in this frame: centre = mean, radius =
    # mean distance from the centre. A group with an occluded marker keeps its
    # last known circle. Republishes when a circle first appears or moves > 2 cm.
    # =========================================================================
    def update_marker_obstacles(self, tracked):
        changed = False
        for i, group in enumerate(self.marker_groups):
            if all(g in tracked for g in group):
                c = circle_from_markers([(tracked[g].x, tracked[g].y) for g in group])
                old = self.marker_circles.get(i)
                if old is None or max(abs(c[k] - old[k]) for k in range(3)) > 0.02:
                    self.marker_circles[i] = c
                    changed = True
        if changed:
            self.publish_obstacles()

    # =========================================================================
    # [ROS-IO] publish_obstacles()
    # -------------------------------------------------------------------------
    # Fixed obstacles + marker circles as a flat list on /hardware_obstacles.
    # =========================================================================
    def publish_obstacles(self):
        flat = list(self.fixed_obstacles)
        for i in sorted(self.marker_circles):
            flat.extend(self.marker_circles[i])
        self.obstacle_pub.publish(Float32MultiArray(data=[float(v) for v in flat]))
        self.published_obstacles = flat

    # =========================================================================
    # [LOG]    warn_missing_obstacles()
    # -------------------------------------------------------------------------
    # 5 s timer: warns about groups never seen fully tracked. The CBF cannot
    # avoid an obstacle it has never been given.
    # =========================================================================
    def warn_missing_obstacles(self):
        missing = [g for i, g in enumerate(self.marker_groups) if i not in self.marker_circles]
        if missing:
            self.get_logger().warn(f'Obstacle markers never fully tracked yet: {missing}')

# =============================================================================
# [LIFE]   main(args=None)
# -----------------------------------------------------------------------------
# Entry point: init rclpy, spin the bridge until Ctrl-C, then clean up.
# A second SIGINT (ros2 launch can deliver one during cleanup) is swallowed so
# shutdown stays quiet.
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PhaseSpaceBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass

if __name__ == '__main__': main()
