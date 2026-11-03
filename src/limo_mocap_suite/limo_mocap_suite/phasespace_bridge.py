# #############################################################################
# [FILE]   phasespace_bridge.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/phasespace_bridge.py
# ROLE  : Hardware abstraction: PhaseSpace markers -> Odometry (+ fake LiDAR).
#
# Console script : ros2 run limo_mocap_suite phasespace_bridge
# Subscribes     : /phasespace/markers  (phasespace_msgs/Markers, depth 1)
# Publishes      : /hardware_odom (nav_msgs/Odometry)
#                  /hardware_scan (sensor_msgs/LaserScan) if use_synthetic_scan
#                  /hardware_obstacles (std_msgs/Float32MultiArray, latched:
#                  flat [x, y, r, ...]) so the GUIs draw the same map
# Heading        : vector from marker rear_id (1) to marker front_id (0)
# Parameters     : front_id (0), rear_id (1), use_synthetic_scan (True),
#                  obstacles (flat [x, y, r, x, y, r, ...] in the PhaseSpace frame)
# Marker access, validity test, IDs and heading convention (rear -> front) are
# taken from the lab's working PhaseSpace script (safe_controller_hw.py).
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
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32MultiArray
from phasespace_msgs.msg import Markers

# Default obstacle map [x, y, radius] in metres (PhaseSpace frame): the four
# simulation pillars plus four huge circles approximating the +/-5 m walls.
DEFAULT_OBSTACLES = [
    2.5, 1.5, 0.15,   2.5, -1.5, 0.15,
    -2.5, 1.5, 0.15,  -2.5, -1.5, 0.15,
    0.0, 5005.0, 5000.0,  0.0, -5005.0, 5000.0,  # Walls
    5005.0, 0.0, 5000.0,  -5005.0, 0.0, 5000.0,
]

# =============================================================================
# [ROS-IO] class PhaseSpaceBridgeNode(Node)
# -----------------------------------------------------------------------------
# Turns two MoCap markers (front + rear of the LIMO) into a 2-D pose,
# velocity and an optional ray-cast LaserScan of known obstacles.
# =============================================================================
class PhaseSpaceBridgeNode(Node):
    # =========================================================================
    # [LIFE]   __init__()
    # -------------------------------------------------------------------------
    # Reads parameters, unpacks the 'obstacles' parameter into [x, y, radius]
    # triples (default: 4 pillars of r=0.15 m + 4 huge circles approximating
    # +/-5 m walls), creates publishers / subscriber, publishes the obstacle
    # map once on the latched /hardware_obstacles topic (late subscribers still
    # receive it) and clears the pose state (pose_valid = False until the first
    # fully tracked marker frame).
    # =========================================================================
    def __init__(self):
        super().__init__('phasespace_bridge_node')
        
        self.declare_parameter('front_id', 0)
        self.declare_parameter('rear_id', 1)
        self.declare_parameter('use_synthetic_scan', True)
        self.declare_parameter('obstacles', DEFAULT_OBSTACLES)

        self.front_id = self.get_parameter('front_id').value
        self.rear_id = self.get_parameter('rear_id').value
        self.use_synthetic_scan = self.get_parameter('use_synthetic_scan').value

        flat = [float(v) for v in self.get_parameter('obstacles').value]
        if len(flat) % 3 != 0:
            raise ValueError("'obstacles' must be a flat list of x, y, radius triples")
        self.obstacles = [flat[i:i + 3] for i in range(0, len(flat), 3)]

        self.odom_pub = self.create_publisher(Odometry, '/hardware_odom', 10)
        if self.use_synthetic_scan:
            self.scan_pub = self.create_publisher(LaserScan, '/hardware_scan', 10)

        # Latched (transient local) so GUIs started later still get the map
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.obstacle_pub = self.create_publisher(Float32MultiArray, '/hardware_obstacles', latched)
        self.obstacle_pub.publish(Float32MultiArray(data=flat))

        # Depth 1: only the newest MoCap frame matters
        self.create_subscription(Markers, '/phasespace/markers', self.marker_cb, 1)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_valid = False
        self.last_time = self.get_clock().now()

        self.get_logger().info(
            f'PhaseSpace Bridge ready | front_id={self.front_id} rear_id={self.rear_id} '
            f'| {len(self.obstacles)} obstacles')

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
    # 1. Read front / rear markers (msg.markers[front_id], msg.markers[rear_id]).
    #    Skip the frame if either is missing, has cond <= 0 (not tracked) or
    #    sits exactly at (0, 0, 0) (PhaseSpace's "lost" value).
    # 2. position = midpoint of the markers; yaw = atan2(fy-ry, fx-rx).
    # 3. Finite-difference world velocity, projected on the previous heading
    #    -> longitudinal speed v_long (0 on the first valid frame).
    # 4. Publish Odometry (frame 'odom', child 'base_footprint').
    # 5. If enabled, publish_synthetic_scan() with the same timestamp.
    # =========================================================================
    def marker_cb(self, msg: Markers):
        if len(msg.markers) <= max(self.front_id, self.rear_id):
            return  # Missing markers
        f, r = msg.markers[self.front_id], msg.markers[self.rear_id]
        for m in (f, r):
            if getattr(m, 'cond', 1.0) <= 0.0 or (m.x == 0.0 and m.y == 0.0 and m.z == 0.0):
                return  # Marker not tracked in this frame
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

        if self.use_synthetic_scan:
            self.publish_synthetic_scan(now)

    # =========================================================================
    # [CORE]   publish_synthetic_scan(stamp)
    # -------------------------------------------------------------------------
    # Builds a fake 240 deg LaserScan (YDLidar-like: +/-2.094 rad, 0.005825 rad
    # step, 0.08..8 m) by analytic ray-circle intersection:
    # 1. Start with every ray at range_max.
    # 2. For each obstacle: transform centre into the robot frame; skip if out
    #    of range; rays whose perpendicular distance < radius hit the circle.
    # 3. Hit range = along-ray projection - sqrt(r^2 - perp^2); keep the
    #   minimum.
    # 4. Publish on /hardware_scan (frame 'laser_link').
    # =========================================================================
    def publish_synthetic_scan(self, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp.to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min, scan.angle_max = -2.094395, 2.094395
        scan.angle_increment = 0.005825
        scan.range_min, scan.range_max = 0.08, 8.0
        
        num_rays = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = np.full(num_rays, scan.range_max)
        angles = scan.angle_min + np.arange(num_rays) * scan.angle_increment
        
        cos_yaw, sin_yaw = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        
        for ox, oy, r in self.obstacles:
            dx, dy = ox - self.robot_x, oy - self.robot_y
            loc_x = dx * cos_yaw + dy * sin_yaw
            loc_y = -dx * sin_yaw + dy * cos_yaw
            
            if math.hypot(loc_x, loc_y) > scan.range_max + r: continue
                
            perp_dists = np.abs(loc_x * np.sin(angles) - loc_y * np.cos(angles))
            hit_mask = perp_dists < r
            
            if np.any(hit_mask):
                hit_dists = (loc_x * np.cos(angles[hit_mask]) + loc_y * np.sin(angles[hit_mask])) - np.sqrt(r**2 - perp_dists[hit_mask]**2)
                valid = hit_dists > scan.range_min
                actual_indices = np.where(hit_mask)[0][valid]
                ranges[actual_indices] = np.minimum(ranges[actual_indices], hit_dists[valid])
                
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

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
