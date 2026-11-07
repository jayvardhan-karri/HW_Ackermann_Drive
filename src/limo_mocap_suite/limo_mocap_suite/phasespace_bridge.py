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
# Frame          : PhaseSpace frame is ROS right-handed: +x forward, +y left, yaw
#                  counter-clockwise. Nothing is mirrored or rotated here.
# Pose           : position = midpoint of the two markers
# Heading        : vector from marker rear_id (1) to marker front_id (0)
# Bad frames     : (PoseGate drops these "ghost" frames) a marker that jumps would swing
#                  the heading and slam the steering,
#                  so a frame is dropped if the two markers are not their usual distance
#                  apart, or the pose jumped further than the robot can move. No odometry
#                  then means the controllers hold a zero command.
# Parameters     : front_id (0), rear_id (1): marker ids (the Marker.id field)
#                  obstacles: flat [x, y, r, ...] in the PhaseSpace frame (none)
#                  obstacle_markers: [(0,1,2,3),(4,5,6,7)]: 4 markers on the rim
#                  of each cylinder; centre and radius come live from them (none)
#                  marker_baseline (0 = learn it from the first frames),
#                  max_baseline_error (0.03 m)
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

# Limits used by PoseGate
BASELINE_FRAMES = 20    # frames used to learn the marker distance
JUMP_POS = 0.05         # m: always allowed between frames, for noise
JUMP_SPEED = 1.5        # m/s: faster than this the robot cannot go
JUMP_YAW = 0.25         # rad: always allowed between frames
JUMP_YAW_RATE = 3.0     # rad/s: faster than this the robot cannot turn
RELOCK_FRAMES = 10      # this many rejected frames in a row: the robot really is there
V_FILTER_TAU = 0.1      # s: the finite-difference speed is noisy (telemetry only)


# =============================================================================
# [CORE]   class PoseGate
# -----------------------------------------------------------------------------
# Decides whether a marker frame can be believed. Plain math, no ROS.
# Keeps the last good pose and counts the frames it dropped (two reasons:
# wrong marker distance, impossible jump) so report_health can print them.
# =============================================================================
class PoseGate:
    # =========================================================================
    # [LIFE]   __init__(baseline=0.0, max_error=0.03)
    # -------------------------------------------------------------------------
    # baseline : expected distance between the two robot markers, m
    #            (0 = learn it from the first BASELINE_FRAMES frames)
    # max_error: allowed difference from the baseline, m
    # =========================================================================
    def __init__(self, baseline=0.0, max_error=0.03):
        self.baseline = float(baseline)
        self.max_error = float(max_error)
        self.learning = []
        self.last = None
        self.last_time = None
        self.jumps = 0
        self.dropped_baseline = 0
        self.dropped_jump = 0

    # =========================================================================
    # [CORE]   accept(fx, fy, rx, ry, now) -> (x, y, yaw) or None
    # -------------------------------------------------------------------------
    # Inputs : front marker (fx, fy) and rear marker (rx, ry) in m, now in s.
    # Output : pose (x m, y m, yaw rad) = midpoint + direction rear -> front,
    #          or None when the frame is not believed.
    # 1. Learn the marker distance (median of the first frames, none published).
    # 2. Drop a frame whose markers are not that far apart.
    # 3. Drop a frame that moved further than the robot could since the last good
    #    one. After RELOCK_FRAMES in a row, accept: it really was moved.
    # =========================================================================
    def accept(self, fx, fy, rx, ry, now):
        distance = math.hypot(fx - rx, fy - ry)
        if self.baseline <= 0.0:    # still learning: publish nothing yet
            self.learning.append(distance)
            if len(self.learning) < BASELINE_FRAMES:
                return None
            self.baseline = sorted(self.learning)[len(self.learning) // 2]    # median
        if abs(distance - self.baseline) > self.max_error:
            self.dropped_baseline += 1
            return None
        pose = ((fx + rx) / 2.0, (fy + ry) / 2.0, math.atan2(fy - ry, fx - rx))
        if self.last is not None:
            dt = now - self.last_time
            moved = math.hypot(pose[0] - self.last[0], pose[1] - self.last[1])
            turned = abs(math.atan2(math.sin(pose[2] - self.last[2]), math.cos(pose[2] - self.last[2])))
            # allowed step = noise allowance + fastest the robot can go * dt
            if moved > JUMP_POS + JUMP_SPEED * dt or turned > JUMP_YAW + JUMP_YAW_RATE * dt:
                self.jumps += 1
                self.dropped_jump += 1
                if self.jumps < RELOCK_FRAMES:
                    return None
        self.jumps = 0
        self.last, self.last_time = pose, now
        return pose


# =============================================================================
# [ROS-IO] class PhaseSpaceBridgeNode(Node)
# -----------------------------------------------------------------------------
# Turns two MoCap markers (front + rear of the LIMO) into a 2-D pose and
# velocity (/hardware_odom), and publishes the obstacle map
# (/hardware_obstacles). Frames go through PoseGate first.
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
        
        # marker ids of the robot: front_id = front marker, rear_id = rear marker
        self.declare_parameter('front_id', 0)
        self.declare_parameter('rear_id', 1)

        self.front_id = self.get_parameter('front_id').value
        self.rear_id = self.get_parameter('rear_id').value
        self.declare_parameter('marker_baseline', 0.0)
        self.declare_parameter('max_baseline_error', 0.03)
        self.gate = PoseGate(self.get_parameter('marker_baseline').value,
                             self.get_parameter('max_baseline_error').value)
        self.baseline_reported = False
        self.dropped_reported = 0
        self.last_seen = None
        self.v_long = 0.0
        self.create_timer(5.0, self.report_health)    # [LOG] health report every 5 s

        # Obstacles from parameters (fixed) and from 4 rim markers (live)
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

        # Last accepted pose, in the mocap frame (m, m, rad)
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
    # yaw in rad -> quaternion. Yaw-only rotation: [0, 0, sin(yaw/2), cos(yaw/2)].
    # =========================================================================
    def euler_to_quat(self, yaw):
        return [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]

    # =========================================================================
    # [CORE]   marker_cb(msg)
    # -------------------------------------------------------------------------
    # Callback for /phasespace/markers (one mocap frame in, at most one Odometry out).
    # Pose estimation from two markers:
    # 0. Update the obstacles that come from markers (even if the robot is lost).
    # 1. Find the front / rear markers by their id field. Skip the frame if either
    #    is missing, has cond <= 0 (not tracked) or sits exactly at (0, 0, 0).
    # 2. PoseGate.accept: ghost frames are dropped (marker distance, impossible
    #    jump). position = midpoint of the markers; yaw = atan2(fy-ry, fx-rx).
    # 3. Finite-difference world velocity, projected on the previous heading
    #    -> longitudinal speed v_long (0 on the first valid frame).
    # 4. Publish Odometry (frame 'odom', child 'base_footprint').
    # =========================================================================
    def marker_cb(self, msg: Markers):
        # id -> marker, only markers that are really seen (cond > 0, not at the origin)
        tracked = {m.id: m for m in msg.markers
                   if m.cond > 0.0 and not (m.x == 0.0 and m.y == 0.0 and m.z == 0.0)}
        self.update_marker_obstacles(tracked)
        if self.front_id not in tracked or self.rear_id not in tracked:
            return  # Robot markers not tracked in this frame
        f, r = tracked[self.front_id], tracked[self.rear_id]
        fx, fy = f.x, f.y
        rx, ry = r.x, r.y

        now = self.get_clock().now()
        self.last_seen = now
        accepted = self.gate.accept(fx, fy, rx, ry, now.nanoseconds / 1e9)
        if accepted is None:
            return
        new_x, new_y, new_yaw = accepted
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if not self.pose_valid:
            # First valid frame: no previous pose to difference against
            self.robot_x, self.robot_y, self.robot_yaw = new_x, new_y, new_yaw
            self.pose_valid = True
            dt = 0.0

        v_x = (new_x - self.robot_x) / dt if dt > 0 else 0.0
        v_y = (new_y - self.robot_y) / dt if dt > 0 else 0.0
        # Dot product with the heading unit vector
        v_long = v_x * math.cos(self.robot_yaw) + v_y * math.sin(self.robot_yaw)
        if dt > 0:    # low-pass filter, time constant V_FILTER_TAU
            self.v_long += (v_long - self.v_long) * dt / (dt + V_FILTER_TAU)

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
        odom.twist.twist.linear.x = self.v_long
        
        self.odom_pub.publish(odom)

    # =========================================================================
    # [CORE]   update_marker_obstacles(tracked)
    # -------------------------------------------------------------------------
    # For each 4-marker group fully tracked in this frame: centre = mean, radius =
    # mean distance from the centre. A group with an occluded marker keeps its
    # last known circle. Republishes when a circle first appears or moves > 2 cm.
    # tracked: dict marker id -> marker (x, y in m).
    # =========================================================================
    def update_marker_obstacles(self, tracked):
        changed = False
        for i, group in enumerate(self.marker_groups):
            if all(g in tracked for g in group):
                c = circle_from_markers([(tracked[g].x, tracked[g].y) for g in group])
                old = self.marker_circles.get(i)
                # c = (x, y, r); any of the three changed by more than 2 cm
                if old is None or max(abs(c[k] - old[k]) for k in range(3)) > 0.02:
                    self.marker_circles[i] = c
                    changed = True
        if changed:
            self.publish_obstacles()

    # =========================================================================
    # [ROS-IO] publish_obstacles()
    # -------------------------------------------------------------------------
    # Fixed obstacles + marker circles as a flat list [x, y, r, ...] (m) on the
    # latched /hardware_obstacles topic.
    # =========================================================================
    def publish_obstacles(self):
        flat = list(self.fixed_obstacles)
        for i in sorted(self.marker_circles):
            flat.extend(self.marker_circles[i])
        self.obstacle_pub.publish(Float32MultiArray(data=[float(v) for v in flat]))
        self.published_obstacles = flat

    # =========================================================================
    # [LOG]    report_health()
    # -------------------------------------------------------------------------
    # 5 s timer: the marker distance once learned, frames dropped since last time,
    # and whether the robot's markers are being seen at all.
    # =========================================================================
    def report_health(self):
        log = self.get_logger()
        if self.gate.baseline > 0.0 and not self.baseline_reported:
            log.info(f'Robot markers are {self.gate.baseline:.3f} m apart')
            self.baseline_reported = True
        dropped = self.gate.dropped_baseline + self.gate.dropped_jump
        if dropped > self.dropped_reported:
            log.warn(f'Dropped mocap frames so far: {self.gate.dropped_baseline} wrong marker distance, '
                     f'{self.gate.dropped_jump} impossible jump')
            self.dropped_reported = dropped
        if self.last_seen is None:
            log.warn(f'Robot markers {self.front_id}, {self.rear_id} not seen yet')

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
