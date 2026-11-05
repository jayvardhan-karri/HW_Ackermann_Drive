# #############################################################################
# [FILE]   trajectory_subscriber.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/trajectory_subscriber.py
# ROLE  : Shared helper: receives /reference_path and computes tracking errors.
#
# Not a node. Every controller owns one instance (composition).
# Waypoint table layout: waypoints[i] = [x, y, yaw, v_ref, curvature].
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
from nav_msgs.msg import Path

# =============================================================================
# [CORE]   class TrajectorySubscriber
# -----------------------------------------------------------------------------
# Holds the current reference path and answers 'where am I relative to it?'.
# =============================================================================
class TrajectorySubscriber:
    """Dynamically computes reference states and curvature from a drawn Path."""
    # =========================================================================
    # [LIFE]   __init__(node, v_nominal=0.40)
    # -------------------------------------------------------------------------
    # Stores the owning node and nominal speed, starts with an empty waypoint
    # table (path_ready = False, is_closed = False, goal_reached = False) and
    # subscribes to /reference_path on that node.
    # =========================================================================
    def __init__(self, node, v_nominal=0.40, ref_offset=0.10):
        self.node = node
        # Distance of the tracked point (marker midpoint) ahead of the rear axle (m)
        self.ref_offset = ref_offset
        self.v_nominal = v_nominal
        self.waypoints = np.zeros((0, 5))
        self.path_ready = False
        self.is_closed = False
        self.goal_reached = False
        self.new_path = False   # set by path_cb, cleared by RunManager
        self.last_idx = 0
        self.sub = self.node.create_subscription(Path, '/reference_path', self.path_cb, 10)

    # =========================================================================
    # [CORE]   path_cb(msg: Path)
    # -------------------------------------------------------------------------
    # Callback for /reference_path. Converts poses into the waypoint table:
    # 1. Ignore paths with < 4 poses.
    # 2. Finite-difference derivatives (np.gradient) -> unit tangent, yaw.
    # 3. Signed curvature kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^1.5.
    # 4. Speed profile v_ref = v_nominal / (1 + 0.75 |kappa|).
    # 5. Closed-loop test: first and last point closer than
    #    max(0.15 m, 3 x median spacing) -> is_closed (indices wrap), else open
    #    (indices clamp at the ends and goal_reached is set at the last point).
    # 6. Store table, set path_ready = True, reset search index.
    # =========================================================================
    def path_cb(self, msg):
        n = len(msg.poses)
        if n < 4: return
        
        x = np.array([p.pose.position.x for p in msg.poses])
        y = np.array([p.pose.position.y for p in msg.poses])
        
        # Approximate path derivatives via finite difference
        dx = np.gradient(x)
        dy = np.gradient(y)
        ds = np.hypot(dx, dy)
        ds[ds == 0] = 1e-5
        
        dx_ds = dx / ds
        dy_ds = dy / ds
        
        ddx = np.gradient(dx_ds) / ds
        ddy = np.gradient(dy_ds) / ds
        
        yaw = np.arctan2(dy, dx)
        denom = (dx_ds**2 + dy_ds**2)**1.5 + 1e-9
        curvature = (dx_ds * ddy - dy_ds * ddx) / denom
        
        # Smooth curvature-based speed scaling
        v_ref = self.v_nominal / (1.0 + 0.75 * np.abs(curvature))
        
        # Closed loop if the stroke ends where it started
        spacing = float(np.median(np.hypot(np.diff(x), np.diff(y))))
        gap = math.hypot(x[-1] - x[0], y[-1] - y[0])
        self.is_closed = gap < max(0.15, 3.0 * spacing)

        self.waypoints = np.column_stack((x, y, yaw, v_ref, curvature))
        self.path_ready = True
        self.goal_reached = False
        self.last_idx = 0
        self.new_path = True
        kind = 'closed loop' if self.is_closed else 'open path'
        self.node.get_logger().info(f"Received new path! Waypoints generated: {n} ({kind})")

    # =========================================================================
    # [CORE]   load_table(table, closed, start_idx)
    # -------------------------------------------------------------------------
    # Replaces the table being tracked: RunManager swaps between the way to the
    # path and the path itself. Rows are [x, y, yaw, v, kappa].
    # =========================================================================
    def load_table(self, table, closed, start_idx=0):
        self.waypoints = table
        self.is_closed = closed
        self.last_idx = int(start_idx)
        self.goal_reached = False
        self.path_ready = True

    # =========================================================================
    # [UTIL]   index(i) -> int
    # -------------------------------------------------------------------------
    # Maps any waypoint index onto the table: wraps modulo N on a closed loop,
    # clamps to [0, N-1] on an open path. Used by the MPC horizon samplers.
    # =========================================================================
    def index(self, i):
        n = len(self.waypoints)
        return int(i) % n if self.is_closed else min(max(int(i), 0), n - 1)

    # =========================================================================
    # [UTIL]   normalize_angle(angle) -> float
    # -------------------------------------------------------------------------
    # Wraps any angle into (-pi, pi] via atan2(sin, cos).
    # =========================================================================
    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    # =========================================================================
    # [CORE]   compute_tracking_errors(x, y, yaw, lookahead_pts=8)
    # -------------------------------------------------------------------------
    # Returns (cte, heading_err, lookahead_point, nearest_idx).
    # 1. Nearest waypoint search in a sliding window [last_idx-25,
    #    last_idx+160) (wrapped or clamped via index()); fall back to a global
    #    search if > 1.5 m away.
    # 2. Lookahead point = index(nearest_idx + lookahead_pts).
    # 3. cte = lateral offset of the NEAREST point in the path frame
    #    (positive = path lies to the robot's left).
    # 4. heading_err = wrap(yaw_nearest - atan(ref_offset*kappa_nearest) - yaw).
    #    Zero when tracking perfectly: a look-ahead yaw would be non-zero on any
    #    curve and force a steady inside offset. The atan term is the angle
    #    between the tracked point's travel direction and the robot's yaw.
    # 5. Open path only: once the nearest point is the last one and the robot
    #    is within 5 cm of (or past) it, set goal_reached = True.
    # Empty table -> zeros and the robot's own pose.
    # =========================================================================
    def compute_tracking_errors(self, x, y, yaw, lookahead_pts=8):
        n = len(self.waypoints)
        if n == 0:
            return 0.0, 0.0, (x, y, yaw, 0.0, 0.0), 0
            
        search_window = 160
        window = np.arange(self.last_idx - 25, self.last_idx + search_window)
        indices = window % n if self.is_closed else np.clip(window, 0, n - 1)
        pts = self.waypoints[indices, :2]
        dists_sq = (pts[:, 0] - x)**2 + (pts[:, 1] - y)**2
        best_local = int(np.argmin(dists_sq))
        nearest_idx = int(indices[best_local])
        
        if dists_sq[best_local] > 2.25:
            all_dists = (self.waypoints[:, 0] - x)**2 + (self.waypoints[:, 1] - y)**2
            nearest_idx = int(np.argmin(all_dists))
            
        self.last_idx = nearest_idx
        target_idx = self.index(nearest_idx + lookahead_pts)
        
        x_r, y_r, yaw_r, v_r, kappa_r = self.waypoints[nearest_idx]
        x_lh, y_lh, yaw_lh, v_lh, kappa_lh = self.waypoints[target_idx]
        
        dx_err = x_r - x
        dy_err = y_r - y
        cte = -math.sin(yaw_r) * dx_err + math.cos(yaw_r) * dy_err
        heading_err = self.normalize_angle(yaw_r - math.atan(self.ref_offset * kappa_r) - yaw)

        if not self.is_closed and nearest_idx == n - 1:
            # Signed distance along the final tangent: >= 0 means at / past the end
            along = math.cos(yaw_r) * (x - x_r) + math.sin(yaw_r) * (y - y_r)
            if along >= -0.05:
                self.goal_reached = True
        
        return float(cte), float(heading_err), (float(x_lh), float(y_lh), float(yaw_lh), float(v_lh), float(kappa_lh)), nearest_idx
