# #############################################################################
# [FILE]   obstacle_cbf.py
# -----------------------------------------------------------------------------
# FILE  : obstacle_cbf.py (limo_mocap_suite and limo_gz_sim)
# ROLE  : Control Barrier Function safety filter for an Ackermann robot, using
#         the known obstacle positions (no LiDAR). Sits between every controller
#         and /cmd_vel. Byte-identical copy in the simulation package.
#
# Math, for a point p at distance a ahead of the rear axle, heading t, left n:
#   p_dot = v t + a omega n
#   h     = |p - o| - R              h >= 0 is safe (R = clearance radius)
#   CBF   : h_dot + alpha h >= 0  =>  (d.t) v + a (d.n) omega + alpha h >= 0
#           with d = (p - o) / |p - o|: one half-plane in (v, omega).
# Input set: 0 <= v <= v_nominal, |omega| <= kappa_max v (a triangle: an
# Ackermann robot cannot turn on the spot). Safe command = the point of
# triangle-minus-half-planes closest to the nominal command, in closed form.
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
import re
import time

import numpy as np
import yaml

PATH_AHEAD = 3.0    # m of path ahead of the robot used to choose the side to pass on
CONE_HYST = 0.08    # rad: the collision cone counts this much wider once a side is committed,
                    # so the test does not flip CLEAR/ACTIVE every tick at the cone edge


# =============================================================================
# [UTIL]   parse_obstacles(value) -> flat list [x, y, r, ...]
# -----------------------------------------------------------------------------
# Accepts None, '', a list of numbers or a string like "[1.97, 0.81, 0.35]"
# (parsed with yaml.safe_load). Units: x, y, r in m.
# Raises ValueError unless the length is a multiple of 3 with radii > 0.
# =============================================================================
def parse_obstacles(value):
    if value is None or value == '' or value == []:
        return []
    if isinstance(value, str):
        value = yaml.safe_load(value)
        if value is None:
            return []
    flat = [float(v) for v in value]
    if len(flat) % 3 != 0 or any(r <= 0.0 for r in flat[2::3]):
        raise ValueError(f"obstacles must be flat [x, y, radius, ...] with radius > 0, got {flat}")
    return flat


# =============================================================================
# [UTIL]   parse_marker_groups(value) -> list of 4-tuples of marker ids
# -----------------------------------------------------------------------------
# Accepts None, '', "[(0,1,2,3),(4,5,6,7)]", "0,1,2,3,4,5,6,7" or a list of
# numbers / strings (what `-p name:=[...]` produces). Raises ValueError unless
# every group has exactly 4 ids.
# =============================================================================
def parse_marker_groups(value):
    if value is None or value == '' or value == []:
        return []
    text = value if isinstance(value, str) else ','.join(str(v) for v in value)
    groups = re.findall(r'\(([^()]*)\)', text)
    if groups:
        ids = [[int(i) for i in re.findall(r'\d+', g)] for g in groups]
    else:
        flat = [int(i) for i in re.findall(r'\d+', text)]
        ids = [flat[i:i + 4] for i in range(0, len(flat), 4)]
    if any(len(g) != 4 for g in ids):
        raise ValueError(f"obstacle_markers needs groups of exactly 4 marker ids, got {ids}")
    return [tuple(g) for g in ids]


# =============================================================================
# [UTIL]   circle_from_markers(points) -> (x, y, r)
# -----------------------------------------------------------------------------
# Cylinder from markers on its rim: centre = mean, radius = mean distance from
# the centre.
# =============================================================================
def circle_from_markers(points):
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    r = sum(math.hypot(p[0] - cx, p[1] - cy) for p in points) / len(points)
    return cx, cy, r


# =============================================================================
# [UTIL]   declare_flexible_parameter(node, name)
# -----------------------------------------------------------------------------
# Declares a parameter with no default that accepts any type: an empty list
# has no ROS parameter type. Humble needs dynamic_typing; Foxy has no such
# field and allows it anyway.
# =============================================================================
def declare_flexible_parameter(node, name):
    from rcl_interfaces.msg import ParameterDescriptor
    try:
        node.declare_parameter(name, None, ParameterDescriptor(dynamic_typing=True))
    except (AttributeError, AssertionError, TypeError):
        node.declare_parameter(name, None)
    return node.get_parameter(name).value


# =============================================================================
# [CORE]   class CbfFilter
# -----------------------------------------------------------------------------
# The filter itself: pure math, no ROS.
#   wheelbase, max_steer : robot (max_steer = what the plant really reaches)
#   ref_offset           : tracked point ahead of the rear axle (m)
#   robot_radius         : body radius around the tracked point (m)
#   safety_margin        : extra clearance beyond the body (m)
#   alpha                : CBF gain (1/s): lower = slows down earlier
#   steer_weight         : cost of changing omega vs v (< 1 prefers steering)
# Barrier h = |p - o| - R, with R = obstacle r + robot_radius + safety_margin.
# Two layers, because a distance barrier alone reacts too late for a robot that
# cannot turn on the spot (it arrives at the boundary pointing inwards and stops):
#   approach : start turning at the distance the turning radius needs, so the
#              heading leaves the obstacle's collision cone in time
#   barrier  : the half-plane above, the actual safety guarantee
# Defaults here (alpha 0.6, steer_weight 0.15, max_steer 0.48869) can differ from
# the node values: ObstacleCbfLayer passes the real ones.
# =============================================================================
class CbfFilter:
    # =========================================================================
    # [LIFE]   CbfFilter.__init__(wheelbase, max_steer, ref_offset, robot_radius, safety_margin,
    #          alpha, steer_weight)
    # -------------------------------------------------------------------------
    # Stores the parameters (see the class banner) and computes the curvature limit
    # kappa_max = tan(max_steer)/wheelbase (1/m). side: obstacle index -> committed
    # turn direction (+1 left, -1 right), empty at the start.
    # =========================================================================
    def __init__(self, wheelbase=0.20, max_steer=0.48869, ref_offset=0.10, robot_radius=0.20,
                 safety_margin=0.10, alpha=0.6, steer_weight=0.15):
        self.L = wheelbase
        self.kappa_max = math.tan(max_steer) / wheelbase
        self.max_steer = max_steer
        self.ref_offset = ref_offset
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.alpha = alpha
        self.steer_weight = steer_weight
        self.side = {}          # obstacle index -> committed turn (+1 left, -1 right)

    # =========================================================================
    # [CORE]   trigger_distance(R) -> m from the clearance circle
    # -------------------------------------------------------------------------
    # Head-on, a turn of radius rho clears a circle of radius R only if started
    # at centre distance >= sqrt(R^2 + 2 R rho). Uses 85 % of the steering limit
    # (rho = 1/(0.85 kappa_max)) and adds a 0.25 m buffer for servo lag.
    # Returned value is measured from the clearance circle: sqrt(...) - R + 0.25.
    # =========================================================================
    def trigger_distance(self, R):
        rho = 1.0 / (0.85 * self.kappa_max)
        return math.sqrt(R * R + 2.0 * R * rho) - R + 0.25

    # =========================================================================
    # [CORE]   filter(v_nom, steer_nom, x, y, yaw, obstacles, path=None)
    #                   -> (v, steer, clearance, status)
    # -------------------------------------------------------------------------
    # Inputs: nominal speed (m/s) and steering (rad) from the controller, pose of the
    # tracked point (m, m, rad), obstacles: list of (x, y, r) in the same frame as the
    # pose. clearance = smallest distance from the tracked point to a cylinder edge (m).
    # For each obstacle within its trigger distance:
    #   barrier half-plane  (d.t) v + a (d.n) omega + alpha h >= 0
    #                       d = unit vector obstacle->robot, t/n = heading/left normal,
    #                       a = ref_offset, omega = v tan(steer)/L
    #   cone violation      heading inside the collision cone (half-angle asin(R/dist),
    #                       plus CONE_HYST), on the committed side (side chosen once:
    #                       away from the obstacle)
    # path (optional, N x >=3: x, y, yaw in the pose frame): the stretch of path the
    # robot drives NEXT. The side is chosen from which side of it the obstacle centre
    # lies, where it passes the obstacle. The robot's heading alone flips the choice
    # when an obstacle is just off the line and the heading is a few degrees off (it
    # then went round the outside).
    # Reference = nominal, or a turn to the committed side if the cone is
    # violated (strength = violation/0.25 rad, capped at 1), slowing by up to 50 %
    # while turning hard. Then _project() gives the closest safe (v, omega) and
    # steer = atan(L omega / v), clipped to +-max_steer.
    # Status: NO_OBSTACLES, CLEAR, CBF_ACTIVE, or CBF_STOP (no safe forward motion).
    # =========================================================================
    def filter(self, v_nom, steer_nom, x, y, yaw, obstacles, path=None):
        if not obstacles:
            return v_nom, steer_nom, float('inf'), 'NO_OBSTACLES'
        if v_nom <= 1e-4:
            clearance = min(math.hypot(x - o[0], y - o[1]) - o[2] for o in obstacles)
            return v_nom, steer_nom, clearance, 'CLEAR'
        c, s = math.cos(yaw), math.sin(yaw)
        w_nom = v_nom * math.tan(steer_nom) / self.L
        cons, near, clearance = [], set(), float('inf')
        w_ref, v_ref, worst = w_nom, v_nom, 0.0
        for i, (ox, oy, r) in enumerate(obstacles):
            dx, dy = x - ox, y - oy
            dist = max(math.hypot(dx, dy), 1e-6)
            clearance = min(clearance, dist - r)
            R = r + self.robot_radius + self.safety_margin
            h = dist - R
            if h > self.trigger_distance(R):
                continue
            near.add(i)
            # half-plane A v + B omega + C >= 0 with A = d.t, B = a (d.n), C = alpha h
            cons.append(((dx * c + dy * s) / dist, self.ref_offset * (-dx * s + dy * c) / dist, self.alpha * h))
            bearing = math.atan2(-dy, -dx) - yaw               # obstacle direction vs heading
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if i not in self.side:
                # Obstacle left of the path -> pass right, and vice versa. No path or on
                # the line: left of the heading -> turn right; dead ahead -> nominal's way
                path_side = self._path_side(path, ox, oy)
                if path_side != 0.0:
                    self.side[i] = -path_side
                elif abs(bearing) > 0.02:
                    self.side[i] = -1.0 if bearing > 0.0 else 1.0
                else:
                    self.side[i] = 1.0 if w_nom >= 0.0 else -1.0
            # half-angle of the collision cone seen from the robot (rad)
            cone = math.asin(min(1.0, R / dist))
            # > 0 when the heading points into the cone (side * bearing < 0 is the safe direction);
            # obstacles behind the robot (|bearing| >= 90 deg) are ignored
            violation = cone + CONE_HYST + self.side[i] * bearing if abs(bearing) < math.pi / 2.0 else 0.0
            if violation > worst:
                worst = violation
                strength = min(1.0, violation / 0.25)
                w_turn = self.side[i] * self.kappa_max * v_nom * strength
                w_ref = max(w_nom, w_turn) if self.side[i] > 0.0 else min(w_nom, w_turn)
                v_ref = v_nom * (1.0 - 0.5 * strength)
        # forget the committed side of obstacles that are no longer near
        for i in list(self.side):
            if i not in near:
                del self.side[i]
        if not cons:
            return v_nom, steer_nom, clearance, 'CLEAR'
        # is the nominal command already inside every half-plane?
        safe = all(A * v_nom + B * w_nom + C >= 0.0 for A, B, C in cons)
        if worst <= 0.0 and safe:
            return v_nom, steer_nom, clearance, 'CLEAR'
        u = self._project(v_ref, w_ref, v_nom, cons)
        if u is None or u[0] < 1e-3:
            return 0.0, steer_nom, clearance, 'CBF_STOP'
        steer = math.atan(self.L * u[1] / u[0])
        return u[0], max(-self.max_steer, min(self.max_steer, steer)), clearance, 'CBF_ACTIVE'

    # =========================================================================
    # [UTIL]   _path_side(path, ox, oy) -> +1 left of the path, -1 right, 0 unknown
    #          (helper of filter)
    # -------------------------------------------------------------------------
    # Cross product of the path direction at its point nearest the obstacle centre
    # and the vector to the centre. 0 if no path or the centre is within 2 cm of it.
    # =========================================================================
    @staticmethod
    def _path_side(path, ox, oy):
        if path is None or len(path) < 2:
            return 0.0
        d2 = (path[:, 0] - ox) ** 2 + (path[:, 1] - oy) ** 2
        k = int(d2.argmin())
        px, py, pyaw = path[k, 0], path[k, 1], path[k, 2]
        cross = math.cos(pyaw) * (oy - py) - math.sin(pyaw) * (ox - px)
        if abs(cross) < 0.02:
            return 0.0
        return 1.0 if cross > 0.0 else -1.0

    # =========================================================================
    # [CORE]   _project(v_ref, w_ref, v_max, cons) -> (v, omega) or None
    #          (helper of filter)
    # -------------------------------------------------------------------------
    # Closest point (cost (v-v_ref)^2 + steer_weight (omega-w_ref)^2) in the
    # triangle (0,0), (v_max, +-kappa_max v_max) clipped by every half-plane.
    # Scaled coordinates q = (v, k omega), k = sqrt(steer_weight), turn the weighted
    # cost into a plain distance. Steps: clip the triangle by each constraint (_clip);
    # empty -> None (no safe motion); else the closest point of the polygon boundary
    # (_closest_on_segment) to the reference, or the reference itself if it is
    # inside (_inside).
    # =========================================================================
    def _project(self, v_ref, w_ref, v_max, cons):
        k = math.sqrt(self.steer_weight)      # scaled coords: q = (v, k omega)
        poly = [(0.0, 0.0), (v_max, -k * self.kappa_max * v_max), (v_max, k * self.kappa_max * v_max)]
        for A, B, C in cons:
            poly = self._clip(poly, A, B / k, C)
            if len(poly) == 0:
                return None
        ref = (v_ref, k * w_ref)
        if len(poly) == 1:
            best = poly[0]
        else:
            best, best_d = None, float('inf')
            for j in range(len(poly)):
                q = self._closest_on_segment(ref, poly[j], poly[(j + 1) % len(poly)])
                d = (q[0] - ref[0]) ** 2 + (q[1] - ref[1]) ** 2
                if d < best_d:
                    best, best_d = q, d
        if self._inside(ref, poly):
            best = ref
        return best[0], best[1] / k

    # =========================================================================
    # [UTIL]   _clip(poly, a, b, c) -> polygon
    #          (helper of _project)
    # -------------------------------------------------------------------------
    # Clips a convex polygon (list of (x, y)) by the half-plane a x + b y + c >= 0
    # (Sutherland-Hodgman): keep inside points, add the edge crossings.
    # =========================================================================
    @staticmethod
    def _clip(poly, a, b, c):
        out = []
        for j in range(len(poly)):
            p, q = poly[j], poly[(j + 1) % len(poly)]
            fp, fq = a * p[0] + b * p[1] + c, a * q[0] + b * q[1] + c
            if fp >= 0.0:
                out.append(p)
            if (fp >= 0.0) != (fq >= 0.0):
                t = fp / (fp - fq)
                out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
        return out

    # =========================================================================
    # [UTIL]   _closest_on_segment(p, s0, s1) -> (x, y)
    #          (helper of _project)
    # -------------------------------------------------------------------------
    # Closest point to p on the segment s0-s1: project p onto the segment line,
    # parameter t clamped to [0, 1].
    # =========================================================================
    @staticmethod
    def _closest_on_segment(p, s0, s1):
        ex, ey = s1[0] - s0[0], s1[1] - s0[1]
        l2 = ex * ex + ey * ey
        t = 0.0 if l2 == 0.0 else max(0.0, min(1.0, ((p[0] - s0[0]) * ex + (p[1] - s0[1]) * ey) / l2))
        return s0[0] + t * ex, s0[1] + t * ey

    # =========================================================================
    # [UTIL]   _inside(p, poly) -> bool
    #          (helper of _project)
    # -------------------------------------------------------------------------
    # Point-in-convex-polygon test: p is inside if the cross products with all edges
    # have the same sign. False for fewer than 3 vertices (a point or a segment).
    # =========================================================================
    @staticmethod
    def _inside(p, poly):
        if len(poly) < 3:
            return False
        sign = 0.0
        for j in range(len(poly)):
            a, b = poly[j], poly[(j + 1) % len(poly)]
            cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
            if cross != 0.0:
                if sign != 0.0 and (cross > 0.0) != (sign > 0.0):
                    return False
                sign = cross
        return True


# =============================================================================
# [ROS-IO] class ObstacleCbfLayer
# -----------------------------------------------------------------------------
# What a controller node uses. Declares robot_radius (0.20 m), safety_margin (0.10 m),
# cbf_alpha (0.6 1/s), cbf_max_steer (0 = use max_steer), cbf_hold (0.6 s),
# cbf_steer_rate (2.0 rad/s) and obstacles (flat list; default none),
# subscribes to the latched /hardware_obstacles (flat [x, y, r, ...]) and merges
# both. apply() takes the pose in the controller frame plus that frame's origin,
# so obstacles given in the lab frame still line up when odometry is re-zeroed.
# Steering smoothing: while an obstacle is near, or cbf_hold s after the filter last
# acted (CBF_ACTIVE / CBF_STOP), the steering output changes by at most
# cbf_steer_rate (rad/s) times the time since the last apply(). Else pass-through.
# =============================================================================
class ObstacleCbfLayer:
    # =========================================================================
    # [LIFE]   ObstacleCbfLayer.__init__(node, wheelbase, max_steer, ref_offset)
    # -------------------------------------------------------------------------
    # Declares the parameters listed in the class banner, builds the CbfFilter (steering
    # limit = cbf_max_steer if > 0 else max_steer), reads the obstacles parameter
    # (any type allowed, see declare_flexible_parameter) and subscribes to
    # /hardware_obstacles with a latched QoS (TRANSIENT_LOCAL, depth 1) so a late
    # node still gets the last list. Smoothing memory starts empty.
    # =========================================================================
    def __init__(self, node, wheelbase, max_steer, ref_offset):
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import Float32MultiArray
        node.declare_parameter('robot_radius', 0.20)
        node.declare_parameter('safety_margin', 0.10)
        node.declare_parameter('cbf_alpha', 0.6)
        node.declare_parameter('cbf_max_steer', 0.0)
        node.declare_parameter('cbf_hold', 0.6)
        node.declare_parameter('cbf_steer_rate', 2.0)
        g = lambda n: float(node.get_parameter(n).value)  # noqa: E731
        limit = g('cbf_max_steer') if g('cbf_max_steer') > 0.0 else max_steer
        self.cbf = CbfFilter(wheelbase, limit, ref_offset, g('robot_radius'), g('safety_margin'), g('cbf_alpha'))
        self.param_obstacles = parse_obstacles(declare_flexible_parameter(node, 'obstacles'))
        self.topic_obstacles = []
        self.node = node
        self.hold = g('cbf_hold')
        self.steer_rate = g('cbf_steer_rate')
        self._prev_steer = None
        self._prev_t = None
        self._hold_until = -float('inf')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Float32MultiArray, '/hardware_obstacles', self._cb, latched)
        if self.param_obstacles:
            node.get_logger().info(f'CBF: {len(self.param_obstacles) // 3} obstacle(s) from parameter')

    # =========================================================================
    # [ROS-IO] _cb(msg: Float32MultiArray)
    #          (helper: subscription callback, only used in this class)
    # -------------------------------------------------------------------------
    # Callback for /hardware_obstacles: stores the flat [x, y, r, ...] list (lab frame, m)
    # as topic_obstacles and logs when the number of obstacles changes.
    # =========================================================================
    def _cb(self, msg):
        data = [float(v) for v in msg.data]
        if len(data) // 3 != len(self.topic_obstacles) // 3:
            self.node.get_logger().info(f'CBF: {len(data) // 3} obstacle(s) from /hardware_obstacles')
        self.topic_obstacles = data

    # =========================================================================
    # [CORE]   apply(v, steer, x, y, yaw, x0, y0, yaw0)
    #                   -> (v_safe, steer_safe, clearance, status)
    # -------------------------------------------------------------------------
    # Moves the lab-frame obstacles into the controller frame (origin x0, y0,
    # yaw0) and runs the filter, with the path being followed (same frame) so the
    # filter passes each obstacle on the side away from it. Then smooths the steering
    # (see the class banner): rate limit and clip, not applied when the speed is ~0.
    # Smoothing is active if any obstacle is within trigger_distance + 0.3 m or the
    # hold time (cbf_hold after the last CBF_ACTIVE / CBF_STOP) has not expired.
    # The time step for the rate limit is capped at 0.1 s.
    # =========================================================================
    def apply(self, v, steer, x, y, yaw, x0=0.0, y0=0.0, yaw0=0.0):
        flat = self.param_obstacles + self.topic_obstacles
        c, s = math.cos(yaw0), math.sin(yaw0)
        obs = []
        for i in range(0, len(flat) - 2, 3):
            dx, dy = flat[i] - x0, flat[i + 1] - y0
            obs.append((c * dx + s * dy, -s * dx + c * dy, flat[i + 2]))
        v_out, steer_out, clearance, status = self.cbf.filter(v, steer, x, y, yaw, obs, self._path_ahead())
        now = time.monotonic()
        if status in ('CBF_ACTIVE', 'CBF_STOP'):
            self._hold_until = now + self.hold
        near = False
        for ox, oy, r in obs:
            R = r + self.cbf.robot_radius + self.cbf.safety_margin
            if math.hypot(x - ox, y - oy) - R <= self.cbf.trigger_distance(R) + 0.3:
                near = True
                break
        if (near or now <= self._hold_until) and abs(v_out) > 1e-3:
            lim = self.cbf.max_steer
            if self._prev_steer is not None and self._prev_t is not None:
                dt = min(max(now - self._prev_t, 0.0), 0.1)
                step = self.steer_rate * dt
                steer_out = max(self._prev_steer - step, min(self._prev_steer + step, steer_out))
            steer_out = max(-lim, min(lim, steer_out))
        self._prev_steer, self._prev_t = steer_out, now
        return v_out, steer_out, clearance, status

    # =========================================================================
    # [UTIL]   _path_ahead() -> N x 5 rows of the path from the robot on, or None
    #          (helper of apply)
    # -------------------------------------------------------------------------
    # PATH_AHEAD metres from the robot's current index (run manager), wrapping on a
    # closed path. Only what comes next: further on, the same path can pass the
    # same obstacle again the other way round (approach + track prefix).
    # Rows = PATH_AHEAD / waypoint spacing (at least 2). None if there is no path yet.
    # =========================================================================
    def _path_ahead(self):
        traj, run = getattr(self.node, 'traj', None), getattr(self.node, 'run', None)
        if traj is None or run is None or len(traj.waypoints) < 2:
            return None
        n = len(traj.waypoints)
        k = max(2, int(PATH_AHEAD / max(float(getattr(run, 'spacing', 0.01)), 1e-3)))
        rows = np.arange(int(getattr(run, 'idx', 0)), int(getattr(run, 'idx', 0)) + k)
        rows = rows % n if traj.is_closed else rows[rows < n]
        return traj.waypoints[rows] if len(rows) >= 2 else None
