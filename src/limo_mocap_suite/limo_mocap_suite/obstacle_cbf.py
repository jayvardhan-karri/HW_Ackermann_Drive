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
# #############################################################################

import math
import re

import yaml


# =============================================================================
# [UTIL]   parse_obstacles(value) -> flat list [x, y, r, ...]
# -----------------------------------------------------------------------------
# Accepts None, '', a list of numbers or a string like "[1.97, 0.81, 0.35]".
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
# Two layers, because a distance barrier alone reacts too late for a robot that
# cannot turn on the spot (it arrives at the boundary pointing inwards and stops):
#   approach : start turning at the distance the turning radius needs, so the
#              heading leaves the obstacle's collision cone in time
#   barrier  : the half-plane above, the actual safety guarantee
# =============================================================================
class CbfFilter:
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
    # and adds a buffer for servo lag.
    # =========================================================================
    def trigger_distance(self, R):
        rho = 1.0 / (0.85 * self.kappa_max)
        return math.sqrt(R * R + 2.0 * R * rho) - R + 0.25

    # =========================================================================
    # [CORE]   filter(v_nom, steer_nom, x, y, yaw, obstacles)
    #          -> (v, steer, clearance, status)
    # -------------------------------------------------------------------------
    # obstacles: list of (x, y, r) in the same frame as the pose. clearance =
    # smallest distance from the tracked point to a cylinder edge (m).
    # For each obstacle within its trigger distance:
    #   barrier half-plane  (d.t) v + a (d.n) omega + alpha h >= 0
    #   cone violation      heading inside the collision cone, on the committed
    #                       side (side chosen once: away from the obstacle)
    # Reference = nominal, or a turn to the committed side if the cone is
    # violated, slowing while turning hard. Then the closest safe command.
    # Status: NO_OBSTACLES, CLEAR, CBF_ACTIVE, or CBF_STOP (no safe forward motion).
    # =========================================================================
    def filter(self, v_nom, steer_nom, x, y, yaw, obstacles):
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
            cons.append(((dx * c + dy * s) / dist, self.ref_offset * (-dx * s + dy * c) / dist, self.alpha * h))
            bearing = math.atan2(-dy, -dx) - yaw               # obstacle direction vs heading
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if i not in self.side:
                # Obstacle on the left -> turn right, and vice versa; dead ahead -> nominal's way
                if abs(bearing) > 0.02:
                    self.side[i] = -1.0 if bearing > 0.0 else 1.0
                else:
                    self.side[i] = 1.0 if w_nom >= 0.0 else -1.0
            cone = math.asin(min(1.0, R / dist))
            violation = cone + self.side[i] * bearing if abs(bearing) < math.pi / 2.0 else 0.0
            if violation > worst:
                worst = violation
                strength = min(1.0, violation / 0.25)
                w_turn = self.side[i] * self.kappa_max * v_nom * strength
                w_ref = max(w_nom, w_turn) if self.side[i] > 0.0 else min(w_nom, w_turn)
                v_ref = v_nom * (1.0 - 0.5 * strength)
        for i in list(self.side):
            if i not in near:
                del self.side[i]
        if not cons:
            return v_nom, steer_nom, clearance, 'CLEAR'
        safe = all(A * v_nom + B * w_nom + C >= 0.0 for A, B, C in cons)
        if worst <= 0.0 and safe:
            return v_nom, steer_nom, clearance, 'CLEAR'
        u = self._project(v_ref, w_ref, v_nom, cons)
        if u is None or u[0] < 1e-3:
            return 0.0, steer_nom, clearance, 'CBF_STOP'
        steer = math.atan(self.L * u[1] / u[0])
        return u[0], max(-self.max_steer, min(self.max_steer, steer)), clearance, 'CBF_ACTIVE'

    # =========================================================================
    # [CORE]   _project(v_ref, w_ref, v_max, cons) -> (v, omega) or None
    # -------------------------------------------------------------------------
    # Closest point (cost (v-v_ref)^2 + steer_weight (omega-w_ref)^2) in the
    # triangle (0,0), (v_max, +-kappa_max v_max) clipped by every half-plane.
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
    # [UTIL]   _clip / _closest_on_segment / _inside
    # -------------------------------------------------------------------------
    # Convex polygon clipped by a x + b y + c >= 0 (Sutherland-Hodgman); closest
    # point on a segment; point-in-convex-polygon test.
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

    @staticmethod
    def _closest_on_segment(p, s0, s1):
        ex, ey = s1[0] - s0[0], s1[1] - s0[1]
        l2 = ex * ex + ey * ey
        t = 0.0 if l2 == 0.0 else max(0.0, min(1.0, ((p[0] - s0[0]) * ex + (p[1] - s0[1]) * ey) / l2))
        return s0[0] + t * ex, s0[1] + t * ey

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
# What a controller node uses. Declares robot_radius, safety_margin, cbf_alpha,
# cbf_max_steer (0 = use max_steer) and obstacles (flat list; default none),
# subscribes to the latched /hardware_obstacles (flat [x, y, r, ...]) and merges
# both. apply() takes the pose in the controller frame plus that frame's origin,
# so obstacles given in the lab frame still line up when odometry is re-zeroed.
# =============================================================================
class ObstacleCbfLayer:
    def __init__(self, node, wheelbase, max_steer, ref_offset):
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import Float32MultiArray
        node.declare_parameter('robot_radius', 0.20)
        node.declare_parameter('safety_margin', 0.10)
        node.declare_parameter('cbf_alpha', 0.6)
        node.declare_parameter('cbf_max_steer', 0.0)
        g = lambda n: float(node.get_parameter(n).value)  # noqa: E731
        limit = g('cbf_max_steer') if g('cbf_max_steer') > 0.0 else max_steer
        self.cbf = CbfFilter(wheelbase, limit, ref_offset, g('robot_radius'), g('safety_margin'), g('cbf_alpha'))
        self.param_obstacles = parse_obstacles(declare_flexible_parameter(node, 'obstacles'))
        self.topic_obstacles = []
        self.node = node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Float32MultiArray, '/hardware_obstacles', self._cb, latched)
        if self.param_obstacles:
            node.get_logger().info(f'CBF: {len(self.param_obstacles) // 3} obstacle(s) from parameter')

    def _cb(self, msg):
        data = [float(v) for v in msg.data]
        if len(data) // 3 != len(self.topic_obstacles) // 3:
            self.node.get_logger().info(f'CBF: {len(data) // 3} obstacle(s) from /hardware_obstacles')
        self.topic_obstacles = data

    # =========================================================================
    # [CORE]   apply(v, steer, x, y, yaw, x0, y0, yaw0)
    #          -> (v_safe, steer_safe, clearance, status)
    # -------------------------------------------------------------------------
    # Moves the lab-frame obstacles into the controller frame (origin x0, y0,
    # yaw0) and runs the filter.
    # =========================================================================
    def apply(self, v, steer, x, y, yaw, x0=0.0, y0=0.0, yaw0=0.0):
        flat = self.param_obstacles + self.topic_obstacles
        c, s = math.cos(yaw0), math.sin(yaw0)
        obs = []
        for i in range(0, len(flat) - 2, 3):
            dx, dy = flat[i] - x0, flat[i + 1] - y0
            obs.append((c * dx + s * dy, -s * dx + c * dy, flat[i + 2]))
        return self.cbf.filter(v, steer, x, y, yaw, obs)
