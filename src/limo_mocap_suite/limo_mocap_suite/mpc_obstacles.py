# #############################################################################
# [FILE]   mpc_obstacles.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/mpc_obstacles.py
# ROLE  : Obstacles as constraints inside the MPC, instead of a CBF filter after it.
#         Used by the LTV-MPC and NMPC nodes with obstacle_mode:=constraint.
#         The nodes' normal solvers are untouched.
#
# Both solvers look 3 s ahead (the normal horizons, under 1 s, end before the robot could
# turn away from anything). Speed stays outside the optimiser, so only steering can
# avoid; if no safe plan exists the robot slows, and stops if it is about to touch.
#   LinearObstacleMpc    linear model about the path, so an obstacle must be turned into
#                        a bound on the sideways offset: one side has to be chosen first.
#                        Convex: one small QP.
#   NonlinearObstacleMpc full bicycle model and the true distance to each cylinder.
#                        Not convex, so it needs a starting guess to one side.
# Not a node; plain math, no ROS.
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
from scipy.optimize import minimize

N = 20            # steps
DT = 0.15         # s per step
ACTIVE_RANGE = 2.0   # obstacles further than this from the robot are ignored (m)
MODEL_MARGIN = 0.10  # linear model's own error while turning hard: extra keep-out in the QP only (m)


# =============================================================================
# [UTIL]   world_obstacles(safety, x0, y0, yaw0) -> list of (x, y, r)
# -----------------------------------------------------------------------------
# The same obstacles the CBF layer holds, in the controller frame.
# =============================================================================
def world_obstacles(safety, x0=0.0, y0=0.0, yaw0=0.0):
    flat = safety.param_obstacles + safety.topic_obstacles
    c, s = math.cos(yaw0), math.sin(yaw0)
    out = []
    for i in range(0, len(flat) - 2, 3):
        dx, dy = flat[i] - x0, flat[i + 1] - y0
        out.append((c * dx + s * dy, -s * dx + c * dy, flat[i + 2]))
    return out


# =============================================================================
# [UTIL]   sample_reference(traj, idx) -> N x 5 table
# -----------------------------------------------------------------------------
# Reference points one step apart at the reference speed, starting after idx.
# =============================================================================
def sample_reference(traj, idx):
    rows = np.zeros((N, 5))
    cur = float(idx)
    for k in range(N):
        i = traj.index(round(cur))
        pt, nxt = traj.waypoints[i], traj.waypoints[traj.index(i + 1)]
        ds = max(math.hypot(nxt[0] - pt[0], nxt[1] - pt[1]), 1e-3)
        cur += pt[3] * DT / ds
        rows[k] = traj.waypoints[traj.index(round(cur))]
    return rows


# =============================================================================
# [UTIL]   clearance(x, y, obstacles) -> m
# -----------------------------------------------------------------------------
# Smallest distance from the point to a cylinder edge.
# =============================================================================
def clearance(x, y, obstacles):
    if not obstacles:
        return float('inf')
    return min(math.hypot(x - ox, y - oy) - r for ox, oy, r in obstacles)


# =============================================================================
# [CORE]   class LinearObstacleMpc
# -----------------------------------------------------------------------------
# Model about the path, with e = sideways offset (left +), th = heading offset:
#     e[k+1]  = e[k] + v DT th[k]
#     th[k+1] = th[k] + v DT sec^2(d_ff)/L u[k],  u = steering minus feed-forward
# So e and th are linear in u, and the cost is quadratic. Each obstacle becomes a bound
# on e at the steps near it: e >= l + w when passing on its left, e <= l - w on its right,
# with l its offset and w = sqrt(R^2 - ds^2). The side is chosen once per obstacle: away
# from its centre, the shorter way. A slack keeps the problem solvable and shows when no
# safe plan exists.
# =============================================================================
class LinearObstacleMpc:
    def __init__(self, wheelbase, max_steer, keep_out, q_e=30.0, q_th=4.0, r=0.5, rd=6.0):
        self.L, self.max_steer, self.keep_out = wheelbase, max_steer, keep_out
        self.q_e, self.q_th, self.r, self.rd = q_e, q_th, r, rd
        self.sides = {}
        self.u = np.zeros(N)

    def reset(self):
        self.sides = {}
        self.u = np.zeros(N)

    def _side(self, key, offset):
        if key not in self.sides:
            self.sides[key] = 1.0 if offset <= 0.0 else -1.0     # pass left if the centre is right of the path
        return self.sides[key]

    # =========================================================================
    # [CORE]   _bounds(ref, obstacles) -> (lower, upper)
    # -------------------------------------------------------------------------
    # Sideways limits per step from the obstacles near each reference point.
    # =========================================================================
    def _bounds(self, ref, obstacles):
        lower, upper = np.full(N, -np.inf), np.full(N, np.inf)
        for j, (ox, oy, r) in enumerate(obstacles):
            R = r + self.keep_out + MODEL_MARGIN
            side = None
            for k in range(N):
                t = np.array([math.cos(ref[k, 2]), math.sin(ref[k, 2])])
                d = np.array([ox - ref[k, 0], oy - ref[k, 1]])
                ds = t.dot(d)
                if abs(ds) >= R:
                    continue
                offset = -t[1] * d[0] + t[0] * d[1]
                if abs(offset) >= R + 0.25:      # beside this point of the path, but too far away to matter
                    continue
                side = self._side(j, offset) if side is None else side
                w = math.sqrt(R * R - ds * ds)
                if side > 0.0:
                    lower[k] = max(lower[k], offset + w)
                else:
                    upper[k] = min(upper[k], offset - w)
        return lower, upper

    # =========================================================================
    # [CORE]   solve(cte, herr, ref, prev_steer, obstacles) -> (steer, slack_m)
    # -------------------------------------------------------------------------
    # Builds e = e_free + G u and th = th_free + H u, then one small QP for u and the
    # slack, by SLSQP with exact gradients.
    # =========================================================================
    def solve(self, cte, herr, ref, prev_steer, obstacles):
        e0, th0 = -cte, -herr
        v = np.maximum(ref[:, 3], 0.05)
        ff = np.arctan(self.L * ref[:, 4])
        b = v * DT * (1.0 + np.tan(ff) ** 2) / self.L
        G, H = np.zeros((N, N)), np.zeros((N, N))
        e_free, th_free = np.zeros(N), np.zeros(N)
        e_k, th_k = e0, th0
        row_e, row_th = np.zeros(N), np.zeros(N)
        for k in range(N):
            e_k = e_k + v[k] * DT * th_k
            row_e = row_e + v[k] * DT * row_th
            row_th = row_th.copy()
            row_th[k] = b[k]
            G[k], H[k] = row_e, row_th
            e_free[k], th_free[k] = e_k, th_k
        lower, upper = self._bounds(ref, obstacles)
        low = np.flatnonzero(np.isfinite(lower))
        up = np.flatnonzero(np.isfinite(upper))
        A = np.eye(N) - np.eye(N, k=-1)                 # steering changes
        c = np.zeros(N)
        c[0] = -(prev_steer - ff[0])
        P = 2.0 * (self.q_e * G.T @ G + self.q_th * H.T @ H + self.r * np.eye(N) + self.rd * A.T @ A)
        q = 2.0 * (self.q_e * G.T @ e_free + self.q_th * H.T @ th_free + self.rd * A.T @ c)
        slack_w = 2.0e5

        def cost(z):
            u, s = z[:N], z[N]
            return 0.5 * u.dot(P).dot(u) + q.dot(u) + slack_w * s * s

        def grad(z):
            return np.append(P.dot(z[:N]) + q, 2.0 * slack_w * z[N])

        cons = []
        if len(low):
            J = np.hstack((G[low], np.ones((len(low), 1))))
            f = e_free[low] - lower[low]
            cons.append({'type': 'ineq', 'fun': lambda z, J=J, f=f: J.dot(z) + f, 'jac': lambda z, J=J: J})
        if len(up):
            J2 = np.hstack((-G[up], np.ones((len(up), 1))))
            f2 = upper[up] - e_free[up]
            cons.append({'type': 'ineq', 'fun': lambda z, J=J2, f=f2: J.dot(z) + f, 'jac': lambda z, J=J2: J})
        bounds = [(-self.max_steer - f, self.max_steer - f) for f in ff] + [(0.0, None)]
        start = np.append(np.append(self.u[1:], self.u[-1]), 0.0)
        res = minimize(cost, start, jac=grad, method='SLSQP', bounds=bounds, constraints=cons,
                       options={'maxiter': 30, 'ftol': 1e-5})
        self.u = res.x[:N]
        return float(np.clip(ff[0] + self.u[0], -self.max_steer, self.max_steer)), float(res.x[N])


# =============================================================================
# [CORE]   class NonlinearObstacleMpc
# -----------------------------------------------------------------------------
# Rolls the bicycle model forward and constrains the true distance from the tracked
# point to each nearby cylinder at every step. When an obstacle first comes into range
# the starting guess is an S to the side away from its centre, because the solver only
# finds the nearest local solution.
# =============================================================================
class NonlinearObstacleMpc:
    def __init__(self, wheelbase, max_steer, ref_offset, keep_out, q_cte=40.0, q_heading=8.0, rd=2.5):
        self.L, self.max_steer, self.a, self.keep_out = wheelbase, max_steer, ref_offset, keep_out
        self.q_cte, self.q_heading, self.rd = q_cte, q_heading, rd
        self.u = np.zeros(N)
        self.seen = set()

    def reset(self):
        self.u = np.zeros(N)
        self.seen = set()

    def _rollout(self, u, x0, y0, yaw0, ref):
        lo, L = self.a, self.L
        yaw = yaw0
        x, y = x0 - lo * math.cos(yaw0), y0 - lo * math.sin(yaw0)
        pts = np.zeros((N, 3))
        for k in range(N):
            v = ref[k, 3]
            x += v * math.cos(yaw) * DT
            y += v * math.sin(yaw) * DT
            yaw += v / L * math.tan(u[k]) * DT
            pts[k] = x + lo * math.cos(yaw), y + lo * math.sin(yaw), yaw
        return pts

    def _guess(self, x, y, yaw, ref, obstacles):
        for j, (ox, oy, _r) in enumerate(obstacles):
            if j in self.seen or math.hypot(ox - x, oy - y) > ACTIVE_RANGE:
                continue
            self.seen.add(j)
            k = int(np.argmin(np.hypot(ref[:, 0] - ox, ref[:, 1] - oy)))
            t = (math.cos(ref[k, 2]), math.sin(ref[k, 2]))
            offset = -t[1] * (ox - ref[k, 0]) + t[0] * (oy - ref[k, 1])
            side = 1.0 if offset <= 0.0 else -1.0
            bend = 0.8 * self.max_steer
            turn = max(2, min(k // 2, N // 3))
            self.u[:turn] = side * bend
            self.u[turn:2 * turn] = -side * bend

    # =========================================================================
    # [CORE]   solve(x, y, yaw, ref, prev_steer, obstacles) -> (steer, violation_m)
    # -------------------------------------------------------------------------
    # SLSQP with the normal NMPC cost, plus distance >= keep-out as inequalities.
    # violation: how far the plan's closest approach falls short.
    # =========================================================================
    def solve(self, x, y, yaw, ref, prev_steer, obstacles):
        near = [o for o in obstacles if math.hypot(o[0] - x, o[1] - y) < ACTIVE_RANGE]
        self._guess(x, y, yaw, ref, obstacles)
        lo = self.a
        th_des = ref[:, 2] - np.arctan(lo * ref[:, 4])

        def cost(u):
            p = self._rollout(u, x, y, yaw, ref)
            dx, dy = ref[:, 0] - p[:, 0], ref[:, 1] - p[:, 1]
            cte = -np.sin(ref[:, 2]) * dx + np.cos(ref[:, 2]) * dy
            th = np.arctan2(np.sin(th_des - p[:, 2]), np.cos(th_des - p[:, 2]))
            slew = np.diff(np.concatenate(([prev_steer], u)))
            return self.q_cte * cte.dot(cte) + self.q_heading * th.dot(th) + self.rd * slew.dot(slew)

        def gaps(u):
            p = self._rollout(u, x, y, yaw, ref)
            return np.concatenate([np.hypot(p[:, 0] - ox, p[:, 1] - oy) - r - self.keep_out for ox, oy, r in near])

        cons = [{'type': 'ineq', 'fun': gaps}] if near else []
        start = np.clip(np.append(self.u[1:], self.u[-1]), -self.max_steer, self.max_steer)
        res = minimize(cost, start, method='SLSQP', bounds=[(-self.max_steer, self.max_steer)] * N,
                       constraints=cons, options={'maxiter': 6, 'ftol': 1e-3})
        self.u = res.x
        short = max(0.0, -float(np.min(gaps(res.x)))) if near else 0.0
        return float(res.x[0]), short


# =============================================================================
# [CORE]   speed_for_plan(v, shortfall, clear, keep_out) -> (v, status)
# -----------------------------------------------------------------------------
# The plan is safe: unchanged. It falls short by a little: slower, to give the steering
# more distance. About to touch with no safe plan: stop.
# =============================================================================
def speed_for_plan(v, shortfall, clear, keep_out):
    if shortfall <= 0.01:
        return v, ('MPC_ACTIVE' if clear < keep_out + 0.6 else 'MPC_CLEAR')
    if clear < 0.08 + 0.5 * (keep_out - 0.3):
        return 0.0, 'MPC_STOP'
    return v * max(0.3, 1.0 - shortfall / 0.2), 'MPC_ACTIVE'
