# #############################################################################
# [FILE]   approach_planner.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/approach_planner.py
# ROLE  : How the robot gets onto a path it is not on. An Ackermann robot cannot
#         turn on the spot or (here) reverse, so the shortest way from one pose to
#         another is a Dubins path: arcs of a fixed radius joined by a straight.
#         Used to reach the path start and to rejoin after drifting away.
#
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

TWO_PI = 2.0 * math.pi


def _mod2pi(a):
    return a % TWO_PI


# =============================================================================
# [CORE]   _dubins_words(d, alpha, beta) -> list of (word, (t, p, q))
# -----------------------------------------------------------------------------
# The six Dubins candidates in normalised form (radius 1, goal at distance d on
# +x, start heading alpha, goal heading beta). L / R = arc left / right, S =
# straight. A word is left out when it has no solution.
# =============================================================================
def _dubins_words(d, alpha, beta):
    sa, ca, sb, cb = math.sin(alpha), math.cos(alpha), math.sin(beta), math.cos(beta)
    cab = math.cos(alpha - beta)
    out = []
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb)
    if p2 >= 0.0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        out.append(('LSL', (_mod2pi(tmp - alpha), math.sqrt(p2), _mod2pi(beta - tmp))))
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sb - sa)
    if p2 >= 0.0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        out.append(('RSR', (_mod2pi(alpha - tmp), math.sqrt(p2), _mod2pi(tmp - beta))))
    p2 = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb)
    if p2 >= 0.0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out.append(('LSR', (_mod2pi(tmp - alpha), p, _mod2pi(tmp - beta))))
    p2 = -2.0 + d * d + 2.0 * cab - 2.0 * d * (sa + sb)
    if p2 >= 0.0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out.append(('RSL', (_mod2pi(alpha - tmp), p, _mod2pi(beta - tmp))))
    c = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0
    if abs(c) <= 1.0:
        p = _mod2pi(TWO_PI - math.acos(c))
        t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        out.append(('RLR', (t, p, _mod2pi(alpha - beta - t + p))))
    c = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sb - sa)) / 8.0
    if abs(c) <= 1.0:
        p = _mod2pi(TWO_PI - math.acos(c))
        t = _mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
        out.append(('LRL', (t, p, _mod2pi(beta - alpha - t + p))))
    return out


# =============================================================================
# [CORE]   _sample(start, word, lengths, radius, ds) -> N x 4 [x, y, yaw, kappa]
# -----------------------------------------------------------------------------
# Walks a Dubins word from the start pose every ds metres, in exact arcs.
# =============================================================================
def _sample(start, word, lengths, radius, ds):
    x, y, yaw = start
    rows = []
    for letter, seg in zip(word, lengths):
        kappa = {'L': 1.0 / radius, 'R': -1.0 / radius, 'S': 0.0}[letter]
        length = seg * radius
        n = int(math.ceil(length / ds))
        if n == 0:
            continue
        step = length / n
        for _ in range(n):
            rows.append((x, y, yaw, kappa))
            if kappa == 0.0:
                x += step * math.cos(yaw)
                y += step * math.sin(yaw)
            else:
                nyaw = yaw + kappa * step
                x += (math.sin(nyaw) - math.sin(yaw)) / kappa
                y += (math.cos(yaw) - math.cos(nyaw)) / kappa
                yaw = nyaw
    rows.append((x, y, yaw, 0.0))
    return np.array(rows)


# =============================================================================
# [CORE]   plan(start, goal, radius, ds=0.01, allowed=None) -> (table, length) or None
# -----------------------------------------------------------------------------
# Shortest Dubins path from start to goal, poses (x, y, yaw). table is N x 4
# [x, y, yaw, kappa]. allowed(table) -> bool rejects a candidate (e.g. it leaves
# the lab or hits an obstacle), so the next shortest is tried. None: none allowed.
# =============================================================================
def plan(start, goal, radius, ds=0.01, allowed=None):
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    d = math.hypot(dx, dy) / radius
    theta = math.atan2(dy, dx) if d > 1e-9 else 0.0
    alpha, beta = _mod2pi(start[2] - theta), _mod2pi(goal[2] - theta)
    words = sorted(_dubins_words(d, alpha, beta), key=lambda w: sum(w[1]))
    for word, lengths in words:
        table = _sample(start, word, lengths, radius, ds)
        if allowed is None or allowed(table):
            return table, sum(lengths) * radius
    return None


# =============================================================================
# [CORE]   plan_rejoin(pose, waypoints, idx, closed, radius, allowed=None)
#              -> (table, join_idx) or None
# -----------------------------------------------------------------------------
# After drifting away: tries joining the path at points 0.5 to 4 m ahead of index
# idx and keeps the shortest way there. Never goes backwards along the path.
# =============================================================================
def plan_rejoin(pose, waypoints, idx, closed, radius, allowed=None):
    n = len(waypoints)
    spacing = max(float(np.median(np.hypot(np.diff(waypoints[:, 0]), np.diff(waypoints[:, 1])))), 1e-3)
    best = None
    for ahead in np.arange(0.5, 4.01, 0.25):
        j = idx + int(ahead / spacing)
        if closed:
            j %= n
        elif j >= n:
            break
        got = plan(pose, tuple(waypoints[j, :3]), radius, allowed=allowed)
        if got is not None and (best is None or got[1] < best[1]):
            best = (got[0], got[1], j)
    return None if best is None else (best[0], best[2])
