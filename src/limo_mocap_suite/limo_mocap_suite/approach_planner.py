# #############################################################################
# [FILE]   approach_planner.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/approach_planner.py
# ROLE  : How the robot gets onto a path it is not on. An Ackermann robot cannot
#         turn on the spot or (here) reverse, so the shortest way from one pose to
#         another is a Dubins path: arcs of a fixed radius joined by a straight.
#         Used to reach the path start and to rejoin after drifting away.
#
# To the start of a CLOSED path (plan_to_start): one fixed rule, like a car joining
# a road. No scores or weights: the same situation always gives the same plan.
#   1. Lane: the closest point of the path that is IN FRONT of the robot and runs
#      within 90 deg of its heading (the part of the loop going the robot's way).
#   2. Merge MERGE_AHEAD (1 m) further along, with the shortest curve there.
#   3. Follow the path to the start. If the start comes before the merge point,
#      merge onto the start itself.
#   4. The curve may not loop (more than MAX_TURN, 180 deg), leave the allowed area
#      or pass close to an obstacle; if it would, the merge point moves MERGE_STEP
#      further along, up to MERGE_MAX.
#   5. No such merge (robot sideways near an end of the loop): the shortest curve
#      straight to the start, loops allowed. Nothing: None (robot not moved).
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
#   _name    a leading underscore = internal helper, only used inside this file/class
# #############################################################################

import math

import numpy as np

TWO_PI = 2.0 * math.pi      # one full turn, rad
MERGE_AHEAD = 1.0       # m past the closest point of the lane
MERGE_STEP = 0.5        # m further along if the curve is not allowed
MERGE_MAX = 6.0         # m: furthest merge point tried
MAX_TURN = math.pi      # rad: a merge curve turning more than this is a loop
CHECK_DS = 0.05         # m: candidates are checked on this coarser sampling


# =============================================================================
# [UTIL]   _mod2pi(a) -> angle in [0, 2*pi)  (helper: used only in this file)
# -----------------------------------------------------------------------------
# Wraps an angle in rad. Used by the Dubins formulas, which want positive arc angles.
# =============================================================================
def _mod2pi(a):
    return a % TWO_PI


# =============================================================================
# [CORE]   _dubins_words(d, alpha, beta) -> list of (word, (t, p, q))  (helper of plan, _best)
# -----------------------------------------------------------------------------
# The six Dubins candidates in normalised form: radius 1, goal at distance d on
# +x, start heading alpha, goal heading beta (d in radii, angles in rad).
# L / R = arc left / right, S = straight. A word is left out when it has no
# solution. t, p, q = length of each of the three parts, in radii (arcs: rad).
# HOW: the closed-form Dubins solution; each word has its own formula for the
# middle part p and the two end parts t and q.
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
# [CORE]   _sample(start, word, lengths, radius, ds) -> N x 4 [x, y, yaw, kappa]  (helper of plan, _best)
# -----------------------------------------------------------------------------
# Turns a Dubins word into points. start = (x, y, yaw) in m / rad, radius and ds in m.
# Walks the word part by part, one point every ds metres (a little less so a part
# ends exactly). Arcs use the exact circle formula, so no drift builds up.
# kappa = +1/radius for L, -1/radius for R, 0 for S (1/m).
# =============================================================================
def _sample(start, word, lengths, radius, ds):
    x, y, yaw = start
    rows = []
    for letter, seg in zip(word, lengths):
        kappa = {'L': 1.0 / radius, 'R': -1.0 / radius, 'S': 0.0}[letter]
        length = seg * radius
        n = int(math.ceil(length / ds))
        if n == 0:
            continue                # zero-length part of the word
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
# Shortest Dubins path from start to goal, poses (x, y, yaw) in m / rad. radius =
# smallest turn radius in m, ds = point spacing in m, length in m.
# table is N x 4 [x, y, yaw, kappa]. allowed(table) -> bool rejects a candidate
# (e.g. it leaves the lab or hits an obstacle), so the next shortest is tried.
# None: none allowed.
# HOW: rotate so the goal lies on +x, scale by the radius, get the Dubins words,
# sort by total length, sample each in turn and return the first one allowed.
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
# pose = (x, y, yaw), waypoints = N x 5 table, closed = wrap around at the end,
# join_idx = index in waypoints where the robot joins (the table ends there).
# HOW: for each candidate point (0.25 m steps) call plan(); keep the shortest.
# On an open path, candidates past the last point are dropped.
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


# =============================================================================
# [CORE]   plan_to_start(pose, waypoints, radius, allowed=None)
#              -> (table, join_idx, text) or None
# -----------------------------------------------------------------------------
# Closed path only; the fixed rule in the file header. table: N x 4 [x, y, yaw,
# kappa] of the curve only; the caller adds waypoints[join_idx:] after it.
# text: the plan, for the log. None: no allowed curve (robot is not moved).
# HOW (details in the file header): lane point k0 -> merge point 1 m further
# (0.5 m steps up to 6 m) with _best() under no_loop() -> else the shortest
# curve straight to the start.
# =============================================================================
def plan_to_start(pose, waypoints, radius, allowed=None):
    n = len(waypoints)
    seg = np.hypot(np.diff(waypoints[:, 0]), np.diff(waypoints[:, 1]))
    spacing = max(float(np.median(seg)), 1e-3)
    dx, dy = waypoints[:, 0] - pose[0], waypoints[:, 1] - pose[1]
    same_way = np.cos(waypoints[:, 2] - pose[2]) >= -1e-6               # runs within 90 deg of the heading
    in_front = dx * math.cos(pose[2]) + dy * math.sin(pose[2]) > 0.0    # ahead of the robot
    lane = same_way & in_front if (same_way & in_front).any() else same_way
    k0 = int(np.argmin(np.where(lane, np.hypot(dx, dy), np.inf)))      # closest point of the lane
    to_start = (n - k0) % n                                             # points from there to the start

    # ---- no_loop(table) -> bool  (helper of plan_to_start) -------------------
    # True if the curve turns at most MAX_TURN (180 deg) in total and allowed()
    # accepts it. The turn is the summed change of yaw along the table.
    def no_loop(table):
        turn = np.abs(np.diff(np.unwrap(table[:, 2]))).sum()
        return turn <= MAX_TURN + 0.05 and (allowed is None or allowed(table))

    # ---- result(j, got, note) -> (table, j, text)  (helper of plan_to_start) --
    # Builds the return value for a merge at waypoint j: the curve resampled at
    # 1 cm and the log text (curve length, distance along the path to the start).
    def result(j, got, note=''):
        word, lengths = got
        length = sum(lengths) * radius
        table = _sample(pose, word, lengths, radius, 0.01)                  # the checked curve, 1 cm steps
        gx, gy = waypoints[j, :2]
        along = ((n - j) % n) * spacing
        where = 'the start' if j == 0 else f'({gx:.2f}, {gy:.2f})'
        return table, j, (f'{note}merge onto the path at {where} by a {length:.2f} m curve, '
                          f'then {along:.2f} m along the path to the start')

    # merge point = lane point + `ahead` m; farther each pass until a curve is allowed
    ahead = MERGE_AHEAD
    while ahead <= MERGE_MAX + 1e-9:                                    # steps 1-4
        steps = int(round(ahead / spacing))
        j = 0 if to_start <= steps else (k0 + steps) % n                # start comes first: merge onto it
        got = _best(pose, tuple(waypoints[j, :3]), radius, no_loop)
        if got is not None:
            return result(j, got)
        if j == 0:
            break                                                       # nothing further before the start
        ahead += MERGE_STEP
    got = _best(pose, tuple(waypoints[0, :3]), radius, allowed)                 # step 5
    if got is not None:
        return result(0, got, 'no smooth merge: ')
    return None


# =============================================================================
# [CORE]   _best(start, goal, radius, allowed) -> (word, lengths) or None  (helper of plan_to_start)
# -----------------------------------------------------------------------------
# The shortest Dubins word from start to goal that allowed() accepts.
# Same search as plan(), but checks each candidate on the coarser CHECK_DS
# sampling (faster, since plan_to_start tries many merge points) and returns
# the word, not the points. None: no word is allowed.
# =============================================================================
def _best(start, goal, radius, allowed):
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    d = math.hypot(dx, dy) / radius
    theta = math.atan2(dy, dx) if d > 1e-9 else 0.0
    words = sorted(_dubins_words(d, _mod2pi(start[2] - theta), _mod2pi(goal[2] - theta)), key=lambda w: sum(w[1]))
    for word, lengths in words:
        if allowed is None or allowed(_sample(start, word, lengths, radius, CHECK_DS)):
            return word, lengths
    return None
