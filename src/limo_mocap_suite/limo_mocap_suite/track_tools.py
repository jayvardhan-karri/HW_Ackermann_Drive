# #############################################################################
# [FILE]   track_tools.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/track_tools.py
# ROLE  : The capsule (running track) reference paths, in one place. Plain math
#         plus the Path message; used by track_publisher and run_laps.
#
# The lab track (default everywhere): R 1.0 m, 4 m straights, centre (3.25, 1.25).
# It spans x 0.25..6.25, y 0.25..2.25 and so keeps 0.75 m to every edge of the
# mocap area (LAB_X, LAB_Y), room to drift or go round an obstacle.
# Race start: the middle of the bottom straight (start_side 'bottom') or of the
# top one ('top'); counter-clockwise ('ccw', left turns) or clockwise ('cw').
# start_side 'auto' (default in the suites): the top straight, unless the robot
# stands closer to the bottom one (below the track centre line): see pick_side().
# start_at 'begin' moves it back to where that straight begins (end of the curve);
# the obstacle add-on uses it so the robot has the whole straight before an obstacle.
#
# Radii for the extra radius study (steering limit: about 0.46 m radius):
#   1.0 m  well inside the limit: every controller should work
#   0.5 m  at the limit
#   0.4 m  beyond the limit: the robot must drift; shows which recovers best
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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

TRACK_RADII = (1.0, 0.5, 0.4)     # radii of the radius study, m
LAB_X = (-0.5, 7.0)          # m, mocap area the robot must stay in
LAB_Y = (-0.5, 3.0)
TRACK_RADIUS = 1.0
TRACK_STRAIGHT = 4.0
TRACK_X = 3.25               # track centre
TRACK_Y = 1.25
START_SIDES = ('bottom', 'top')
DIRECTIONS = ('ccw', 'cw')
START_AT = ('middle', 'begin')


# =============================================================================
# [UTIL]   pick_side(start_side, robot_y, cy) -> 'bottom' or 'top'
# -----------------------------------------------------------------------------
# 'auto': top, unless the robot is below the track centre line (closer to the
# bottom straight). 'top' / 'bottom' are kept as given.
# robot_y, cy: robot y and track centre y in m (robot_y None: no odometry yet, so
# 'top'). Used by run_laps and track_publisher, so both pick the same side.
# =============================================================================
def pick_side(start_side, robot_y, cy):
    if start_side != 'auto':
        return start_side
    return 'bottom' if robot_y is not None and robot_y < cy else 'top'


# =============================================================================
# [UTIL]   capsule(straight, radius, cx, cy, ds=0.01, start_side, direction, start_at) -> (x, y)
# -----------------------------------------------------------------------------
# Running-track shape. Point 0 is the race start: the middle (or the beginning) of
# the bottom or top straight; the points follow the driving direction. With the
# defaults it is the same track as matlab/lap_study/capsule_path.m (ccw from mid
# bottom, heading +X). Lengths in m; ds = point spacing. Returns x and y arrays.
# HOW: build the ccw loop starting at the bottom middle (_capsule_ccw); for cw
# reverse the point order; for 'top' or 'begin' rotate the array (np.roll) so
# point 0 is the wanted point. Raises ValueError for an unknown option.
# =============================================================================
def capsule(straight, radius, cx, cy, ds=0.01, start_side='bottom', direction='ccw', start_at='middle'):
    if start_side not in START_SIDES or direction not in DIRECTIONS or start_at not in START_AT:
        raise ValueError(f'start_side must be one of {START_SIDES}, direction one of {DIRECTIONS}, '
                         f'start_at one of {START_AT}')
    x, y = _capsule_ccw(straight, radius, cx, cy, ds)
    if direction == 'cw':                       # same points, driven the other way
        x, y = np.roll(x[::-1], 1), np.roll(y[::-1], 1)
    if start_side == 'top':                     # top middle is half a lap away
        top = int(np.argmin(np.hypot(x - cx, y - (cy + radius))))
        x, y = np.roll(x, -top), np.roll(y, -top)
    if start_at == 'begin':                     # back along the straight by half its length
        hx, hy = x[1] - x[0], y[1] - y[0]
        n = math.hypot(hx, hy)
        bx, by = x[0] - straight / 2.0 * hx / n, y[0] - straight / 2.0 * hy / n
        k = int(np.argmin(np.hypot(x - bx, y - by)))
        x, y = np.roll(x, -k), np.roll(y, -k)
    return x, y


# =============================================================================
# [CORE]   _capsule_ccw(straight, radius, cx, cy, ds) -> (x, y)  (helper of capsule)
# -----------------------------------------------------------------------------
# The capsule counter-clockwise, point 0 at the middle of the bottom straight.
# HOW: five pieces joined in order: half the bottom straight, the right half
# circle (angles -90..+90 deg), the top straight (right to left), the left half
# circle, then the other half of the bottom straight. Straights are +-straight/2
# from the centre; circles have the given radius.
# =============================================================================
def _capsule_ccw(straight, radius, cx, cy, ds):
    h = straight / 2.0
    n = lambda length: max(2, int(round(length / ds)))  # noqa: E731
    a1 = np.linspace(-math.pi / 2.0, math.pi / 2.0, n(math.pi * radius), endpoint=False)
    a2 = a1 + math.pi
    half = np.arange(0.0, h, ds)
    back = np.arange(-h, -ds / 2.0, ds)
    x = np.concatenate((half, h + radius * np.cos(a1), np.linspace(h, -h, n(straight), endpoint=False),
                        -h + radius * np.cos(a2), back)) + cx
    y = np.concatenate((np.full(len(half), -radius), radius * np.sin(a1), np.full(n(straight), radius),
                        radius * np.sin(a2), np.full(len(back), -radius))) + cy
    return x, y


# =============================================================================
# [ROS-IO] capsule_message(straight, radius, cx, cy, stamp, lab_x, lab_y,
#                          start_side, direction, start_at) -> Path
# -----------------------------------------------------------------------------
# The track as a Path in frame 'odom' (position only, no heading: the controller
# works it out). Raises ValueError if it does not fit the lab (lab_x, lab_y in m).
# HOW: capsule() at 1 cm spacing, check the bounding box against the lab, copy
# each point into a PoseStamped.
# =============================================================================
def capsule_message(straight, radius, cx, cy, stamp, lab_x, lab_y, start_side='bottom', direction='ccw',
                    start_at='middle'):
    x, y = capsule(straight, radius, cx, cy, start_side=start_side, direction=direction, start_at=start_at)
    if x.min() < lab_x[0] or x.max() > lab_x[1] or y.min() < lab_y[0] or y.max() > lab_y[1]:
        raise ValueError(f'Track does not fit the lab: x {x.min():.2f}..{x.max():.2f}, '
                         f'y {y.min():.2f}..{y.max():.2f}, lab {list(lab_x)} {list(lab_y)}')
    msg = Path()
    msg.header.frame_id = 'odom'
    msg.header.stamp = stamp
    for px, py in zip(x, y):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x, ps.pose.position.y = float(px), float(py)
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg
