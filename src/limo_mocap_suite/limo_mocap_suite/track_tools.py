# #############################################################################
# [FILE]   track_tools.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/track_tools.py
# ROLE  : The capsule (running track) reference paths, in one place. Plain math
#         plus the Path message; used by track_publisher and lap_test.
#
# Default radii for comparing controllers (steering limit: about 0.46 m radius):
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
# #############################################################################

import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

TRACK_RADII = (1.0, 0.5, 0.4)


# =============================================================================
# [UTIL]   capsule(straight, radius, cx, cy, ds=0.01) -> (x, y)
# -----------------------------------------------------------------------------
# Running-track shape, counter-clockwise, starting mid lower straight heading +X.
# Same track as matlab/lap_study/capsule_path.m.
# =============================================================================
def capsule(straight, radius, cx, cy, ds=0.01):
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
# [ROS-IO] capsule_message(straight, radius, cx, cy, stamp, lab_x, lab_y) -> Path
# -----------------------------------------------------------------------------
# The track as a Path. Raises ValueError if it does not fit the lab.
# =============================================================================
def capsule_message(straight, radius, cx, cy, stamp, lab_x, lab_y):
    x, y = capsule(straight, radius, cx, cy)
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
