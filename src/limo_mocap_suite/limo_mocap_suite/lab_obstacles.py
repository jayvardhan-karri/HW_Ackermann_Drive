# #############################################################################
# [FILE]   lab_obstacles.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/lab_obstacles.py
# ROLE  : Add-on only. The two fixed obstacles of the obstacle lap test, worked
#         out from the lab track. Plain math, no ROS. Used by obstacle_lap.launch.py;
#         nothing else in the package depends on it.
#
# Layout (default track, counter-clockwise):
#   two cylinders, 0.60 m diameter, one on each straight, OBSTACLE_AHEAD (1 m)
#   past the middle in the driving direction. The obstacle run starts where the
#   straight begins (start_at:=begin), so the robot has 3 m of straight first.
#   Centres are OBSTACLE_OUTSIDE (0.15 m) outside the track line, so each pushes
#   the robot INTO the track, never out of the lab.
#   The second is the first mirrored through the track centre.
#
#        top straight   <- driving      O  (cx-1, cy+R+0.15)
#      (                                          )
#        bottom straight -> driving     O  (cx+1, cy-R-0.15)
#
# The CBF keeps the marker midpoint r + robot_radius + safety_margin = 0.60 m
# from each centre, so the robot passes about 0.45 m inside the line.
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

OBSTACLE_RADIUS = 0.30     # m (0.60 m diameter)
OBSTACLE_OUTSIDE = 0.15    # m, centre outside the track line
OBSTACLE_AHEAD = 1.0       # m, past the middle of the straight, in the driving direction


# =============================================================================
# [UTIL]   lab_obstacles(straight, radius, cx, cy, direction, lab_x, lab_y) -> [x, y, r, x, y, r]
# -----------------------------------------------------------------------------
# Flat list for the bridge's 'obstacles' parameter (lab frame, metres). Raises
# ValueError if an obstacle is not on a straight or does not fit inside the lab.
# HOW: bottom obstacle at cx +- 1 m (sign from the direction: the bottom straight
# is driven +x when ccw), y = cy - radius - 0.15; the top one is the same point
# mirrored through the track centre. Rounded to 0.1 mm.
# =============================================================================
def lab_obstacles(straight, radius, cx, cy, direction='ccw', lab_x=None, lab_y=None):
    if OBSTACLE_AHEAD > straight / 2.0:
        raise ValueError(f'Straights of {straight} m are too short for obstacles {OBSTACLE_AHEAD} m past the middle')
    sign = 1.0 if direction == 'ccw' else -1.0     # bottom straight is driven +X when ccw
    r = OBSTACLE_RADIUS
    obstacles = [
        (cx + sign * OBSTACLE_AHEAD, cy - radius - OBSTACLE_OUTSIDE, r),   # bottom straight
        (cx - sign * OBSTACLE_AHEAD, cy + radius + OBSTACLE_OUTSIDE, r),   # top straight
    ]
    if lab_x is not None and lab_y is not None:
        for x, y, r in obstacles:
            if x - r < lab_x[0] or x + r > lab_x[1] or y - r < lab_y[0] or y + r > lab_y[1]:
                raise ValueError(f'Obstacle at ({x:.2f}, {y:.2f}) r {r} does not fit the lab {list(lab_x)} {list(lab_y)}')
    return [round(float(v), 4) for ob in obstacles for v in ob]
