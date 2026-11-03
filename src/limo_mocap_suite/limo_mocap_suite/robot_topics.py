# #############################################################################
# [FILE]   robot_topics.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/robot_topics.py
# ROLE  : Which topics belong to the LIMO itself, and the remapping rules that
#         move them into the robot's namespace (e.g. /limo1/cmd_vel).
#
# The nodes use absolute names (/cmd_vel, /imu). A LIMO started with a
# namespace (`ros2 topic list` shows /limo1/cmd_vel ...) listens elsewhere, so
# the launch files remap these two topics. Topics produced by this package
# (/hardware_odom, /hardware_scan, /hardware_obstacles, /reference_path,
# /reference_path_viz) are not the robot's and are never remapped.
# Without launch:  ros2 run ... --ros-args -r /cmd_vel:=/limo1/cmd_vel
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
# #############################################################################

ROBOT_TOPICS = ('/cmd_vel', '/imu')
DEFAULT_ROBOT_NS = 'limo1'


# =============================================================================
# [UTIL]   robot_remappings(robot_ns) -> list of (from, to)
# -----------------------------------------------------------------------------
# robot_ns 'limo1' -> [('/cmd_vel', '/limo1/cmd_vel'), ('/imu', '/limo1/imu')].
# 'none', '/' or '' -> [] (robot without namespace; use robot_ns:=none on the
# command line, ros2 launch rejects an empty value). Leading / trailing '/'
# are ignored.
# =============================================================================
def robot_remappings(robot_ns):
    ns = robot_ns.strip().strip('/')
    if not ns or ns.lower() == 'none':
        return []
    return [(t, '/' + ns + t) for t in ROBOT_TOPICS]
