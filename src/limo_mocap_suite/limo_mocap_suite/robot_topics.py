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
# (/hardware_odom, /hardware_obstacles, /reference_path,
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

# Topics that belong to the robot; only these are remapped into its namespace
ROBOT_TOPICS = ('/cmd_vel', '/imu')
DEFAULT_ROBOT_NS = 'limo1'    # namespace of the lab LIMO


# =============================================================================
# [UTIL]   robot_remappings(robot_ns) -> list of (from, to)
# -----------------------------------------------------------------------------
# Used by the launch files: hw_tracking, run_laps, hw_check, limo_test.
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

# =============================================================================
# [UTIL]   parse_controller_params(text) -> {name: value}
# -----------------------------------------------------------------------------
# "q_cte:=300 r_steer:=0.8 gain_cache:=false" (spaces or commas between) ->
# {'q_cte': 300, 'r_steer': 0.8, 'gain_cache': False}. Values are read as YAML,
# so true/false and lists keep their type. A whole number becomes a float
# (q_cte:=300 -> 300.0) unless the parameter is one of INT_PARAMS, because ROS
# refuses an int for a parameter declared as a float. Used to change a
# controller parameter at launch, in the lab, without editing code.
# =============================================================================
INT_PARAMS = ('laps', 'lookahead_pts', 'horizon_steps', 'max_rejoins')   # declared as int in the nodes


def parse_controller_params(text):
    import yaml
    out = {}
    for item in str(text or '').replace(',', ' ').split():
        if ':=' not in item:
            raise ValueError(f'controller_params: "{item}" is not name:=value')
        name, value = item.split(':=', 1)
        name, value = name.strip(), yaml.safe_load(value)
        if isinstance(value, int) and not isinstance(value, bool):
            value = int(value) if name in INT_PARAMS else float(value)
        out[name] = value
    return out
