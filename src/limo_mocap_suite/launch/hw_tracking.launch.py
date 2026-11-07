# #############################################################################
# [FILE]   hw_tracking.launch.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/launch/hw_tracking.launch.py
# ROLE  : One-command lab bring-up: bridge + one controller (+ optional GUI).
#         The path is drawn in the builder, or a capsule track is sent with
#         track_radius > 0. For repeatable lap tests with reports use
#         record_lap / compare_lap (they go through run_laps.launch.py).
# Asked : controller and track_radius (draw a path, or the R1 lab track).
#         Whatever is given on the command line (controller:=lqr) is used and NOT
#         asked; every other argument keeps its default.
#
# Usage : ros2 launch limo_mocap_suite hw_tracking.launch.py controller:=nmpc
# Args  : controller         pid | lqr | mpc | nmpc          (default nmpc)
#         builder            true | false: start the drawing GUI (default true)
#         plotter            true | false: start the live plotter (default true)
#         rezero_odom        true | false: path relative to start pose (default false =
#                            absolute mocap frame);
#                            forwarded to the controller AND both GUIs
#         run_duration       seconds, 0 = until the path end / Ctrl-C (0.0)
#         obstacles          "[x, y, r, ...]" fixed cylinders (default: none)
#         obstacle_markers   "[(0,1,2,3),(4,5,6,7)]" 4 rim markers per cylinder (none)
#         robot_radius, safety_margin, cbf_alpha  CBF clearance (0.20, 0.10, 0.6)
#         cbf_max_steer      steering the CBF plans with; 0 = max_steer (0.0)
#         lab_x, lab_y       "[min, max]" lab extents for both GUIs; '' = default
#                            ([-0.5, 7.0] and [-0.5, 3.0])
#         view               'behind' (+X up, +Y left) or 'topdown' for both GUIs
#         track_radius       0 = draw a path (default); > 0 = capsule track, no drawing
#         track_straight, track_x, track_y, start_delay   capsule size, centre, warning
#         guard              true: stop / rejoin on drift (extension; default false)
#         ref_offset         marker midpoint ahead of the rear axle in m (0.10);
#                            measure it on the robot, forwarded to the controller
#         robot_ns           namespace of the LIMO's own topics (default limo1):
#                            /cmd_vel -> /<robot_ns>/cmd_vel, /imu -> /<robot_ns>/imu
#                            for the controller; none = no namespace (plain /cmd_vel)
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from limo_mocap_suite.obstacle_cbf import parse_marker_groups, parse_obstacles
from limo_mocap_suite.launch_prompts import CONTROLLERS as CHOICES, Ask, ask_first
from limo_mocap_suite.robot_topics import DEFAULT_ROBOT_NS, parse_controller_params, robot_remappings
from limo_mocap_suite.track_tools import TRACK_STRAIGHT, TRACK_X, TRACK_Y

CONTROLLERS = ('pid', 'lqr', 'mpc', 'nmpc')    # names accepted for controller:=


# =============================================================================
# [LIFE]   _controller_node(context) -> [Node]  (helper of generate_launch_description)
# -----------------------------------------------------------------------------
# Runs at launch time (OpaqueFunction), when the answers are known.
# Resolves the 'controller' argument and returns the matching
# <controller>_tracker Node with rezero_odom / run_duration forwarded and
# /cmd_vel, /imu remapped into robot_ns (robot_remappings).
# Raises on an unknown controller name.
# =============================================================================
def _controller_node(context):
    name = LaunchConfiguration('controller').perform(context).lower()
    if name not in CONTROLLERS:
        raise RuntimeError(f"controller must be one of {CONTROLLERS}, got '{name}'")
    rezero = LaunchConfiguration('rezero_odom').perform(context).lower() == 'true'
    duration = float(LaunchConfiguration('run_duration').perform(context))
    offset = float(LaunchConfiguration('ref_offset').perform(context))
    # Parameters of the tracker node, all read from the launch arguments
    params = {'rezero_odom': rezero, 'run_duration': duration, 'ref_offset': offset,
              'laps': int(LaunchConfiguration('laps').perform(context)),
              'guard': LaunchConfiguration('guard').perform(context).lower() == 'true',
              'approach': LaunchConfiguration('approach').perform(context).lower() == 'true',
              'log_dir': LaunchConfiguration('log_dir').perform(context),
              'path_name': LaunchConfiguration('path_name').perform(context)}
    for arg in ('approach_radius', 'approach_speed', 'slow_cte', 'rejoin_cte', 'abort_cte'):
        params[arg] = float(LaunchConfiguration(arg).perform(context))
    for arg, key in (('lab_x', 'guard_lab_x'), ('lab_y', 'guard_lab_y')):
        text = LaunchConfiguration(arg).perform(context).strip()
        if text:
            params[key] = [float(v) for v in yaml.safe_load(text)]    # "[min, max]" text -> list
    for arg in ('robot_radius', 'safety_margin', 'cbf_alpha', 'cbf_max_steer'):
        params[arg] = float(LaunchConfiguration(arg).perform(context))
    params.update(parse_controller_params(LaunchConfiguration('controller_params').perform(context)))  # lab overrides
    return [Node(
        package='limo_mocap_suite',
        executable=f'{name}_tracker',
        output='screen',
        emulate_tty=True,
        parameters=[params],
        remappings=robot_remappings(LaunchConfiguration('robot_ns').perform(context)),
    )]


# =============================================================================
# [LIFE]   _bridge_node(context) -> [Node]  (helper of generate_launch_description)
# -----------------------------------------------------------------------------
# Returns the phasespace_bridge Node (markers -> /hardware_odom) with the 'obstacles' and 'obstacle_markers'
# strings, each forwarded only when non-empty. Bad values fail here, at launch.
# =============================================================================
def _bridge_node(context):
    params = {}
    obstacles = parse_obstacles(LaunchConfiguration('obstacles').perform(context).strip())
    if obstacles:
        params['obstacles'] = obstacles
    markers = LaunchConfiguration('obstacle_markers').perform(context).strip()
    if parse_marker_groups(markers):
        params['obstacle_markers'] = markers
    return [Node(
        package='limo_mocap_suite',
        executable='phasespace_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[params],
    )]


# =============================================================================
# [LIFE]   _track_node(context) -> [Node] or []  (helper of generate_launch_description)
# -----------------------------------------------------------------------------
# With track_radius > 0, track_publisher sends the capsule instead of a drawn path
# (/reference_path); with 0 it returns no node. The track sits in the lab frame,
# so rezero_odom:=true is refused.
# =============================================================================
def _track_node(context):
    radius = float(LaunchConfiguration('track_radius').perform(context))
    if radius <= 0.0:
        return []
    if LaunchConfiguration('rezero_odom').perform(context).lower() == 'true':
        raise RuntimeError('track_radius places the track in the lab frame, so it needs rezero_odom:=false')
    params = {'track_radius': radius}
    for arg in ('track_straight', 'track_x', 'track_y', 'start_delay'):
        params[arg] = float(LaunchConfiguration(arg).perform(context))
    for arg in ('start_side', 'direction', 'start_at'):
        params[arg] = LaunchConfiguration(arg).perform(context)
    for arg in ('lab_x', 'lab_y'):
        text = LaunchConfiguration(arg).perform(context).strip()
        if text:
            params[arg] = [float(v) for v in yaml.safe_load(text)]
    return [Node(package='limo_mocap_suite', executable='track_publisher', output='screen',
                 emulate_tty=True, parameters=[params])]


# =============================================================================
# [LIFE]   _gui_nodes(context) -> [Node, ...]  (helper of generate_launch_description)
# -----------------------------------------------------------------------------
# Returns interactive_builder and live_plotter (each only if its argument is
# true) with rezero_odom and, when non-empty, lab_x / lab_y (YAML lists).
# The builder is skipped for a capsule track (nothing to draw). live_plotter gets
# the robot remapping because its KILL button sends zero commands on /cmd_vel.
# =============================================================================
def _gui_nodes(context):
    params = {'rezero_odom': LaunchConfiguration('rezero_odom').perform(context).lower() == 'true',
              'view': LaunchConfiguration('view').perform(context)}
    for name in ('lab_x', 'lab_y'):
        text = LaunchConfiguration(name).perform(context).strip()
        if text:
            params[name] = [float(v) for v in yaml.safe_load(text)]
    remap = robot_remappings(LaunchConfiguration('robot_ns').perform(context))   # KILL sends zero commands
    nodes = []
    # drawing = no capsule track, so the builder is useful
    for arg, exe in (('builder', 'interactive_builder'), ('plotter', 'live_plotter')):
        drawing = float(LaunchConfiguration('track_radius').perform(context)) <= 0.0
        if LaunchConfiguration(arg).perform(context).lower() == 'true' and (drawing or exe != 'interactive_builder'):
            nodes.append(Node(package='limo_mocap_suite', executable=exe,
                              output='screen', parameters=[params],
                              remappings=remap if exe == 'live_plotter' else []))
    return nodes


# =============================================================================
# [LIFE]   generate_launch_description()
# -----------------------------------------------------------------------------
# Declares the launch arguments, asks for controller and track_radius
# (launch_prompts; their empty default means "not given, ask"), then starts:
# 1. phasespace_bridge (via _bridge_node: obstacles, obstacle_markers),
# 2. the selected controller (via _controller_node),
# 3. interactive_builder if builder:=true, live_plotter if plotter:=true
#    (via _gui_nodes: rezero_odom, lab_x, lab_y),
# 4. track_publisher when track_radius > 0 (via _track_node).
# =============================================================================
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='',
                              description='pid | lqr | mpc | nmpc (nmpc)'),
        DeclareLaunchArgument('builder', default_value='true',
                              description='Start the interactive path builder GUI'),
        DeclareLaunchArgument('plotter', default_value='true',
                              description='Start the live plotter GUI'),
        DeclareLaunchArgument('rezero_odom', default_value='false',
                              description='false: absolute MoCap frame (default); true: path relative to the start pose'),
        DeclareLaunchArgument('laps', default_value='1',
                              description='Laps on a closed path; 0 = until stopped. Open paths run once'),
        DeclareLaunchArgument('log_dir', default_value='evaluation',
                              description='Each run gets its own folder here'),
        DeclareLaunchArgument('track_radius', default_value='',
                              description='0 = draw a path; > 0 = capsule track with this curve radius (m)'),
        DeclareLaunchArgument('track_straight', default_value=str(TRACK_STRAIGHT), description='Capsule straight length (m)'),
        DeclareLaunchArgument('track_x', default_value=str(TRACK_X), description='Capsule centre X (m)'),
        DeclareLaunchArgument('track_y', default_value=str(TRACK_Y), description='Capsule centre Y (m)'),
        DeclareLaunchArgument('start_side', default_value='auto',
                              description="Race start straight: 'auto' (top, or bottom if the robot is nearer it), 'top', 'bottom'"),
        DeclareLaunchArgument('direction', default_value='ccw', description="'ccw' or 'cw'"),
        DeclareLaunchArgument('start_at', default_value='middle', description="'middle' or 'begin' of the straight"),
        DeclareLaunchArgument('start_delay', default_value='5.0', description='Capsule: seconds of warning before moving'),
        DeclareLaunchArgument('guard', default_value='false',
                              description='true: slow, rejoin and stop on drift (extension)'),
        DeclareLaunchArgument('approach', default_value='true',
                              description='true: drive to the path start first; false: track directly'),
        DeclareLaunchArgument('path_name', default_value='drawn', description='Label in the run folder name'),
        DeclareLaunchArgument('approach_radius', default_value='0.60',
                              description='Turning radius used to reach the path (m)'),
        DeclareLaunchArgument('approach_speed', default_value='0.25', description='Speed on the way to the path (m/s)'),
        DeclareLaunchArgument('slow_cte', default_value='0.15', description='Guard: slow down above this error (m)'),
        DeclareLaunchArgument('rejoin_cte', default_value='0.35', description='Guard: plan a way back above this error (m)'),
        DeclareLaunchArgument('abort_cte', default_value='1.00', description='Guard: stop above this error (m)'),
        DeclareLaunchArgument('run_duration', default_value='0.0',
                              description='Run time limit in seconds (0 = none)'),
        DeclareLaunchArgument('obstacles', default_value='',
                              description='Fixed cylinders "[x, y, r, ...]" (empty = none)'),
        DeclareLaunchArgument('obstacle_markers', default_value='',
                              description='4 rim markers per cylinder "[(0,1,2,3),(4,5,6,7)]" (empty = none)'),
        DeclareLaunchArgument('robot_radius', default_value='0.20',
                              description='Body radius around the tracked point (m)'),
        DeclareLaunchArgument('safety_margin', default_value='0.10',
                              description='Extra clearance beyond the body (m)'),
        DeclareLaunchArgument('cbf_max_steer', default_value='0.0',
                              description='Steering limit for the CBF (rad); 0 = max_steer'),
        DeclareLaunchArgument('cbf_alpha', default_value='0.6',
                              description='CBF gain (1/s)'),
        DeclareLaunchArgument('view', default_value='behind',
                              description="GUI orientation: 'behind' or 'topdown'"),
        DeclareLaunchArgument('lab_x', default_value='',
                              description='GUI lab X extent "[min, max]" (empty = default)'),
        DeclareLaunchArgument('lab_y', default_value='',
                              description='GUI lab Y extent "[min, max]" (empty = default)'),
        DeclareLaunchArgument('ref_offset', default_value='0.10',
                              description='Marker midpoint ahead of the rear axle (m)'),
        DeclareLaunchArgument('controller_params', default_value='',
                              description='Change controller parameters without editing code: "q_cte:=300 gain_cache:=false"'),
        DeclareLaunchArgument('robot_ns', default_value=DEFAULT_ROBOT_NS,
                              description="LIMO topic namespace: /cmd_vel -> /<robot_ns>/cmd_vel (none = no namespace)"),

        ask_first('hw_tracking: one controller on a drawn path or the lab track', [
            Ask('controller', 'Controller', 'nmpc', CHOICES),
            Ask('track_radius', 'Path', '0.0', [('0.0', 'draw it in the path builder'), ('1.0', 'R1 lab track')], strict=False),
        ]),
        OpaqueFunction(function=_bridge_node),
        OpaqueFunction(function=_controller_node),
        OpaqueFunction(function=_gui_nodes),
        OpaqueFunction(function=_track_node),
    ])
