# #############################################################################
# [FILE]   hw_tracking.launch.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/launch/hw_tracking.launch.py
# ROLE  : One-command lab bring-up: bridge + one controller (+ optional GUI).
#
# Usage : ros2 launch limo_mocap_suite hw_tracking.launch.py controller:=nmpc
# Args  : controller         pid | lqr | mpc | nmpc          (default nmpc)
#         builder            true | false: start the drawing GUI (default true)
#         plotter            true | false: start the live plotter (default true)
#         use_synthetic_scan true | false: bridge fakes /hardware_scan (true)
#         rezero_odom        true | false: path relative to start pose (true);
#                            forwarded to the controller AND both GUIs
#         run_duration       seconds, 0 = until the path end / Ctrl-C (0.0)
#         obstacles          "[x, y, r, ...]" for the bridge; '' = bridge default
#         lab_x, lab_y       "[min, max]" lab extents for both GUIs; '' = default
#                            ([-0.5, 8.0] and [-1.0, 6.0])
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
# #############################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from limo_mocap_suite.robot_topics import DEFAULT_ROBOT_NS, robot_remappings

CONTROLLERS = ('pid', 'lqr', 'mpc', 'nmpc')


# =============================================================================
# [LIFE]   _controller_node(context)
# -----------------------------------------------------------------------------
# Resolves the 'controller' argument at launch time and returns the matching
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
    return [Node(
        package='limo_mocap_suite',
        executable=f'{name}_tracker',
        output='screen',
        emulate_tty=True,
        parameters=[{'rezero_odom': rezero, 'run_duration': duration}],
        remappings=robot_remappings(LaunchConfiguration('robot_ns').perform(context)),
    )]


# =============================================================================
# [LIFE]   _bridge_node(context)
# -----------------------------------------------------------------------------
# Returns the phasespace_bridge Node. use_synthetic_scan is forwarded; the
# 'obstacles' string is parsed as YAML ("[1.97, 0.81, 0.35]") and forwarded
# only when non-empty, so '' keeps the bridge's built-in default map.
# =============================================================================
def _bridge_node(context):
    params = {'use_synthetic_scan':
              LaunchConfiguration('use_synthetic_scan').perform(context).lower() == 'true'}
    obstacles = LaunchConfiguration('obstacles').perform(context).strip()
    if obstacles:
        params['obstacles'] = [float(v) for v in yaml.safe_load(obstacles)]
    return [Node(
        package='limo_mocap_suite',
        executable='phasespace_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[params],
    )]


# =============================================================================
# [LIFE]   _gui_nodes(context)
# -----------------------------------------------------------------------------
# Returns interactive_builder and live_plotter (each only if its argument is
# true) with rezero_odom and, when non-empty, lab_x / lab_y (YAML lists).
# =============================================================================
def _gui_nodes(context):
    params = {'rezero_odom': LaunchConfiguration('rezero_odom').perform(context).lower() == 'true'}
    for name in ('lab_x', 'lab_y'):
        text = LaunchConfiguration(name).perform(context).strip()
        if text:
            params[name] = [float(v) for v in yaml.safe_load(text)]
    nodes = []
    for arg, exe in (('builder', 'interactive_builder'), ('plotter', 'live_plotter')):
        if LaunchConfiguration(arg).perform(context).lower() == 'true':
            nodes.append(Node(package='limo_mocap_suite', executable=exe,
                              output='screen', parameters=[params]))
    return nodes


# =============================================================================
# [LIFE]   generate_launch_description()
# -----------------------------------------------------------------------------
# Declares the launch arguments, then starts:
# 1. phasespace_bridge (via _bridge_node: use_synthetic_scan, obstacles),
# 2. the selected controller (via _controller_node),
# 3. interactive_builder if builder:=true, live_plotter if plotter:=true
#    (via _gui_nodes: rezero_odom, lab_x, lab_y).
# =============================================================================
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='nmpc',
                              description='pid | lqr | mpc | nmpc'),
        DeclareLaunchArgument('builder', default_value='true',
                              description='Start the interactive path builder GUI'),
        DeclareLaunchArgument('plotter', default_value='true',
                              description='Start the live plotter GUI'),
        DeclareLaunchArgument('use_synthetic_scan', default_value='true',
                              description='Bridge publishes a ray-cast /hardware_scan'),
        DeclareLaunchArgument('rezero_odom', default_value='true',
                              description='Interpret the path relative to the start pose'),
        DeclareLaunchArgument('run_duration', default_value='0.0',
                              description='Run time limit in seconds (0 = none)'),
        DeclareLaunchArgument('obstacles', default_value='',
                              description='Bridge obstacle map "[x, y, r, ...]" (empty = default)'),
        DeclareLaunchArgument('lab_x', default_value='',
                              description='GUI lab X extent "[min, max]" (empty = default)'),
        DeclareLaunchArgument('lab_y', default_value='',
                              description='GUI lab Y extent "[min, max]" (empty = default)'),
        DeclareLaunchArgument('robot_ns', default_value=DEFAULT_ROBOT_NS,
                              description="LIMO topic namespace: /cmd_vel -> /<robot_ns>/cmd_vel (none = no namespace)"),

        OpaqueFunction(function=_bridge_node),
        OpaqueFunction(function=_controller_node),
        OpaqueFunction(function=_gui_nodes),
    ])
