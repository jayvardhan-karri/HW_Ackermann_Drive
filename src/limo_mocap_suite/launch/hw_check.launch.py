# #############################################################################
# [FILE]   hw_check.launch.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/launch/hw_check.launch.py
# ROLE  : Lab check before tracking: bridge + hw_check.
#
# Usage : ros2 launch limo_mocap_suite hw_check.launch.py               # robot still
#         ros2 launch limo_mocap_suite hw_check.launch.py drive:=true   # slow test drive
# Args  : drive, speed, ref_offset, max_range, robot_ns, bridge, log_dir
# Asked : drive (still check, or also the slow test drive). Whatever is given on
#         the command line (drive:=true) is used and NOT asked; the other args are
#         never asked and keep their defaults.
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
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from limo_mocap_suite.launch_prompts import CONTROLLERS as CHOICES, Ask, ask_first
from limo_mocap_suite.robot_topics import DEFAULT_ROBOT_NS, robot_remappings


# =============================================================================
# [LIFE]   _nodes(context) -> [Node, ...]  (helper of generate_launch_description)
# -----------------------------------------------------------------------------
# Runs at launch time (OpaqueFunction). robot_ns remaps /cmd_vel for hw_check.
# The bridge (bridge:=false if one is already running) and the check. When the
# check ends, everything ends.
# =============================================================================
def _nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context).strip()  # noqa: E731
    check = Node(package='limo_mocap_suite', executable='hw_check', output='screen', emulate_tty=True,
                 parameters=[{'drive': arg('drive').lower() == 'true', 'speed': float(arg('speed')),
                              'ref_offset': float(arg('ref_offset')), 'max_range': float(arg('max_range')),
                              'log_dir': arg('log_dir')}],
                 remappings=robot_remappings(arg('robot_ns')))
    nodes = [check, RegisterEventHandler(OnProcessExit(target_action=check, on_exit=[EmitEvent(event=Shutdown())]))]
    if arg('bridge').lower() == 'true':
        nodes.insert(0, Node(package='limo_mocap_suite', executable='phasespace_bridge', output='screen',
                             emulate_tty=True))
    return nodes


# =============================================================================
# [LIFE]   generate_launch_description()
# -----------------------------------------------------------------------------
# Declares the arguments (name, default, help text), asks for 'drive' first
# (launch_prompts), then runs _nodes. drive has an empty default: empty means
# "not given, ask".
# =============================================================================
def generate_launch_description():
    defaults = (('drive', '', 'true: also do the slow test drive. Needs clear space around the robot (false)'),
                ('speed', '0.15', 'Test drive speed (m/s)'),
                ('ref_offset', '0.10', 'Marker midpoint ahead of the rear axle (m)'),
                ('max_range', '3.0', 'Stop if the robot gets this far from where it started (m)'),
                ('robot_ns', DEFAULT_ROBOT_NS, 'LIMO topic namespace (none = no namespace)'),
                ('bridge', 'true', 'false: a phasespace_bridge is already running'),
                ('log_dir', 'evaluation', 'Where the report folder goes'))
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in defaults]
                             + [ask_first('hw_check: is the lab set up right?', [
                                 Ask('drive', 'Check', 'false', [('false', 'robot standing still'),
                                                                  ('true', 'also a slow test drive (needs 3 m clear)')]),
                             ]), OpaqueFunction(function=_nodes)])
