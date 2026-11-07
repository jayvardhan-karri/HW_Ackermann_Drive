# #############################################################################
# [FILE]   limo_test.launch.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/launch/limo_test.launch.py
# ROLE  : Start limo_test_suite (wheels-up open-loop test) with /cmd_vel sent
#         to the LIMO's namespaced topic.
#
# Usage : ros2 launch limo_mocap_suite limo_test.launch.py
#         ros2 launch limo_mocap_suite limo_test.launch.py path:=circle speed:=0.15
# Args  : robot_ns  namespace of the LIMO's topics (default limo1):
#                   /cmd_vel -> /<robot_ns>/cmd_vel; none = plain /cmd_vel
#         path      straight | circle | figure8 | drawn  (default figure8)
#         speed     m/s during the test (default 0.20)
#         view      behind | topdown (default behind); use the same in the builder
# Open loop: the commands come from the path only, no odometry or mocap, so it
# checks the connection and the actuators, not the tracking.
# Asked : path and speed. Whatever is given on the command line is used and NOT
#         asked. robot_ns and view are never asked and keep their defaults.
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

from limo_mocap_suite.launch_prompts import Ask, ask_first
from limo_mocap_suite.robot_topics import DEFAULT_ROBOT_NS


# =============================================================================
# [LIFE]   _test_node(context) -> [Node]  (helper of generate_launch_description)
# -----------------------------------------------------------------------------
# Runs at launch time (OpaqueFunction), after the questions were answered.
# Returns the limo_test_suite Node with path, speed, view and robot_ns. The node
# itself maps robot_ns to /<robot_ns>/cmd_vel, so no remapping is needed here.
# =============================================================================
def _test_node(context):
    return [Node(
        package='limo_mocap_suite',
        executable='limo_test_suite',
        output='screen',
        parameters=[{'path': LaunchConfiguration('path').perform(context),
                     'speed': float(LaunchConfiguration('speed').perform(context)),
                     'view': LaunchConfiguration('view').perform(context),
                     'robot_ns': LaunchConfiguration('robot_ns').perform(context)}],
    )]


# =============================================================================
# [LIFE]   generate_launch_description()
# -----------------------------------------------------------------------------
# Declares robot_ns / path / speed / view, asks for path and speed (launch_prompts;
# their empty default means "not given, ask") and starts the test suite.
# =============================================================================
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value=DEFAULT_ROBOT_NS,
                              description="LIMO topic namespace (none = no namespace)"),
        DeclareLaunchArgument('path', default_value='',
                              description='straight | circle | figure8 | drawn'),
        DeclareLaunchArgument('speed', default_value='', description='test speed (m/s)'),
        DeclareLaunchArgument('view', default_value='behind', description='behind | topdown'),
        ask_first('limo_test: open-loop drive, no feedback (checks the connection)', [
            Ask('path', 'Path', 'figure8', [('straight', 'straight 2 m'), ('circle', 'circle'), ('figure8', 'figure 8'),
                                    ('drawn', 'a path drawn in the builder')]),
            Ask('speed', 'Speed (m/s)', '0.20', kind='float'),
        ]),
        OpaqueFunction(function=_test_node),
    ])
