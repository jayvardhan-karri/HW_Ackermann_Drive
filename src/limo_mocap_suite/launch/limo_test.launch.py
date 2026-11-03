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

from limo_mocap_suite.robot_topics import DEFAULT_ROBOT_NS, robot_remappings


# =============================================================================
# [LIFE]   _test_node(context)
# -----------------------------------------------------------------------------
# Returns the limo_test_suite Node with path / speed and the robot_ns remap.
# =============================================================================
def _test_node(context):
    return [Node(
        package='limo_mocap_suite',
        executable='limo_test_suite',
        output='screen',
        parameters=[{'path': LaunchConfiguration('path').perform(context),
                     'speed': float(LaunchConfiguration('speed').perform(context))}],
        remappings=robot_remappings(LaunchConfiguration('robot_ns').perform(context)),
    )]


# =============================================================================
# [LIFE]   generate_launch_description()
# -----------------------------------------------------------------------------
# Declares robot_ns / path / speed and starts the test suite.
# =============================================================================
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value=DEFAULT_ROBOT_NS,
                              description="LIMO topic namespace (none = no namespace)"),
        DeclareLaunchArgument('path', default_value='figure8',
                              description='straight | circle | figure8 | drawn'),
        DeclareLaunchArgument('speed', default_value='0.20', description='test speed (m/s)'),
        OpaqueFunction(function=_test_node),
    ])
