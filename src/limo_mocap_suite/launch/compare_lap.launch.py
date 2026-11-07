# #############################################################################
# [FILE]   compare_lap.launch.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/launch/compare_lap.launch.py
# ROLE  : Suite 2, compare all four controllers: PID, LQR, LTV-MPC, NMPC one after
#         the other on the R1 lab track, 3 laps each. Between two controllers the
#         robot is held stopped for buffer_time, then the next one drives itself
#         back to the race start (Dubins path, logged apart from the laps).
#         One report compares them at the end.
#
# Usage : ros2 launch limo_mocap_suite compare_lap.launch.py
#         ros2 launch limo_mocap_suite compare_lap.launch.py buffer_time:=20 start_side:=top
#         ros2 launch limo_mocap_suite compare_lap.launch.py controllers:=lqr,nmpc
#         ros2 launch limo_mocap_suite compare_lap.launch.py sim:=true   # rehearsal
#         Any run_laps.launch.py argument works too.
# Asked : controllers, laps, buffer_time, start_side, sim. Whatever is missing on
#         the command line is asked in the terminal (launch_prompts.ask_first,
#         Enter = default). A value given as name:=value is used and NOT asked.
# Engine: includes run_laps.launch.py with the R1 track and session name 'compare'.
# Log   : <workspace>/evaluation/compare_<stamp>/
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
# #############################################################################
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from limo_mocap_suite.launch_prompts import CONTROLLERS as CHOICES, SIM, START_SIDES, Ask, ask_first


# =============================================================================
# [LIFE]   generate_launch_description()
# -----------------------------------------------------------------------------
# Arguments left out on the command line are asked in the terminal first
# (launch_prompts), then run_laps.launch.py is included. The arguments are
# declared with an empty default: empty means "not given, ask".
# =============================================================================
def generate_launch_description():
    path = os.path.join(get_package_share_directory('limo_mocap_suite'), 'launch', 'run_laps.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('controllers', default_value='', description='Controllers, in order (all four)'),
        DeclareLaunchArgument('laps', default_value='', description='Laps per controller (3)'),
        DeclareLaunchArgument('buffer_time', default_value='', description='Seconds stopped between controllers (10)'),
        DeclareLaunchArgument('start_side', default_value='', description='auto, top or bottom (auto)'),
        DeclareLaunchArgument('sim', default_value='', description='true: rehearsal with the stand-in robot (false)'),
        ask_first('compare_lap: controllers one after the other on the R1 lab track', [
            Ask('controllers', 'Controllers', 'pid,lqr,mpc,nmpc', CHOICES, kind='many', strict=False),
            Ask('laps', 'Laps each', '3', kind='int'),
            Ask('buffer_time', 'Pause between (s)', '10.0', kind='float'),
            Ask('start_side', 'Start side', 'auto', START_SIDES),
            Ask('sim', 'Robot', 'false', SIM),
        ]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments={
            'controllers': LaunchConfiguration('controllers'),
            'laps': LaunchConfiguration('laps'),
            'buffer_time': LaunchConfiguration('buffer_time'),
            'track_radii': '1.0',
            'session_name': 'compare',
        }.items()),
    ])
