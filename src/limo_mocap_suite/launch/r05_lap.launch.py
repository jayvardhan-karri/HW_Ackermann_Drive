# #############################################################################
# [FILE]   r05_lap.launch.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/launch/r05_lap.launch.py
# ROLE  : ADD-ON test, kept apart from the main suites. The lab track with 0.5 m
#         curves instead of 1 m (same 4 m straights and centre: x 0.75..5.75,
#         y 0.75..1.75). 0.5 m is close to the LIMO's tightest turn (about
#         0.46 m), so the steering sits near its limit in every curve: it shows
#         how each controller copes at the edge of what the robot can do.
#         Same rules as the R1 suites: auto start side, merge onto the track,
#         CCW, one report.
#
# Usage : ros2 launch limo_mocap_suite r05_lap.launch.py          (asks what is missing)
#         ros2 launch limo_mocap_suite r05_lap.launch.py controllers:=lqr,nmpc laps:=3
#         ros2 launch limo_mocap_suite r05_lap.launch.py sim:=true    # rehearsal
#         Any run_laps.launch.py argument works too.
# Asked : controllers, laps, start_side, sim. Whatever is missing on the command
#         line is asked in the terminal (launch_prompts.ask_first, Enter = default).
#         A value given as name:=value is used and NOT asked.
# Engine: includes run_laps.launch.py with track_radii 0.5.
# Log   : <workspace>/evaluation/r05_lap_<stamp>/
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
# (launch_prompts), then run_laps.launch.py is included with track_radii 0.5.
# The arguments are declared with an empty default: empty means "not given, ask".
# =============================================================================
def generate_launch_description():
    path = os.path.join(get_package_share_directory('limo_mocap_suite'), 'launch', 'run_laps.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('controllers', default_value='', description='Controllers, in order (all four)'),
        DeclareLaunchArgument('laps', default_value='', description='Laps per controller (3)'),
        DeclareLaunchArgument('start_side', default_value='', description='auto, top or bottom (auto)'),
        DeclareLaunchArgument('sim', default_value='', description='true: rehearsal with the stand-in robot (false)'),
        ask_first('r05_lap: R0.5 track, near the steering limit (add-on)', [
            Ask('controllers', 'Controllers', 'pid,lqr,mpc,nmpc', CHOICES, kind='many', strict=False),
            Ask('laps', 'Laps each', '3', kind='int'),
            Ask('start_side', 'Start side', 'auto', START_SIDES),
            Ask('sim', 'Robot', 'false', SIM),
        ]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments={
            'controllers': LaunchConfiguration('controllers'),
            'laps': LaunchConfiguration('laps'),
            'track_radii': '0.5',
            'session_name': 'r05_lap',
        }.items()),
    ])
