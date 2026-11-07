# #############################################################################
# [FILE]   record_lap.launch.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/launch/record_lap.launch.py
# ROLE  : Suite 1, record and analyse ONE controller: 3 laps of the R1 lab track
#         from a race start, then report.md with charts. The robot drives itself
#         to the start first, from anywhere in the mocap area (Dubins path).
#
# Usage : ros2 launch limo_mocap_suite record_lap.launch.py controller:=lqr
#         ros2 launch limo_mocap_suite record_lap.launch.py controller:=nmpc start_side:=top
#         ros2 launch limo_mocap_suite record_lap.launch.py controller:=pid sim:=true   # rehearsal
#         Any run_laps.launch.py argument works too (laps, v_nominal, direction, ...).
# Asked : controller, laps, start_side, sim. Whatever is missing on the command
#         line is asked in the terminal (launch_prompts.ask_first, Enter = default).
#         A value given as name:=value is used and NOT asked.
# Engine: includes run_laps.launch.py with one controller on the R1 track.
# Log   : <workspace>/evaluation/record_<controller>_<stamp>/
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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from limo_mocap_suite.launch_prompts import CONTROLLERS as CHOICES, SIM, START_SIDES, Ask, ask_first

CONTROLLERS = ('pid', 'lqr', 'mpc', 'nmpc')    # names accepted for controller:=


# =============================================================================
# [LIFE]   _include(context) -> [IncludeLaunchDescription]  (helper of generate_launch_description)
# -----------------------------------------------------------------------------
# Runs at launch time (OpaqueFunction), after the questions were answered, so
# the launch values can be read. Checks the controller name and includes
# run_laps.launch.py for it alone, with the R1 track (track_radii 1.0) and the
# session name record_<controller>.
# =============================================================================
def _include(context):
    name = LaunchConfiguration('controller').perform(context).strip().lower()
    if name not in CONTROLLERS:
        raise RuntimeError(f'controller:={name!r}: choose one of {", ".join(CONTROLLERS)}')
    path = os.path.join(get_package_share_directory('limo_mocap_suite'), 'launch', 'run_laps.launch.py')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments={
        'controllers': name,
        'laps': LaunchConfiguration('laps').perform(context),
        'track_radii': '1.0',
        'session_name': f'record_{name}',
    }.items())]


# =============================================================================
# [LIFE]   generate_launch_description()
# -----------------------------------------------------------------------------
# Arguments left out on the command line are asked in the terminal first
# (launch_prompts), then _include adds run_laps.launch.py. The arguments are
# declared with an empty default: empty means "not given, ask".
# =============================================================================
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='', description='Controller to record: pid, lqr, mpc or nmpc'),
        DeclareLaunchArgument('laps', default_value='', description='Laps to record (3)'),
        DeclareLaunchArgument('start_side', default_value='', description='auto, top or bottom (auto)'),
        DeclareLaunchArgument('sim', default_value='', description='true: rehearsal with the stand-in robot (false)'),
        ask_first('record_lap: one controller, laps on the R1 lab track', [
            Ask('controller', 'Controller', 'nmpc', CHOICES),
            Ask('laps', 'Laps', '3', kind='int'),
            Ask('start_side', 'Start side', 'auto', START_SIDES),
            Ask('sim', 'Robot', 'false', SIM),
        ]),
        OpaqueFunction(function=_include),
    ])
