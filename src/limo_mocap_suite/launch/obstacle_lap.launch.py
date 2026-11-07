# #############################################################################
# [FILE]   obstacle_lap.launch.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/launch/obstacle_lap.launch.py
# ROLE  : ADD-ON, kept apart from the main suites. The lap test on the R1 lab
#         track with two fixed obstacles (0.60 m diameter, lab_obstacles.py), one
#         on each straight, centred just outside the line so the robot is pushed
#         into the track. The bridge publishes them; the live view draws them (red
#         discs) and the controllers go round them (CBF, or constraints in the
#         LTV-MPC / NMPC with obstacle_mode:=constraint).
#         The robot starts where the straight begins (start_at:=begin), so it has
#         3 m of straight before the first obstacle.
#         Obstacles are virtual: nothing has to stand there. To make them real,
#         put 0.60 m cylinders at the positions printed at start-up.
#
# Usage : ros2 launch limo_mocap_suite obstacle_lap.launch.py                 # all four
#         ros2 launch limo_mocap_suite obstacle_lap.launch.py controllers:=lqr # one
#         ros2 launch limo_mocap_suite obstacle_lap.launch.py obstacle_mode:=constraint
#         ros2 launch limo_mocap_suite obstacle_lap.launch.py compare_modes:=true
#             LTV-MPC and NMPC with the CBF filter, then both with obstacles as
#             constraints inside the solver; one report (default for all: CBF)
#         ros2 launch limo_mocap_suite obstacle_lap.launch.py sim:=true        # rehearsal
# Asked : compare_modes, controllers (not asked when compare_modes is true),
#         laps, start_side, sim. Whatever is missing on the command line is asked
#         in the terminal (launch_prompts.ask_first, Enter = default). A value
#         given as name:=value is used and NOT asked. direction (ccw) is never asked.
# Engine: includes run_laps.launch.py with the R1 track and the two obstacles.
# Log   : <workspace>/evaluation/obstacle_lap_<stamp>/
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from limo_mocap_suite.launch_prompts import CONTROLLERS as CHOICES, SIM, START_SIDES, Ask, ask_first
from limo_mocap_suite.lab_obstacles import lab_obstacles
from limo_mocap_suite.track_tools import LAB_X, LAB_Y, TRACK_RADIUS, TRACK_STRAIGHT, TRACK_X, TRACK_Y


# =============================================================================
# [LIFE]   _include(context) -> [LogInfo, IncludeLaunchDescription]  (helper of generate_launch_description)
# -----------------------------------------------------------------------------
# Runs at launch time (OpaqueFunction), after the questions were answered.
# 1. lab_obstacles() gives the two obstacles [x, y, r, ...] (m) for the chosen
#    direction, placed on the R1 track.
# 2. compare_modes true: the controller list becomes the four runs
#    mpc, nmpc, mpc:constraint, nmpc:constraint.
# 3. Includes run_laps.launch.py with the obstacles, start_at 'begin'.
# =============================================================================
def _include(context):
    direction = LaunchConfiguration('direction').perform(context).strip()
    flat = lab_obstacles(TRACK_STRAIGHT, TRACK_RADIUS, TRACK_X, TRACK_Y, direction, LAB_X, LAB_Y)
    text = '[' + ', '.join(f'{v:g}' for v in flat) + ']'
    where = '; '.join(f'({flat[i]:g}, {flat[i + 1]:g}) r {flat[i + 2]:g}' for i in range(0, len(flat), 3))
    path = os.path.join(get_package_share_directory('limo_mocap_suite'), 'launch', 'run_laps.launch.py')
    # name:constraint = that controller with the obstacles as solver constraints
    controllers = LaunchConfiguration('controllers').perform(context)
    session = 'obstacle_lap'
    if LaunchConfiguration('compare_modes').perform(context).lower() == 'true':
        controllers, session = 'mpc,nmpc,mpc:constraint,nmpc:constraint', 'obstacle_modes'
    return [
        LogInfo(msg=f'Obstacle add-on: {where} (lab frame, m); runs: {controllers}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments={
            'controllers': controllers,
            'laps': LaunchConfiguration('laps').perform(context),
            'track_radii': str(TRACK_RADIUS),
            'track_straight': str(TRACK_STRAIGHT),
            'track_x': str(TRACK_X),
            'track_y': str(TRACK_Y),
            'direction': direction,
            'start_at': 'begin',
            'obstacles': text,
            'session_name': session,
        }.items()),
    ]


# =============================================================================
# [LIFE]   generate_launch_description()
# -----------------------------------------------------------------------------
# Arguments left out on the command line are asked in the terminal first
# (launch_prompts); _include then works out the obstacles and includes run_laps.
# The arguments are declared with an empty default: empty means "not given, ask".
# =============================================================================
def generate_launch_description():
    # Choices of the compare_modes question
    modes = [('false', 'CBF filter for every controller'),
             ('true', 'compare LTV-MPC and NMPC: CBF filter vs obstacles as constraints')]
    return LaunchDescription([
        DeclareLaunchArgument('compare_modes', default_value='',
                              description='true: LTV-MPC and NMPC with the CBF, then with constraints (false)'),
        DeclareLaunchArgument('controllers', default_value='', description='Controllers, in order (all four)'),
        DeclareLaunchArgument('laps', default_value='', description='Laps per controller (3)'),
        DeclareLaunchArgument('start_side', default_value='', description='auto, top or bottom (auto)'),
        DeclareLaunchArgument('sim', default_value='', description='true: rehearsal with the stand-in robot (false)'),
        DeclareLaunchArgument('direction', default_value='ccw', description="'ccw' or 'cw'; obstacles follow it"),
        ask_first('obstacle_lap: R1 lab track with two obstacles (add-on)', [
            Ask('compare_modes', 'Obstacle handling', 'false', modes),
            Ask('controllers', 'Controllers', 'pid,lqr,mpc,nmpc', CHOICES, kind='many', strict=False,
                skip=lambda cfg: cfg.get('compare_modes') == 'true'),
            Ask('laps', 'Laps each', '3', kind='int'),
            Ask('start_side', 'Start side', 'auto', START_SIDES),
            Ask('sim', 'Robot', 'false', SIM),
        ]),
        OpaqueFunction(function=_include),
    ])
