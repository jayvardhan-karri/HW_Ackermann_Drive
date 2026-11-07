# #############################################################################
# [FILE]   run_laps.launch.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/launch/run_laps.launch.py
# ROLE  : The real test drive: bridge + live view (with KILL) + run_laps. Each
#         controller drives the robot to the start of a capsule track, does its laps
#         and is logged; a report with charts is written at the end.
#         The lab entry points include this file with their own defaults:
#           record_lap.launch.py    one controller, 3 laps, R1
#           compare_lap.launch.py   all four in turn, R1
#           obstacle_lap.launch.py add-on: same with two fixed obstacles
#           r05_lap.launch.py      add-on: R0.5 track (track_radii 0.5)
# Asked : nothing. This is the engine; it asks no questions and uses the defaults
#         below. The entry files above ask for the crucial values (launch_prompts)
#         and pass them in. Command-line values (name:=value) are never asked.
#
# Usage : ros2 launch limo_mocap_suite run_laps.launch.py
#         ros2 launch limo_mocap_suite run_laps.launch.py controllers:=lqr,nmpc laps:=5
#         ros2 launch limo_mocap_suite run_laps.launch.py track_radii:=1.0,0.5,0.4
#         guard:=true turns on the drift guard (extension)
#         ros2 launch limo_mocap_suite run_laps.launch.py sim:=true     # rehearsal, no lab
# Log   : <workspace>/evaluation/<session_name>_<stamp>/ (report.md, charts, one folder per controller)
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
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from limo_mocap_suite.obstacle_cbf import parse_marker_groups, parse_obstacles
from limo_mocap_suite.robot_topics import DEFAULT_ROBOT_NS, robot_remappings

from limo_mocap_suite.track_tools import LAB_X, LAB_Y, TRACK_STRAIGHT, TRACK_X, TRACK_Y

# Arguments that become float parameters of run_laps
FLOATS = ('v_nominal', 'track_straight', 'track_x', 'track_y', 'start_delay', 'buffer_time', 'ref_offset',
          'cbf_max_steer', 'approach_radius', 'approach_speed', 'approach_spill', 'slow_cte', 'rejoin_cte', 'abort_cte')


# =============================================================================
# [LIFE]   _nodes(context) -> [Node, ...]  (helper of generate_launch_description)
# -----------------------------------------------------------------------------
# Runs at launch time (OpaqueFunction) so the argument values can be read.
# Resolves the arguments once and returns: the bridge (PhaseSpace markers ->
# /hardware_odom) or, with sim:=true, sim_limo; the live view; and run_laps,
# which starts the controllers one after another.
# robot_ns remaps /cmd_vel and /imu into the robot's namespace (robot_topics).
# =============================================================================
def _nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context).strip()  # noqa: E731  (value of an argument)
    ns = arg('robot_ns')
    remap = robot_remappings(ns)
    cmd_topic = dict(remap).get('/cmd_vel', '/cmd_vel')
    imu_topic = dict(remap).get('/imu', '/imu')
    lab_x = [float(v) for v in yaml.safe_load(arg('lab_x'))]
    lab_y = [float(v) for v in yaml.safe_load(arg('lab_y'))]
    nodes = []
    # Odometry source: the stand-in robot (rehearsal) or the real mocap bridge
    if arg('sim').lower() == 'true':
        sim = {'x': float(arg('sim_x')), 'y': float(arg('sim_y')), 'yaw': float(arg('sim_yaw'))}
        obstacles = parse_obstacles(arg('obstacles'))
        if obstacles:
            sim['obstacles'] = [float(v) for v in obstacles]
        nodes.append(Node(package='limo_mocap_suite', executable='sim_limo', output='screen',
                          parameters=[sim], remappings=remap))
    else:
        params = {}
        obstacles = parse_obstacles(arg('obstacles'))
        if obstacles:
            params['obstacles'] = obstacles
        if parse_marker_groups(arg('obstacle_markers')):
            params['obstacle_markers'] = arg('obstacle_markers')
        nodes.append(Node(package='limo_mocap_suite', executable='phasespace_bridge', output='screen',
                          emulate_tty=True, parameters=[params]))
    nodes.append(Node(package='limo_mocap_suite', executable='live_plotter', output='screen',
                      parameters=[{'rezero_odom': False, 'view': arg('view'), 'lab_x': lab_x, 'lab_y': lab_y}],
                      remappings=remap))
    # Parameters of run_laps: the float arguments plus the text/flag ones
    test = {key: float(arg(key)) for key in FLOATS}
    test.update(controllers=arg('controllers'), laps=int(arg('laps')),
                guard=arg('guard').lower() == 'true', track_radii=arg('track_radii'),
                start_side=arg('start_side'), direction=arg('direction'), start_at=arg('start_at'), session_name=arg('session_name'),
                obstacle_mode=arg('obstacle_mode'),
                log_dir=arg('log_dir'), lab_x=lab_x, lab_y=lab_y, cmd_topic=cmd_topic, imu_topic=imu_topic)
    nodes.append(Node(package='limo_mocap_suite', executable='run_laps', output='screen',
                      emulate_tty=True, parameters=[test], remappings=remap))
    return nodes


# =============================================================================
# [LIFE]   generate_launch_description()
# -----------------------------------------------------------------------------
# Declares every argument with its default (name, default, help text) and then
# calls _nodes. Empty-string defaults mean "none".
# =============================================================================
def generate_launch_description():
    defaults = (
        ('controllers', 'pid,lqr,mpc,nmpc', 'Controllers to run, in order'),
        ('laps', '3', 'Laps per controller'),
        ('guard', 'false', 'Extension: slow, rejoin and stop on drift'),
        ('v_nominal', '0.30', 'Cruise speed on the track (m/s)'),
        ('track_straight', str(TRACK_STRAIGHT), 'Length of each straight (m)'),
        ('track_radii', '1.0', 'Curve radii (m); the radius study is 1.0,0.5,0.4'),
        ('track_x', str(TRACK_X), 'Track centre X (m)'),
        ('track_y', str(TRACK_Y), 'Track centre Y (m)'),
        ('start_side', 'auto', "Race start straight: 'auto' (top, or bottom if the robot is nearer it), 'top', 'bottom'"),
        ('direction', 'ccw', "'ccw' (left turns) or 'cw' (right turns)"),
        ('start_at', 'middle', "Start at the 'middle' of the straight or where it 'begin's"),
        ('buffer_time', '10.0', 'Seconds stopped between two controllers'),
        ('session_name', 'run_laps', 'Test folder name: <session_name>_<stamp>'),
        ('obstacle_mode', 'cbf', "LTV-MPC / NMPC: 'cbf' filter or 'constraint' in the solver"),
        ('start_delay', '5.0', 'Seconds of warning before the robot moves'),
        ('lab_x', str(list(LAB_X)), 'Mocap area X; the track must fit, the guard stops outside it'),
        ('lab_y', str(list(LAB_Y)), 'Mocap area Y; the track must fit, the guard stops outside it'),
        ('view', 'behind', "Live view orientation: 'behind' or 'topdown'"),
        ('robot_ns', DEFAULT_ROBOT_NS, 'LIMO topic namespace (none = no namespace)'),
        ('ref_offset', '0.10', 'Marker midpoint ahead of the rear axle (m)'),
        ('cbf_max_steer', '0.0', 'Steering limit for the CBF (rad); 0 = max_steer'),
        ('approach_radius', '0.60', 'Turning radius used to reach the track (m)'),
        ('approach_speed', '0.25', 'Speed on the way to the track (m/s)'),
        ('approach_spill', '0.30', 'Way to the start may go this far past the lab edge, else the robot is not moved (m)'),
        ('slow_cte', '0.15', 'Guard: slow down above this error (m)'),
        ('rejoin_cte', '0.35', 'Guard: plan a way back above this error (m)'),
        ('abort_cte', '1.00', 'Guard: stop above this error (m)'),
        ('obstacles', '', 'Fixed cylinders "[x, y, r, ...]" (default: none)'),
        ('obstacle_markers', '', '4 rim markers per cylinder (default: none)'),
        ('log_dir', '', "Where the test folder is created; '' = <workspace>/evaluation"),
        ('sim', 'false', 'true: rehearse with sim_limo instead of the bridge'),
        ('sim_x', '1.0', 'sim_limo start X (m)'),
        ('sim_y', '0.5', 'sim_limo start Y (m)'),
        ('sim_yaw', '0.0', 'sim_limo start heading (rad)'),
    )
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in defaults]
        + [OpaqueFunction(function=_nodes)])
