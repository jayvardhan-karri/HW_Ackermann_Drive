# #############################################################################
# [FILE]   lap_test.launch.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/launch/lap_test.launch.py
# ROLE  : The real test drive: bridge + live view (with KILL) + lap_test. Each
#         controller drives the robot to the start of a capsule track, does its laps
#         and is logged; a report with charts is written at the end.
#
# Usage : ros2 launch limo_mocap_suite lap_test.launch.py
#         ros2 launch limo_mocap_suite lap_test.launch.py controllers:=lqr,nmpc laps:=5
#         ros2 launch limo_mocap_suite lap_test.launch.py track_radii:=1.0,0.5,0.4
#         guard:=true turns on the drift guard (extension)
#         ros2 launch limo_mocap_suite lap_test.launch.py sim:=true     # rehearsal, no lab
# Log   : evaluation/lap_test_<stamp>/ (report.md, charts, one folder per controller)
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
# #############################################################################
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from limo_mocap_suite.obstacle_cbf import parse_marker_groups, parse_obstacles
from limo_mocap_suite.robot_topics import DEFAULT_ROBOT_NS, robot_remappings

FLOATS = ('v_nominal', 'track_straight', 'track_x', 'track_y', 'start_delay', 'ref_offset',
          'cbf_max_steer', 'approach_radius', 'approach_speed', 'slow_cte', 'rejoin_cte', 'abort_cte')


# =============================================================================
# [LIFE]   _nodes(context)
# -----------------------------------------------------------------------------
# Resolves the arguments once and returns: the bridge or, with sim:=true, sim_limo;
# the live view; and lap_test, which starts the controllers one after another.
# =============================================================================
def _nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context).strip()  # noqa: E731
    ns = arg('robot_ns')
    remap = robot_remappings(ns)
    cmd_topic = dict(remap).get('/cmd_vel', '/cmd_vel')
    imu_topic = dict(remap).get('/imu', '/imu')
    lab_x = [float(v) for v in yaml.safe_load(arg('lab_x'))]
    lab_y = [float(v) for v in yaml.safe_load(arg('lab_y'))]
    nodes = []
    if arg('sim').lower() == 'true':
        nodes.append(Node(package='limo_mocap_suite', executable='sim_limo', output='screen',
                          parameters=[{'x': float(arg('sim_x')), 'y': float(arg('sim_y')), 'yaw': float(arg('sim_yaw'))}],
                          remappings=remap))
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
    test = {key: float(arg(key)) for key in FLOATS}
    test.update(controllers=arg('controllers'), laps=int(arg('laps')),
                guard=arg('guard').lower() == 'true', track_radii=arg('track_radii'),
                log_dir=arg('log_dir'), lab_x=lab_x, lab_y=lab_y, cmd_topic=cmd_topic, imu_topic=imu_topic)
    nodes.append(Node(package='limo_mocap_suite', executable='lap_test', output='screen',
                      emulate_tty=True, parameters=[test], remappings=remap))
    return nodes


# =============================================================================
# [LIFE]   generate_launch_description()
# =============================================================================
def generate_launch_description():
    defaults = (
        ('controllers', 'pid,lqr,mpc,nmpc', 'Controllers to run, in order'),
        ('laps', '3', 'Laps per controller'),
        ('guard', 'false', 'Extension: slow, rejoin and stop on drift'),
        ('v_nominal', '0.30', 'Cruise speed on the track (m/s)'),
        ('track_straight', '3.0', 'Length of each straight (m)'),
        ('track_radii', '1.0,0.5,0.4', 'Curve radii to test (m): inside the limit, at it, beyond it'),
        ('track_x', '3.75', 'Track centre X (m)'),
        ('track_y', '2.5', 'Track centre Y (m)'),
        ('start_delay', '5.0', 'Seconds of warning before the robot moves'),
        ('lab_x', '[-0.5, 8.0]', 'Lab X extent; the guard stops the robot outside it'),
        ('lab_y', '[-1.0, 6.0]', 'Lab Y extent; the guard stops the robot outside it'),
        ('view', 'behind', "Live view orientation: 'behind' or 'topdown'"),
        ('robot_ns', DEFAULT_ROBOT_NS, 'LIMO topic namespace (none = no namespace)'),
        ('ref_offset', '0.10', 'Marker midpoint ahead of the rear axle (m)'),
        ('cbf_max_steer', '0.0', 'Steering limit for the CBF (rad); 0 = max_steer'),
        ('approach_radius', '0.60', 'Turning radius used to reach the track (m)'),
        ('approach_speed', '0.25', 'Speed on the way to the track (m/s)'),
        ('slow_cte', '0.15', 'Guard: slow down above this error (m)'),
        ('rejoin_cte', '0.35', 'Guard: plan a way back above this error (m)'),
        ('abort_cte', '1.00', 'Guard: stop above this error (m)'),
        ('obstacles', '', 'Fixed cylinders "[x, y, r, ...]" (default: none)'),
        ('obstacle_markers', '', '4 rim markers per cylinder (default: none)'),
        ('log_dir', 'evaluation', 'Where the test folder is created'),
        ('sim', 'false', 'true: rehearse with sim_limo instead of the bridge'),
        ('sim_x', '1.0', 'sim_limo start X (m)'),
        ('sim_y', '0.5', 'sim_limo start Y (m)'),
        ('sim_yaw', '0.0', 'sim_limo start heading (rad)'),
    )
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in defaults]
        + [OpaqueFunction(function=_nodes)])
