#!/usr/bin/env python3
"""Dependency and compatibility check for the LIMO MoCap suite.

Run it on the target machine, in a terminal where ROS 2 is sourced:

    source /opt/ros/foxy/setup.bash        # or humble
    python3 tools/check_deps.py

It reports, without installing anything:
  * OS, Python version and the ROS 2 distro, and whether they match
    (Foxy = Ubuntu 20.04 + Python 3.8, Humble = Ubuntu 22.04 + Python 3.10)
  * every Python module the package imports (ROS and third-party), with its
    version, which part of the package needs it, and the apt package to install
  * library features the code relies on (QoS, SciPy solvers, Matplotlib TkAgg)
  * a syntax check of every package source file with THIS interpreter
  * build tools (colcon, ros2) and a display for the GUIs
Exit code 0 = everything required is present, 1 = something required is missing.

Written for Python 3.6+ so it runs on old systems too.
"""
import importlib
import os
import platform
import py_compile
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG_DIRS = [os.path.join(ROOT, 'src', 'limo_mocap_suite', 'limo_mocap_suite'),
            os.path.join(ROOT, 'src', 'limo_mocap_suite', 'launch')]

DISTRO = os.environ.get('ROS_DISTRO', '')
EXPECTED = {'foxy': ('20.04', (3, 8)), 'galactic': ('20.04', (3, 8)),
            'humble': ('22.04', (3, 10)), 'iron': ('22.04', (3, 10)), 'jazzy': ('24.04', (3, 12))}

results = []   # (status, name, version/detail, needed_by, hint)


def add(status, name, detail, needed_by, hint=''):
    results.append((status, name, detail, needed_by, hint))


def ros_pkg(name):
    return 'ros-{}-{}'.format(DISTRO or '<distro>', name.replace('_', '-'))


def try_import(module, needed_by, hint, required=True, attr_checks=()):
    """Import a module; optionally check attributes ('a.b.c' paths)."""
    try:
        mod = importlib.import_module(module)
    except Exception as exc:  # noqa: BLE001
        add('MISSING' if required else 'WARN', module, type(exc).__name__ + ': ' + str(exc)[:60],
            needed_by, hint)
        return None
    version = getattr(mod, '__version__', '')
    where = getattr(mod, '__file__', '') or ''
    missing = []
    for path in attr_checks:
        obj = mod
        for part in path.split('.'):
            obj = getattr(obj, part, None)
            if obj is None:
                missing.append(path)
                break
    if missing:
        add('MISSING' if required else 'WARN', module, 'lacks ' + ', '.join(missing), needed_by, hint)
    else:
        loc = os.path.dirname(where)
        add('OK', module, version or (loc if len(loc) <= 45 else '...' + loc[-42:]), needed_by, '')
    return mod


def os_release():
    info = {}
    try:
        with open('/etc/os-release') as f:
            for line in f:
                if '=' in line:
                    k, v = line.rstrip().split('=', 1)
                    info[k] = v.strip('"')
    except OSError:
        pass
    return info


def version_tuple(text):
    out = []
    for part in text.split('.'):
        num = ''.join(ch for ch in part if ch.isdigit())
        if not num:
            break
        out.append(int(num))
    return tuple(out)


def main():
    osr = os_release()
    py = sys.version_info[:3]
    print('=' * 78)
    print(' LIMO MoCap suite - dependency check')
    print('=' * 78)
    print(' OS         : {} ({})'.format(osr.get('PRETTY_NAME', platform.platform()), platform.machine()))
    print(' Python     : {} ({})'.format('.'.join(map(str, py)), sys.executable))
    print(' ROS_DISTRO : {}'.format(DISTRO or '(not set - source /opt/ros/<distro>/setup.bash)'))

    # ---------------------------------------------------------- platform match
    if not DISTRO:
        add('MISSING', 'ROS 2 environment', 'ROS_DISTRO not set', 'everything',
            'source /opt/ros/foxy/setup.bash  (or humble)')
    elif DISTRO in EXPECTED:
        ubuntu, pyver = EXPECTED[DISTRO]
        if tuple(py[:2]) != pyver:
            add('MISSING', 'Python for ' + DISTRO,
                'running {}.{}, {} needs {}.{}'.format(py[0], py[1], DISTRO, *pyver), 'every ROS node',
                'run with /usr/bin/python3; rclpy is compiled for Python {}.{} only'.format(*pyver))
        else:
            add('OK', 'Python for ' + DISTRO, '{}.{} matches'.format(*pyver), 'every ROS node')
        if osr.get('VERSION_ID') and osr.get('VERSION_ID') != ubuntu:
            add('WARN', 'Ubuntu for ' + DISTRO, '{} (expected {})'.format(osr.get('VERSION_ID'), ubuntu),
                'binary ROS packages', '')
    else:
        add('WARN', 'ROS distro', DISTRO + ' not tested', 'everything', 'tested on foxy and humble')

    if py < (3, 6):
        add('MISSING', 'Python >= 3.6', '.'.join(map(str, py)), 'f-strings in all files', '')

    # ---------------------------------------------------------- ROS 2 Python packages
    rclpy = try_import('rclpy', 'every node', ros_pkg('rclpy') + ' (part of ros-base)')
    try_import('rclpy.qos', 'bridge, builder, live_plotter (latched obstacle map)', ros_pkg('rclpy'),
               attr_checks=('QoSProfile', 'DurabilityPolicy.TRANSIENT_LOCAL'))
    sig = try_import('rclpy.signals', 'Ctrl-C handling (optional)', '', required=False)
    if rclpy is not None:
        if sig is not None and hasattr(sig, 'SignalHandlerOptions'):
            add('OK', 'SignalHandlerOptions', 'present (Humble+): used by ros_compat.init_ros',
                'clean Ctrl-C stop', '')
        else:
            add('OK', 'SignalHandlerOptions', 'absent (Foxy): ros_compat.init_ros falls back to rclpy.init',
                'clean Ctrl-C stop', '')
    for msg in ('nav_msgs', 'geometry_msgs', 'sensor_msgs', 'std_msgs'):
        try_import(msg + '.msg', 'nodes (topics)', ros_pkg(msg))
    pm = try_import('phasespace_msgs.msg', 'phasespace_bridge only',
                    "the lab's phasespace_msgs package: build it in this workspace or source its overlay")
    if pm is not None:
        try:
            m = pm.Marker()
            fields = [f for f in ('x', 'y', 'z', 'cond') if hasattr(m, f)]
            missing = [f for f in ('x', 'y', 'z') if f not in fields]
            if missing:
                add('MISSING', 'phasespace_msgs/Marker fields', 'lacks ' + ', '.join(missing),
                    'phasespace_bridge.marker_cb', 'adapt marker_cb to the real field names')
            else:
                add('OK', 'phasespace_msgs/Marker fields', ', '.join(fields) +
                    ('' if 'cond' in fields else ' (no cond: validity uses zero position only)'),
                    'phasespace_bridge.marker_cb', '')
            if not hasattr(pm.Markers(), 'markers'):
                add('MISSING', 'phasespace_msgs/Markers.markers', 'field not found',
                    'phasespace_bridge.marker_cb', 'adapt msg.markers[...] to the real field name')
        except Exception as exc:  # noqa: BLE001
            add('WARN', 'phasespace_msgs fields', 'could not inspect: ' + str(exc)[:50], 'bridge', '')
    try_import('launch', 'hw_tracking.launch.py', ros_pkg('launch'), attr_checks=('actions.OpaqueFunction',))
    lr = try_import('launch_ros.actions', 'hw_tracking.launch.py', ros_pkg('launch_ros'))
    if lr is not None:
        import inspect
        params = inspect.signature(lr.Node.__init__).parameters
        if 'executable' not in params:
            add('MISSING', "launch_ros Node(executable=)", 'old launch_ros (Eloquent or older)',
                'hw_tracking.launch.py', 'use Foxy or newer')
    try_import('ament_index_python', 'ros2 run / launch', ros_pkg('ament_index_python'))

    # ---------------------------------------------------------- third-party Python
    np_ = try_import('numpy', 'all nodes', 'python3-numpy')
    if np_ is not None and version_tuple(np_.__version__) < (1, 17):
        add('WARN', 'numpy version', np_.__version__ + ' < 1.17 (tested minimum)', 'all nodes', 'python3-numpy')
    sp = try_import('scipy', 'LQR, NMPC, builder', 'python3-scipy')
    if sp is not None:
        try_import('scipy.linalg', 'lqr_tracker', 'python3-scipy', attr_checks=('solve_continuous_are',))
        try_import('scipy.optimize', 'nmpc_tracker (SLSQP)', 'python3-scipy', attr_checks=('minimize',))
        try_import('scipy.interpolate', 'interactive_builder (Smooth Path)', 'python3-scipy',
                   attr_checks=('splprep', 'splev'))
        if version_tuple(sp.__version__) < (1, 3):
            add('WARN', 'scipy version', sp.__version__ + ' < 1.3 (tested minimum)', 'LQR/NMPC', 'python3-scipy')
    mpl = try_import('matplotlib', 'builder, live_plotter, limo_test_suite', 'python3-matplotlib')
    if mpl is not None:
        if version_tuple(mpl.__version__) < (3, 1):
            add('WARN', 'matplotlib version', mpl.__version__ + ' < 3.1 (tested minimum)', 'GUIs',
                'python3-matplotlib')
        try:
            import inspect
            import matplotlib.animation as anim
            if 'cache_frame_data' not in inspect.signature(anim.FuncAnimation.__init__).parameters:
                add('MISSING', 'FuncAnimation(cache_frame_data)', 'matplotlib too old', 'live_plotter, test suite',
                    'pip3 install --user "matplotlib>=3.1"')
        except Exception:  # noqa: BLE001
            pass
        try:
            importlib.import_module('matplotlib.backends.backend_tkagg')
            add('OK', 'matplotlib TkAgg backend', 'importable', 'GUI windows', '')
        except Exception as exc:  # noqa: BLE001
            add('MISSING', 'matplotlib TkAgg backend', str(exc)[:60], 'GUI windows', 'python3-tk')
    try_import('tkinter', 'GUI windows, builder screen size', 'python3-tk')
    try_import('yaml', 'hw_tracking.launch.py (obstacles / lab_x args)', 'python3-yaml')
    try_import('setuptools', 'colcon build', 'python3-setuptools')

    # ---------------------------------------------------------- tools and display
    for tool, hint in (('colcon', 'python3-colcon-common-extensions'), ('ros2', ros_pkg('ros2cli'))):
        path = shutil.which(tool)
        add('OK' if path else 'MISSING', tool + ' (command)', path or 'not on PATH', 'build / run', hint)
    disp = os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')
    add('OK' if disp else 'WARN', 'display', disp or 'no DISPLAY (GUIs will not open; nodes still run)',
        'builder, live_plotter, test suite', 'run on the desktop or use ssh -X')
    add('OK' if shutil.which('matlab') else 'INFO', 'matlab (optional)',
        shutil.which('matlab') or 'not on PATH', 'matlab/ folder only', '')

    # ---------------------------------------------------------- package syntax with this Python
    bad, n = [], 0
    tmp = tempfile.mkdtemp()
    for d in PKG_DIRS:
        if not os.path.isdir(d):
            continue
        for fn in sorted(os.listdir(d)):
            if fn.endswith('.py'):
                n += 1
                try:
                    py_compile.compile(os.path.join(d, fn), cfile=os.path.join(tmp, fn + 'c'), doraise=True)
                except py_compile.PyCompileError as exc:
                    bad.append('{}: {}'.format(fn, str(exc).splitlines()[-1][:60]))
    if n == 0:
        add('WARN', 'package sources', 'not found next to this script', 'syntax check', '')
    elif bad:
        add('MISSING', 'package syntax ({} files)'.format(n), '; '.join(bad), 'the failing files', '')
    else:
        add('OK', 'package syntax ({} files)'.format(n), 'all compile with Python {}.{}'.format(*py[:2]),
            'all nodes', '')

    # ---------------------------------------------------------- built and sourced?
    try:
        from ament_index_python.packages import get_package_prefix
        pre = get_package_prefix('limo_mocap_suite')
        add('OK', 'limo_mocap_suite (built)', pre if len(pre) <= 45 else '...' + pre[-42:], 'ros2 run', '')
    except Exception:  # noqa: BLE001
        add('INFO', 'limo_mocap_suite (built)', 'not found in the sourced workspace', 'ros2 run',
            'colcon build --packages-select limo_mocap_suite && source install/setup.bash')

    # ---------------------------------------------------------- report
    print('-' * 78)
    w = max(len(r[1]) for r in results)
    for status, name, detail, needed, hint in results:
        print(' [{:<7}] {:<{w}}  {}'.format(status, name, detail, w=w))
        if status != 'OK':
            print(' {:9} {:<{w}}  needed by: {}'.format('', '', needed, w=w))
            if hint:
                print(' {:9} {:<{w}}  fix: {}'.format('', '', hint, w=w))
    missing = [r for r in results if r[0] == 'MISSING']
    warns = [r for r in results if r[0] == 'WARN']
    print('-' * 78)
    apt = sorted({r[4] for r in missing if r[4].startswith(('python3-', 'ros-'))})
    if apt:
        print(' apt packages to install:\n   sudo apt install ' + ' '.join(apt))
    print(' Summary: {} OK, {} missing, {} warnings'.format(
        sum(r[0] == 'OK' for r in results), len(missing), len(warns)))
    if not missing:
        print(' All required dependencies are present.')
    print('=' * 78)
    return 1 if missing else 0


if __name__ == '__main__':
    sys.exit(main())
