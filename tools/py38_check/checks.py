# #############################################################################
# [FILE]   checks.py
# -----------------------------------------------------------------------------
# FILE  : tools/py38_check/checks.py
# ROLE  : The Foxy-machine check, run by tools/py38_check.sh inside a local
#         Python 3.8 venv with Ubuntu 20.04's library versions. No ROS needed:
#         stubs/ stands in for rclpy and the message packages (imports only).
#
# Checks : 1. every .py in the package, launch/ and tools/ compiles on 3.8
#          2. no Python 3.9+ API is used (removeprefix, dict |, list[int], ...)
#          3. every module of limo_mocap_suite imports
#          4. the maths runs: CBF, LTV / NMPC obstacle solvers, LQR gain (scipy
#             solve_continuous_are), NMPC SLSQP, lap statistics, launch prompts
#          5. the way-to-the-start sweep gives the known counts (568/37/187/48)
#          6. report_lap writes a report from evaluation/ data (if present)
# Exit   : 0 when everything passes, 1 otherwise. Writes only to a temp folder.
# #############################################################################
import glob
import importlib
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import types

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PKG = os.path.join(ROOT, 'src', 'limo_mocap_suite')
sys.path[:0] = [os.path.join(os.path.dirname(os.path.abspath(__file__)), 'stubs'), PKG]
os.environ.setdefault('MPLBACKEND', 'Agg')
results = []


def check(name):
    """Decorator: run the check, record PASS / FAIL with the message."""
    def wrap(fn):
        try:
            note = fn() or ''
            results.append((True, name, note))
        except Exception as e:     # noqa: BLE001  any failure is a FAIL line
            results.append((False, name, f'{type(e).__name__}: {e}'))
        return fn
    return wrap


def sources():
    files = glob.glob(os.path.join(PKG, 'limo_mocap_suite', '*.py')) + glob.glob(os.path.join(PKG, 'launch', '*.py'))
    files += [f for f in glob.glob(os.path.join(ROOT, 'tools', '**', '*.py'), recursive=True) if '/stubs/' not in f]
    return sorted(files)


@check('1. compiles on Python 3.8')
def _compile():
    for f in sources():
        compile(open(f).read(), f, 'exec')
    return f'{len(sources())} files'


@check('2. no Python 3.9+ APIs')
def _new_apis():
    pattern = re.compile(r'\.removeprefix\(|\.removesuffix\(|math\.(lcm|nextafter|ulp)\(|functools\.cache\b|'
                         r'zoneinfo|\.bit_count\(|randbytes\(|^\s*match\s+\S+:\s*$|zip\([^)]*strict=|'
                         r'(:|->)\s*(list|dict|tuple|set)\[|:\s*\w+\s*\|\s*None')
    hits = []
    for f in [f for f in sources() if os.path.abspath(f) != os.path.abspath(__file__)]:   # not this file's patterns
        for n, line in enumerate(open(f), 1):
            code = line.split('#', 1)[0]
            if pattern.search(code):
                hits.append(f'{os.path.relpath(f, ROOT)}:{n}: {line.strip()}')
    if hits:
        raise RuntimeError('; '.join(hits))


@check('3. every module imports')
def _imports():
    names = sorted(os.path.basename(f)[:-3] for f in glob.glob(os.path.join(PKG, 'limo_mocap_suite', '*.py')))
    for n in names:
        importlib.import_module(f'limo_mocap_suite.{n}')
    return f'{len(names)} modules'


@check('4. controller and safety maths')
def _maths():
    import numpy as np
    from limo_mocap_suite import obstacle_cbf, mpc_obstacles as mo, lqr_tracker_node as lqr, nmpc_tracker_node as nm
    from limo_mocap_suite import stats_lap, launch_prompts as lp
    from limo_mocap_suite.track_tools import capsule
    v, steer, _, status = obstacle_cbf.CbfFilter().filter(0.3, 0.0, 0.0, 0.0, 0.0, [(0.6, 0.02, 0.15)])
    assert status == 'CBF_ACTIVE', status
    x, y = capsule(4.0, 1.0, 3.25, 1.25)
    yaw = np.arctan2(np.roll(y, -1) - y, np.roll(x, -1) - x)
    kap = np.gradient(np.unwrap(yaw)) / np.maximum(np.hypot(np.gradient(x), np.gradient(y)), 1e-3)
    wp = np.column_stack((x, y, yaw, np.full_like(x, 0.3), kap))
    traj = types.SimpleNamespace(waypoints=wp, index=lambda i: int(i) % len(wp), ref_offset=0.1)
    ref = mo.sample_reference(traj, 5)
    mo.LinearObstacleMpc(0.2, 0.4084, 0.3).solve(0.02, 0.01, ref, 0.0, [(float(x[40]), float(y[40]) + 0.05, 0.15)])
    mo.NonlinearObstacleMpc(0.2, 0.4084, 0.1, 0.3).solve(float(x[0]), float(y[0]), float(yaw[0]), ref, 0.0,
                                                        [(float(x[8]), float(y[8]) + 0.03, 0.15)])
    k = lqr.LqrTrackerNode._solve_lqr_gain(types.SimpleNamespace(wheelbase=0.2, q_cte=450.0, q_heading=35.0,
                                                                 r_steer=0.5), 0.4, 0.0)
    assert abs(k[0] - 30.0) < 0.05, f'LQR gain {k}'
    cls = [c for c in vars(nm).values() if isinstance(c, type) and hasattr(c, '_solve_nmpc')][0]
    fn = types.SimpleNamespace(wheelbase=0.2, mpc_dt=0.1, N=10, q_cte=40., q_heading=8., rd_steer=2.5, prev_steer=0.0,
                               traj=traj, max_steer=0.4084, u_guess=np.zeros(10), x=float(x[0]), y=float(y[0]),
                               yaw=float(yaw[0]))
    fn._cost_function = types.MethodType(cls._cost_function, fn)
    cls._solve_nmpc(fn, 5)
    s = stats_lap.LapStats(0.4084)
    for i in range(100):
        s.record(0.1, 0.2 * math.sin(i / 5), 0.02 * i / 100, 0.02, score=i > 5)
    s.end_lap()
    q = lp.Ask('c', 'C', 'pid', [('pid', 'PID'), ('lqr', 'LQR')], kind='many')
    assert lp._check(q, '2 pid') == 'pid,lqr'
    return f'LQR K = [{k[0]:.2f}, {k[1]:.2f}]'


@check('5. way-to-the-start sweep')
def _sweep():
    spec = importlib.util.spec_from_file_location('planner_check', os.path.join(ROOT, 'tools', 'planner_check.py'))
    pc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pc)
    import io
    import contextlib
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        pc.sweep()
    text = out.getvalue()
    counts = [int(m) for m in re.findall(r'\s(\d+)\n', text)[:4]]
    assert counts == [568, 37, 187, 48], text
    return '568 / 37 / 187 / 48'


@check('6. report_lap on evaluation/ data')
def _report():
    data = sorted(glob.glob(os.path.join(ROOT, 'evaluation', 'lap_test_*')))
    if not data:
        return 'skipped: no evaluation/lap_test_* folder on this machine'
    tmp = tempfile.mkdtemp()
    try:
        dst = os.path.join(tmp, 'data')
        shutil.copytree(data[0], dst)
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path[:2]))
        r = subprocess.run([sys.executable, '-m', 'limo_mocap_suite.report_lap', dst], env=env,
                           capture_output=True, text=True)
        assert r.returncode == 0, r.stderr[-400:]
        assert os.path.isfile(os.path.join(dst, 'report.md'))
        return os.path.basename(data[0])
    finally:
        shutil.rmtree(tmp)


if __name__ == '__main__':
    import numpy, scipy, matplotlib, yaml
    print(f'Python {sys.version.split()[0]}, numpy {numpy.__version__}, scipy {scipy.__version__}, '
          f'matplotlib {matplotlib.__version__}, PyYAML {yaml.__version__}')
    for ok, name, note in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f'  ({note})' if note else ''))
    failed = sum(not ok for ok, _, _ in results)
    print(f'\n{len(results) - failed} passed, {failed} failed')
    sys.exit(1 if failed else 0)
