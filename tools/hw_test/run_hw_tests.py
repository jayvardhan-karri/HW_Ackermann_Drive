#!/usr/bin/env python3
"""Rehearses the hardware stack with no robot and no mocap: stand-in robot (sim_limo) -> fake markers (fake_mocap.py)
-> real phasespace_bridge -> real hw_check or controller. Everything is built and logged outside the repo.

    python3 tools/hw_test/run_hw_tests.py                 # all cases, about 4 minutes
    python3 tools/hw_test/run_hw_tests.py normal mirror   # some of them

Run it with ROS 2 sourced. src/phasespace_msgs is a STUB with only the fields the bridge reads, used because the real
package is not on this machine. Never source it on a machine that has the real one. COLCON_IGNORE keeps a normal
colcon build from picking it up.

Expected: normal -> OK to run tracking, with warnings for trim, left vs right, right radius and no command timeout
(sim_limo is imperfect on purpose). mirror, swap, nolisten -> a FAIL naming the fault. glitch -> dropped frames reported,
RUN FINISHED, rc 124 (the controller stays up after a run, so the timeout ends it).
"""
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
WORK = os.path.expanduser('~/.cache/limo_hw_test')
ENV = dict(os.environ, ROS_DOMAIN_ID='87', ROS_LOCALHOST_ONLY='1', MPLBACKEND='Agg')
CASES = {'normal': ('normal', '/cmd_vel'), 'mirror': ('mirror', '/cmd_vel'), 'swap': ('swap', '/cmd_vel'),
         'nolisten': ('normal', '/other')}


def shell(cmd, log=None, **kw):
    full = f'source {WORK}/install/setup.bash && exec {cmd}'
    out = open(log, 'w') if log else subprocess.DEVNULL
    return subprocess.Popen(['bash', '-c', full], env=ENV, stdout=out, stderr=subprocess.STDOUT,
                            start_new_session=True, **kw)


def stop(*procs):
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for p in procs:
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(2.0)


def background(name, mode, cmd_topic, x, y):
    sim = shell(f'ros2 run limo_mocap_suite sim_limo --ros-args -r /hardware_odom:=/sim_truth '
                f'-r /cmd_vel:={cmd_topic} -p x:={x} -p y:={y}', f'{WORK}/logs/{name}.sim')
    fake = shell(f'python3 {HERE}/fake_mocap.py {mode}', f'{WORK}/logs/{name}.fake')
    time.sleep(2.0)
    return sim, fake


def main():
    os.makedirs(f'{WORK}/logs', exist_ok=True)
    subprocess.run(['bash', '-c', f'cd {WORK} && colcon build --symlink-install --base-paths {HERE}/src {REPO}/src '
                    '--packages-select phasespace_msgs limo_mocap_suite'], check=True, stdout=subprocess.DEVNULL)
    names = sys.argv[1:] or list(CASES) + ['glitch']
    for name in names:
        log = f'{WORK}/logs/{name}.log'
        if name == 'glitch':
            procs = background(name, 'glitch', '/cmd_vel', 2.5, 1.5)
            cmd = ('timeout 90 ros2 launch limo_mocap_suite hw_tracking.launch.py controller:=pid robot_ns:=none '
                   'builder:=false plotter:=false track_radius:=1.0 track_straight:=1.0 laps:=1 start_delay:=1.0 '
                   f'log_dir:={WORK}/out/glitch')
        else:
            procs = background(name, CASES[name][0], CASES[name][1], 2.0, 2.0)
            cmd = (f'timeout 120 ros2 launch limo_mocap_suite hw_check.launch.py robot_ns:=none drive:=true '
                   f'log_dir:={WORK}/out/{name}')
        code = shell(cmd, log).wait()
        stop(*procs)
        print(f'--- {name}: rc {code}. Log: {log}', flush=True)
        keep = ('PASS', 'WARN', 'FAIL', 'INFO', 'OK to', 'do not run', 'Incomplete', 'Dropped', 'LAP ', 'RUN ')
        for line in open(log, errors='ignore'):
            if any(k in line for k in keep):
                print('  ', line.rstrip()[:170])


if __name__ == '__main__':
    main()
