#!/usr/bin/env python3
# #############################################################################
# [FILE]   planner_check.py
# -----------------------------------------------------------------------------
# FILE  : tools/planner_check.py
# ROLE  : Checks the way-to-the-start rule (approach_planner.plan_to_start) without
#         ROS running: draws it from six start poses and sweeps a grid of start
#         poses (every 0.5 m, 8 headings) on the R1 lab track, with the same lab
#         edge rules as run_manager (0.25 m inside, then the lab, then 0.3 m out,
#         else refused). The numbers in the design report come from here.
#
# Usage : source /opt/ros/humble/setup.bash   (nav_msgs is imported by track_tools)
#         python3 tools/planner_check.py            -> docs/figures/fixed_merge_rule.png + table
# #############################################################################
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'limo_mocap_suite'))
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from limo_mocap_suite.approach_planner import plan_to_start  # noqa: E402
from limo_mocap_suite.lab_obstacles import lab_obstacles  # noqa: E402
from limo_mocap_suite.track_tools import LAB_X, LAB_Y, capsule, pick_side  # noqa: E402

RADIUS = 0.6            # approach_radius
EDGES = (0.25, 0.0, -0.3)   # EDGE_MARGIN, lab, -approach_spill
OBSTACLE_MARGIN = 0.5


def track(side, at='middle'):
    x, y = capsule(4.0, 1.0, 3.25, 1.25, start_side=side, start_at=at)
    yaw = np.arctan2(np.roll(y, -1) - y, np.roll(x, -1) - x)
    return np.column_stack((x, y, yaw, np.zeros_like(x), np.zeros_like(x)))


def allowed(obstacles, edge):
    """Same test as run_manager._allowed: area minus edge, never further out than the start, obstacle margin."""
    def check(t):
        out = np.maximum.reduce([LAB_X[0] + edge - t[:, 0], t[:, 0] - LAB_X[1] + edge,
                                 LAB_Y[0] + edge - t[:, 1], t[:, 1] - LAB_Y[1] + edge, np.zeros(len(t))])
        if out.max() > out[0] + 1e-3:
            return False
        for ox, oy, r in obstacles:
            d = np.hypot(t[:, 0] - ox, t[:, 1] - oy)
            if d.min() < min(r + OBSTACLE_MARGIN, d[0]) - 1e-3:
                return False
        return True
    return check


def way(pose, main, obstacles=()):
    for edge in EDGES:
        got = plan_to_start(pose, main, RADIUS, allowed(obstacles, edge))
        if got:
            return got
    return None


def figure(path):
    """Six start poses, 3 x 2 panels, big enough to read on an A4 page."""
    import textwrap
    obs = np.array(lab_obstacles(4.0, 1.0, 3.25, 1.25, 'ccw')).reshape(-1, 3)
    poses = [((1.0, 2.0, 0.0), 0, 'facing away from the top start'),
             ((5.5, 0.6, math.pi), 0, 'facing away from the bottom start'),
             ((6.0, 1.8, math.pi / 2), 0, 'beside the track, facing up'),
             ((1.0, 1.0, math.pi / 2), 0, 'sideways near the end of the loop'),
             ((3.25, 1.25, -math.pi / 2), 0, 'in the middle, facing the bottom'),
             ((5.0, 1.0, math.pi), 1, 'obstacle run (start where the straight begins)')]
    fig, axes = plt.subplots(3, 2, figsize=(12, 9.2))
    for ax, (pose, with_obs, label) in zip(axes.flat, poses):
        o = obs if with_obs else []
        main = track(pick_side('auto', pose[1], 1.25), 'begin' if with_obs else 'middle')
        ax.add_patch(plt.Rectangle((LAB_X[0], LAB_Y[0]), 7.5, 3.5, fc='#F4F6F9', ec='#C0392B', lw=1))
        ax.plot(*np.append(main[:, :2], main[:1, :2], 0).T, '--', color='#2E7D32', lw=1)
        for ox, oy, r in o:
            ax.add_patch(plt.Circle((ox, oy), r, color='#E67E22', alpha=0.75))
        got = way(pose, main, o)
        if got:
            t, j, text = got
            ax.plot(t[:, 0], t[:, 1], color='#D9731A', lw=2.4)
            if j:
                ax.plot(main[j:, 0], main[j:, 1], color='#005A9E', lw=3, alpha=0.55)
            note = text.replace('merge onto the path at ', 'merge at ').replace(' along the path to the start', ' along the track')
        else:
            note = 'refused: robot not moved'
        ax.set_title(label, fontsize=11, fontweight='bold', loc='left')
        ax.text(-0.45, -0.42, textwrap.fill(note, 70), fontsize=8.5, color='#2D3748', va='bottom')
        ax.annotate('', xy=(pose[0] + 0.5 * math.cos(pose[2]), pose[1] + 0.5 * math.sin(pose[2])), xytext=pose[:2],
                    arrowprops=dict(arrowstyle='-|>', lw=2.2, color='k'))
        ax.plot(*main[0, :2], '*', color='k', ms=15)
        ax.set_xlim(-0.6, 7.1); ax.set_ylim(-0.6, 3.1); ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
    fig.tight_layout(h_pad=1.2, w_pad=1.0)
    fig.savefig(path, dpi=170, bbox_inches='tight')


def sweep():
    counts = {'merge onto the lane': 0, 'merge onto the start': 0, 'no smooth merge': 0, 'refused': 0}
    drive, outside = [], 0.0
    for x0 in np.arange(-0.25, 6.8, 0.5):
        for y0 in np.arange(-0.25, 2.8, 0.5):
            for h in np.arange(0.0, 2 * math.pi, math.pi / 4):
                got = way((x0, y0, h), track(pick_side('auto', y0, 1.25)))
                if not got:
                    counts['refused'] += 1
                    continue
                t, j, text = got
                outside = max(outside, LAB_X[0] - t[:, 0].min(), t[:, 0].max() - LAB_X[1],
                              LAB_Y[0] - t[:, 1].min(), t[:, 1].max() - LAB_Y[1])
                key = 'no smooth merge' if text.startswith('no smooth') else 'merge onto the start' if j == 0 else 'merge onto the lane'
                counts[key] += 1
                drive.append(float(text.split('by a ')[1].split(' m')[0]) + float(text.split('then ')[1].split(' m')[0]))
    d = np.array(drive)
    print(f'{sum(counts.values())} start poses')
    for k, v in counts.items():
        print(f'  {k:22s} {v}')
    print(f'  drive to the start: median {np.median(d):.1f} m, 90 % {np.percentile(d, 90):.1f} m, max {d.max():.1f} m')
    print(f'  worst point outside the mocap area: {max(0.0, outside):.2f} m')


if __name__ == '__main__':
    out = os.path.join(ROOT, 'docs', 'figures', 'fixed_merge_rule.png')
    figure(out)
    print(f'figure: {out}')
    sweep()
