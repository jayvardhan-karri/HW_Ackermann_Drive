# #############################################################################
# [FILE]   report_lap.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/report_lap.py
# ROLE  : Turns run folders into a report: report.md, summary.csv and charts.
#         Works on a lap test folder or on any folder holding run folders.
#
# Console script : ros2 run limo_mocap_suite report_lap <folder>
#                  (run_laps calls it at the end of a test)
# No ROS; reads each run's telemetry.csv and summary.json.
# Metrics per lap (computed while driving, in stats_lap.py, stored in summary.json):
#   rms_cte / max_cte  RMS and largest distance from the path (m)
#   sat_share          share of ticks with steering at the limit
#   peak_steer_use     largest steering sent / limit;  peak_geometry_use: same for
#                      the steering the path shape itself needs
#   rms_steer_rate     RMS of the steering change per second (rad/s)
#   lap_time, rejoins  seconds per lap, drift recoveries in the lap
# Metrics from the telemetry here: recovery() (peak error and settling distance
# after each curve).
# Charts: tracks.png        driven lines against the path, one colour per lap
#         laps.png          RMS and max error per lap: does each lap get better?
#         error_along.png   error along the path per lap, curves shaded
#         steering.png      steering sent, what the geometry needs, and the limit
#         curvature.png     error and steering against curve tightness
#         recovery.png      how far the error peaks after a curve, how fast it settles
# A folder of radius sub-folders (R1, R0.5, ...) gets one report per radius plus a
# comparison across them at its top level.
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

import csv
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

LAP_COLORS = ['#2f6db5', '#d9731a', '#2fa35a', '#8e44ad', '#555555']
LIMIT_COLOR = '#c62828'
ORDER = {'pid': 0, 'lqr': 1, 'mpc': 2, 'nmpc': 3}
LABEL = {'pid': 'PID', 'lqr': 'LQR', 'mpc': 'LTV-MPC', 'nmpc': 'NMPC'}
SETTLED = 0.02       # error counted as recovered below this (m)
SETTLED_FOR = 0.5    # ... for this far along the path (m)


# =============================================================================
# [UTIL]   load_runs(folder) -> list of dict
# -----------------------------------------------------------------------------
# Every sub-folder with a summary.json (or the folder itself). Adds the telemetry
# columns as numpy arrays, text columns as lists.
# Each dict is summary.json plus 'data' (the columns) and 'dir'. Sorted so the
# plain runs come first, then the 'constraint' runs, each in controller order.
# =============================================================================
def load_runs(folder):
    dirs = [folder] + [os.path.join(folder, d) for d in sorted(os.listdir(folder))]
    runs = []
    for d in dirs:
        js = os.path.join(d, 'summary.json')
        if not os.path.isfile(js):
            continue
        with open(js) as f:
            run = json.load(f)
        with open(os.path.join(d, 'telemetry.csv')) as f:
            rows = list(csv.DictReader(f))
        data = {}
        for key in (rows[0] if rows else {}):
            try:
                data[key] = np.array([float(r[key]) for r in rows])
            except ValueError:
                data[key] = [r[key] for r in rows]
        run['data'] = data
        run['dir'] = d
        runs.append(run)
    runs.sort(key=lambda r: (r.get('obstacle_mode') == 'constraint', ORDER.get(r['controller'], 9)))
    return runs


# =============================================================================
# [UTIL]   _on_track(run, lap=None) -> bool mask or None  (helper: used only in this file)
# -----------------------------------------------------------------------------
# Which telemetry rows are in the TRACK phase (not the approach), optionally
# only those of one lap. None: the run stopped before its first tick.
# =============================================================================
def _on_track(run, lap=None):
    if 'phase' not in run['data']:     # run stopped before its first tick
        return None
    d = run['data']
    if not d:
        return np.zeros(0, dtype=bool)
    mask = np.array([p == 'TRACK' for p in d['phase']])
    return mask if lap is None else mask & (d['lap'] == lap)


# =============================================================================
# [UTIL]   _grid(n) -> (rows, cols)  (helper: used only in this file)
# -----------------------------------------------------------------------------
# Subplot layout for n runs: two columns when there is more than one run.
# =============================================================================
def _grid(n):
    cols = 2 if n > 1 else 1
    return (n + cols - 1) // cols, cols


# =============================================================================
# [UTIL]   _label(run) -> text  (helper: used only in this file)
# -----------------------------------------------------------------------------
# Controller name for legends and titles; "constraints" is added when the run used
# the 'constraint' obstacle mode.
# =============================================================================
def _label(run):
    name = LABEL.get(run['controller'], run['controller'])
    return name + ' constraints' if run.get('obstacle_mode') == 'constraint' else name


# =============================================================================
# [UI]     _shade_curves(ax, run, mask)  (helper: used only in this file)
# -----------------------------------------------------------------------------
# Greys the parts of the path where |curvature| > 0.3 1/m, so the error plot
# shows which stretches are curves.
# =============================================================================
def _shade_curves(ax, run, mask):
    d = run['data']
    order = np.argsort(d['s'][mask])
    s, k = d['s'][mask][order], np.abs(d['kappa'][mask][order])
    ax.fill_between(s, 0, 1, where=k > 0.3, transform=ax.get_xaxis_transform(), color='0.92', zorder=0, label='curve')


# =============================================================================
# [LOG]    plot_tracks(runs, out)  -> tracks.png
# -----------------------------------------------------------------------------
# One panel per run: the way to the path (dotted), each lap in its own colour,
# the reference path (dashed) and the known obstacles as red discs. X / Y in m.
# =============================================================================
def plot_tracks(runs, out):
    rows, cols = _grid(len(runs))
    fig, axes = plt.subplots(rows, cols, figsize=(6.5 * cols, 4.6 * rows), squeeze=False)
    for ax, run in zip(axes.flat, runs):
        d = run['data']
        approach = np.array([p == 'APPROACH' for p in d['phase']]) if d else np.zeros(0, dtype=bool)
        if approach.any():
            ax.plot(d['x'][approach], d['y'][approach], ':', color='0.45', lw=1.5, label='way to the path')
        for i, lap in enumerate(sorted(set(d['lap'][_on_track(run)]))):
            m = _on_track(run, lap)
            ax.plot(d['x'][m], d['y'][m], lw=1.3, color=LAP_COLORS[i % 5], label=f'lap {int(lap)}')
        m = _on_track(run)
        if m.any():
            ax.plot(d['x_ref'][m], d['y_ref'][m], 'k--', lw=0.8, label='path')
        flat = run.get('obstacles', [])
        for i in range(0, len(flat) - 2, 3):
            ax.add_patch(plt.Circle((flat[i], flat[i + 1]), flat[i + 2], color='#c62828', alpha=0.7, zorder=3))
        ax.set_aspect('equal')
        ax.grid(alpha=0.4)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f"{_label(run)}: {run['result']}")
        ax.legend(fontsize=8)
    for ax in axes.flat[len(runs):]:
        ax.axis('off')
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


# =============================================================================
# [LOG]    plot_laps(runs, out)  -> laps.png
# -----------------------------------------------------------------------------
# Two bar charts, one bar per lap and run: RMS error and largest error (cm).
# Shows whether the controller gets better, or worse, from lap to lap.
# =============================================================================
def plot_laps(runs, out):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
    n_laps = max([len(r['laps']) for r in runs] + [1])
    width = 0.8 / n_laps
    x = np.arange(len(runs))
    for lap in range(n_laps):
        rms = [100 * r['laps'][lap]['rms_cte'] if lap < len(r['laps']) else np.nan for r in runs]
        peak = [100 * r['laps'][lap]['max_cte'] if lap < len(r['laps']) else np.nan for r in runs]
        off = (lap - (n_laps - 1) / 2.0) * width
        axes[0].bar(x + off, rms, width, color=LAP_COLORS[lap % 5], label=f'lap {lap + 1}')
        axes[1].bar(x + off, peak, width, color=LAP_COLORS[lap % 5])
    for ax, title in zip(axes, ('RMS error per lap (cm)', 'Largest error per lap (cm)')):
        ax.set_xticks(x)
        ax.set_xticklabels([_label(r) for r in runs])
        ax.set_title(title)
        ax.grid(axis='y', alpha=0.4)
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


# =============================================================================
# [LOG]    plot_error_along(runs, out)  -> error_along.png
# -----------------------------------------------------------------------------
# Per run: signed error (cm) against distance along the path (m), one line per
# lap, curves shaded. Points are sorted by path distance before drawing.
# =============================================================================
def plot_error_along(runs, out):
    rows, cols = _grid(len(runs))
    fig, axes = plt.subplots(rows, cols, figsize=(6.5 * cols, 3.6 * rows), squeeze=False)
    for ax, run in zip(axes.flat, runs):
        d = run['data']
        m = _on_track(run)
        if not m.any():
            ax.set_title(f'{_label(run)}: no lap driven')
            continue
        _shade_curves(ax, run, m)
        for i, lap in enumerate(sorted(set(d['lap'][m]))):
            ml = _on_track(run, lap)
            o = np.argsort(d['s'][ml])
            ax.plot(d['s'][ml][o], 100 * d['cte'][ml][o], lw=1.1, color=LAP_COLORS[i % 5], label=f'lap {int(lap)}')
        ax.axhline(0, color='0.3', lw=0.8)
        ax.grid(alpha=0.4)
        ax.set_xlabel('distance along the path (m)')
        ax.set_ylabel('error (cm)')
        ax.set_title(_label(run))
        ax.legend(fontsize=8)
    for ax in axes.flat[len(runs):]:
        ax.axis('off')
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


# =============================================================================
# [LOG]    plot_steering(runs, out)  -> steering.png
# -----------------------------------------------------------------------------
# Per run, last lap: steering sent and the steering the path geometry needs
# (steer_ff), both in deg, against the steering limit (red). The title gives
# the peak use and the share of the lap spent at the limit.
# =============================================================================
def plot_steering(runs, out):
    rows, cols = _grid(len(runs))
    fig, axes = plt.subplots(rows, cols, figsize=(6.5 * cols, 3.6 * rows), squeeze=False)
    for ax, run in zip(axes.flat, runs):
        d = run['data']
        m = _on_track(run)
        if not m.any():
            ax.set_title(f'{_label(run)}: no lap driven')
            continue
        last = max(d['lap'][m])
        ml = _on_track(run, last)
        o = np.argsort(d['s'][ml])
        s = d['s'][ml][o]
        ax.plot(s, np.degrees(d['steer_rad'][ml][o]), color=LAP_COLORS[0], lw=1.1, label='steering sent')
        ax.plot(s, np.degrees(d['steer_ff'][ml][o]), 'k--', lw=1.2, label='needed by the geometry')
        limit = np.degrees(run['max_steer'])
        ax.axhline(limit, color=LIMIT_COLOR, lw=1.5, label='limit')
        ax.axhline(-limit, color=LIMIT_COLOR, lw=1.5)
        stats = run['laps'][-1] if run['laps'] else {}
        ax.set_title(f"{_label(run)}, lap {int(last)}: peak {100 * stats.get('peak_steer_use', 0):.0f}% of limit, "
                     f"at the limit {100 * stats.get('sat_share', 0):.1f}% of the lap")
        ax.set_xlabel('distance along the path (m)')
        ax.set_ylabel('steering (deg)')
        ax.grid(alpha=0.4)
        ax.legend(fontsize=8, loc='lower right')
    for ax in axes.flat[len(runs):]:
        ax.axis('off')
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


# =============================================================================
# [LOG]    plot_curvature(runs, out)  -> curvature.png
# -----------------------------------------------------------------------------
# Error and steering use against how tight the curve is.
# HOW: rows are grouped into bins of |curvature| (1/m). Per bin with more than
# 20 rows: RMS error (left) and 95th percentile of |steering| / limit (right).
# The red line is the curvature the steering limit allows.
# =============================================================================
def plot_curvature(runs, out):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
    edges = np.array([-0.01, 0.1, 0.4, 0.8, 1.2, 1.6, 2.0, 3.0, 6.0])
    centers = 0.5 * (edges[:-1] + edges[1:])
    for i, run in enumerate(runs):
        d = run['data']
        m = _on_track(run)
        if not m.any():
            continue
        k = np.abs(d['kappa'][m])
        err = np.abs(d['cte'][m])
        use = np.abs(d['steer_rad'][m]) / run['max_steer']
        idx = np.digitize(k, edges) - 1
        ok = [b for b in range(len(centers)) if (idx == b).sum() > 20]
        axes[0].plot(centers[ok], [100 * np.sqrt(np.mean(err[idx == b] ** 2)) for b in ok], '-o', label=_label(run), color=LAP_COLORS[i % 5])
        axes[1].plot(centers[ok], [100 * np.percentile(use[idx == b], 95) for b in ok], '-o', label=_label(run), color=LAP_COLORS[i % 5])
    if runs:
        for ax in axes:
            ax.axvline(runs[0]['curvature_limit'], color=LIMIT_COLOR, lw=1.5, label='steering limit')
            ax.set_xlabel('curve tightness |curvature| (1/m)')
            ax.grid(alpha=0.4)
            ax.legend(fontsize=8)
    axes[0].set_title('RMS error by curve tightness (cm)')
    axes[1].set_title('Steering used, 95th percentile (% of limit)')
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


# =============================================================================
# [CORE]   recovery(run) -> dict
# -----------------------------------------------------------------------------
# For each time the robot leaves a curve: the largest error from the curve's start
# until the next curve, and the distance from the curve's end until the error stays
# below SETTLED for SETTLED_FOR metres. peak_cm: mean largest error. settle_m: mean
# distance over the exits that settled. unsettled: exits that never settled before
# the next curve. exits: how many were measured.
# =============================================================================
def recovery(run):
    d = run['data']
    m = _on_track(run)
    empty = {'peak_cm': float('nan'), 'settle_m': float('nan'), 'unsettled': 0, 'exits': 0}
    if m is None or not m.any():
        return empty
    # HOW: a curve is |kappa| > 0.1 1/m. starts / ends = rows where it begins / ends.
    curve = np.abs(d['kappa'][m]) > 0.1
    err = np.abs(d['cte'][m])
    s = d['s'][m]
    starts = np.flatnonzero(curve[1:] & ~curve[:-1]) + 1
    ends = np.flatnonzero(~curve[1:] & curve[:-1]) + 1
    peaks, settles, unsettled = [], [], 0
    for end in ends:
        before = starts[starts < end]
        after = starts[starts > end]
        if len(before) == 0 or len(after) == 0:
            continue
        begin, stop = before[-1], after[0]
        peaks.append(err[begin:stop].max())
        # distance driven since the curve exit, from the positions: s itself wraps to 0
        # at the start line, which sits in the middle of a straight
        x, y = d['x'][m][end:stop], d['y'][m][end:stop]
        dist = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
        settled = None
        for i in np.flatnonzero(err[end:stop] < SETTLED):    # first point from which it stays settled
            later = (dist >= dist[i]) & (dist < dist[i] + SETTLED_FOR)
            if dist[-1] - dist[i] >= SETTLED_FOR and err[end:stop][later].max() < SETTLED:
                settled = dist[i]
                break
        if settled is None:
            unsettled += 1
        else:
            settles.append(settled)
    if not peaks:
        return empty
    return {'peak_cm': 100.0 * float(np.mean(peaks)), 'settle_m': float(np.mean(settles)) if settles else float('nan'),
            'unsettled': unsettled, 'exits': len(peaks)}


# =============================================================================
# [UTIL]   _metres(value) -> text  (helper: used only in this file)
# -----------------------------------------------------------------------------
# Settling distance for the table: 'never' for NaN (it never settled), else metres.
# =============================================================================
def _metres(value):
    return 'never' if value != value else f'{value:.2f}'


# =============================================================================
# [LOG]    plot_recovery(runs, out)  -> recovery.png
# -----------------------------------------------------------------------------
# Two bar charts from recovery(): mean largest error per curve (cm) and the
# distance to settle below SETTLED after a curve (m), one bar per run.
# =============================================================================
def plot_recovery(runs, out):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
    names = [_label(r) for r in runs]
    rec = [recovery(r) for r in runs]
    axes[0].bar(names, [x['peak_cm'] for x in rec], color=LAP_COLORS[0])
    axes[0].set_title('Largest error per curve, mean (cm)')
    axes[1].bar(names, [x['settle_m'] for x in rec], color=LAP_COLORS[1])
    axes[1].set_title(f'Distance to settle below {100 * SETTLED:.0f} cm after a curve (m)')
    for ax in axes:
        ax.grid(axis='y', alpha=0.4)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


# =============================================================================
# [LOG]    write_tables(runs, folder)
# -----------------------------------------------------------------------------
# summary.csv (one row per lap) and report.md (results, then the charts).
# report.md has a results table (per run: result, time to reach the path, rejoins,
# RMS first / last lap, largest error, share at the steering limit), a recovery
# table, the reason for each run that did not FINISH, and the chart list.
# =============================================================================
def write_tables(runs, folder):
    keys = ['rms_cte', 'max_cte', 'sat_share', 'peak_steer_use', 'peak_geometry_use', 'rms_steer_rate',
            'lap_time', 'rejoins']
    with open(os.path.join(folder, 'summary.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['controller', 'result', 'lap', 'partial'] + keys)
        for r in runs:
            for lap in r['laps']:
                w.writerow([r['controller'], r['result'], lap['lap'], lap.get('partial', False)] + [lap[k] for k in keys])
    lines = ['# Lap test report', '', f'Folder: {os.path.basename(os.path.abspath(folder))}', '']
    lines += ['| Controller | Result | Way to the path (s) | Rejoins | RMS first lap (cm) | RMS last lap (cm) | Largest (cm) | At steering limit, last lap | Peak steering, last lap |',
              '|---|---|---|---|---|---|---|---|---|']
    for r in runs:
        laps = r['laps']
        if laps:
            peak = max(x['max_cte'] for x in laps)
            lines.append(f"| {_label(r)} | {r['result']} | {r['approach_time_s']:.1f} | {r['rejoins']} | "
                         f"{100 * laps[0]['rms_cte']:.2f} | {100 * laps[-1]['rms_cte']:.2f} | {100 * peak:.1f} | "
                         f"{100 * laps[-1]['sat_share']:.1f}% | {100 * laps[-1]['peak_steer_use']:.0f}% |")
        else:
            lines.append(f"| {_label(r)} | {r['result']} | {r['approach_time_s']:.1f} | {r['rejoins']} | - | - | - | - | - |")
    lines += ['', f'| Controller | Largest error per curve, mean (cm) | Distance to settle below {100 * SETTLED:.0f} cm (m) | Curve exits that never settled |',
              '|---|---|---|---|']
    for r in runs:
        x = recovery(r)
        lines.append(f"| {_label(r)} | {x['peak_cm']:.1f} | {_metres(x['settle_m'])} | {x['unsettled']} of {x['exits']} |")
    stopped = [r for r in runs if r['result'] != 'FINISHED']
    lines += [''] + [f"**{_label(r)} {r['result']}:** {r['reason']}" for r in stopped] + ['']
    lines += ['A lap counted as partial was cut short by the stop.', '']
    for name, title in (('tracks', 'Driven lines'), ('laps', 'Does each lap get better?'),
                        ('error_along', 'Where along the path the error is'),
                        ('recovery', 'Which controller recovers best after a curve?'),
                        ('steering', 'Is the controller using its steering?'),
                        ('curvature', 'How curve tightness affects error and steering')):
        lines += [f'## {title}', '', f'![{title}]({name}.png)', '']
    with open(os.path.join(folder, 'report.md'), 'w') as f:
        f.write('\n'.join(lines))


# =============================================================================
# [CORE]   generate(folder)  -> charts, summary.csv, report.md
# -----------------------------------------------------------------------------
# Builds the whole report for a folder.
# HOW: if the folder holds radius sub-folders (R1, R0.5, ...), make a report for
# each (recursion) and then compare_radii() across them. Otherwise load the runs,
# draw the six charts for runs that logged telemetry, and write_tables().
# Stops with a message if there is no run folder.
# =============================================================================
def generate(folder):
    runs = load_runs(folder)
    subs = [d for d in sorted(os.listdir(folder)) if d.startswith('R') and os.path.isdir(os.path.join(folder, d))]
    groups = [(d, load_runs(os.path.join(folder, d))) for d in subs]
    groups = [g for g in groups if g[1]]
    if groups:
        for label, _runs in groups:
            generate(os.path.join(folder, label))
        groups.sort(key=lambda g: -float(g[0][1:]))
        compare_radii(folder, groups)
        return
    if not runs:
        raise SystemExit(f'No run folders with summary.json in {folder}')
    driven = [r for r in runs if 'phase' in r['data']]   # an aborted run may have no telemetry rows
    if driven:
        plot_tracks(driven, os.path.join(folder, 'tracks.png'))
        plot_laps(driven, os.path.join(folder, 'laps.png'))
        plot_error_along(driven, os.path.join(folder, 'error_along.png'))
        plot_steering(driven, os.path.join(folder, 'steering.png'))
        plot_curvature(driven, os.path.join(folder, 'curvature.png'))
        plot_recovery(driven, os.path.join(folder, 'recovery.png'))
    write_tables(runs, folder)
    print(f"Report written: {os.path.join(folder, 'report.md')}")


# =============================================================================
# [LOG]    compare_radii(folder, groups)
# -----------------------------------------------------------------------------
# Top-level report of a test with several radii: one table and one chart across them.
# groups: list of (radius label, runs), widest radius first. Writes comparison.csv,
# comparison.png (RMS last lap, error per curve, settling distance) and report.md.
# =============================================================================
def compare_radii(folder, groups):
    lines = ['# Lap test report: all radii', '', f'Folder: {os.path.basename(os.path.abspath(folder))}', '',
             '| Radius | Controller | Result | Laps done | RMS last lap (cm) | Largest (cm) | At steering limit, last lap | '
             'Largest per curve (cm) | To settle (m) | Never settled |', '|---|---|---|---|---|---|---|---|---|---|']
    series = {}
    with open(os.path.join(folder, 'comparison.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['radius', 'controller', 'result', 'laps', 'rms_last', 'max_cte', 'sat_last', 'peak_curve', 'settle_m', 'unsettled', 'exits'])
        for label, runs in groups:
            for r in runs:
                x = recovery(r)
                laps = r['laps']
                rms = laps[-1]['rms_cte'] if laps else float('nan')
                peak = max(lap['max_cte'] for lap in laps) if laps else float('nan')
                sat = laps[-1]['sat_share'] if laps else float('nan')
                w.writerow([label, r['controller'], r['result'], len(laps), rms, peak, sat, x['peak_cm'], x['settle_m'], x['unsettled'], x['exits']])
                lines.append(f"| {label} | {_label(r)} | {r['result']} | {len(laps)} | {100 * rms:.2f} | {100 * peak:.1f} | "
                             f"{100 * sat:.1f}% | {x['peak_cm']:.1f} | {_metres(x['settle_m'])} | {x['unsettled']} of {x['exits']} |")
                series.setdefault(_label(r), []).append((label, rms, x['peak_cm'], x['settle_m']))
    labels = [g[0] for g in groups]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))
    width = 0.8 / max(len(series), 1)
    for i, (ctrl, rows) in enumerate(series.items()):     # runs are already in order
        at = {row[0]: row for row in rows}
        pos = np.arange(len(labels)) + (i - (len(series) - 1) / 2.0) * width
        nan = float('nan')
        for ax, col, scale in zip(axes, (1, 2, 3), (100.0, 1.0, 1.0)):
            ax.bar(pos, [scale * at[n][col] if n in at else nan for n in labels], width,
                   color=LAP_COLORS[i % 5], label=ctrl)
    for ax, title in zip(axes, ('RMS error, last lap (cm)', 'Largest error per curve (cm)', 'Distance to settle after a curve (m)')):
        ax.set_xticks(np.arange(len(labels)))
        ax.set_xticklabels(labels)
        ax.set_title(title)
        ax.grid(axis='y', alpha=0.4)
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(folder, 'comparison.png'), dpi=110)
    plt.close(fig)
    lines += ['', '![Comparison](comparison.png)', '', 'Each radius has its own report: ' +
              ', '.join(f'[{n}]({n}/report.md)' for n in labels), '']
    with open(os.path.join(folder, 'report.md'), 'w') as f:
        f.write('\n'.join(lines))
    print(f"Report written: {os.path.join(folder, 'report.md')}")


# =============================================================================
# [LIFE]   main()  -> console script report_lap
# -----------------------------------------------------------------------------
# Takes one argument, the folder, and calls generate(). Usage text otherwise.
# =============================================================================
def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: report_lap <folder with run folders>')
    generate(sys.argv[1])


if __name__ == '__main__':
    main()
