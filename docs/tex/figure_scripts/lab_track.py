# Draws docs/figures/lab_track.png: mocap area, R1 lab track, R0.5 add-on, starts, obstacle spots.
# Run from docs/:  python3 tex/figure_scripts/lab_track.py
import sys, numpy as np
sys.path.insert(0, '../src/limo_mocap_suite')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from limo_mocap_suite.track_tools import capsule, LAB_X, LAB_Y
from limo_mocap_suite.lab_obstacles import lab_obstacles
plt.rcParams.update({'font.size': 9})
fig, ax = plt.subplots(figsize=(8.2, 4.6))
ax.add_patch(plt.Rectangle((LAB_X[0], LAB_Y[0]), LAB_X[1]-LAB_X[0], LAB_Y[1]-LAB_Y[0], fc='#EDF2F7', ec='#C0392B', lw=1.4))
ax.add_patch(plt.Rectangle((LAB_X[0]+0.25, LAB_Y[0]+0.25), LAB_X[1]-LAB_X[0]-0.5, LAB_Y[1]-LAB_Y[0]-0.5, fill=False, ec='#C0392B', lw=0.8, ls=':'))
x, y = capsule(4, 1, 3.25, 1.25)
ax.plot(np.append(x, x[0]), np.append(y, y[0]), color='#005A9E', lw=2.2, label='R1 lab track (CCW)')
x5, y5 = capsule(4, 0.5, 3.25, 1.25)
ax.plot(np.append(x5, x5[0]), np.append(y5, y5[0]), color='#7F8C8D', lw=1, ls='--', label='R0.5 add-on track')
for k in range(0, len(x), 180):
    ax.annotate('', xy=(x[(k+8) % len(x)], y[(k+8) % len(y)]), xytext=(x[k], y[k]),
                arrowprops=dict(arrowstyle='-|>', color='#005A9E', lw=1.2))
for d, c, a, lbl in (('ccw', '#E67E22', 0.75, 'obstacles, CCW runs'), ('cw', '#BDC3C7', 0.6, 'obstacles, CW runs')):
    for i, (ox, oy, r) in enumerate(np.array(lab_obstacles(4, 1, 3.25, 1.25, d)).reshape(-1, 3)):
        ax.add_patch(plt.Circle((ox, oy), r, color=c, alpha=a, label=lbl if i == 0 else None))
ax.plot([3.25, 3.25], [0.25, 2.25], '*', color='k', ms=13, label='race starts (middle of a straight)')
ax.plot([1.25], [0.25], 'D', color='#27AE60', ms=7, label='obstacle-run start (CCW)')
ax.annotate('top start', (3.25, 2.25), xytext=(3.35, 2.55), fontsize=8)
ax.annotate('bottom start', (3.25, 0.25), xytext=(3.35, -0.17), fontsize=8)
ax.plot(3.25, 1.25, '+', color='#2D3748'); ax.annotate('centre (3.25, 1.25)', (3.25, 1.25), xytext=(3.25, 1.36), ha='center', fontsize=8, color='#2D3748')
ax.annotate('mocap area x -0.5..7.0, y -0.5..3.0', (-0.45, 2.85), fontsize=8, color='#C0392B')
ax.annotate('0.25 m edge margin', (5.2, -0.42), fontsize=7, color='#C0392B')
ax.annotate('', xy=(7.35, 1.25), xytext=(6.65, 1.25), arrowprops=dict(arrowstyle='-|>', lw=1.5)); ax.text(7.4, 1.22, '+x')
ax.annotate('', xy=(6.95, 1.95), xytext=(6.95, 1.25), arrowprops=dict(arrowstyle='-|>', lw=1.5)); ax.text(6.8, 2.0, '+y')
ax.set_xlim(-0.7, 7.7); ax.set_ylim(-0.7, 3.2); ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.grid(alpha=0.3, lw=0.5)
ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.14), ncol=3, fontsize=7.5, frameon=False)
fig.savefig('figures/lab_track.png', dpi=200, bbox_inches='tight')
