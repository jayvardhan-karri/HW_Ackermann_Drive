# Lap-to-lap learning: design note for later

Removed from the code on 2026-11-05 to keep the controllers simple. Nothing in the package uses it now. This note has what is needed to put it back: the idea, where it plugs in, what was measured, and the full code, because it was never committed to git.

## What it does

On a closed path the same errors come back at the same places, so the robot can learn them.

1. **Feedback-error learning.** At arc length `s`, `f(s) = steer_ctrl(s) - atan(L*kappa(s))` is how far the controller had to steer beyond the geometry. At the end of a lap, a share of that becomes feed-forward for the next lap, shifted ahead by the steering delay:
   ```
   D[j+1](s) = D[j](s) + alpha * f[j](s + lead)
   steer_sent = steer_ctrl + D[j](s)
   ```
   It needs no knowledge of the controller's gains, so one implementation serves all four.
2. **Safeguards.** No learning when the lap's RMS error is below a deadband. A lap more than 10 % worse than the best so far restores the best table and halves `alpha`.
3. **Tuning once per lap**, using the share of the lap at the steering limit:
   - at the limit and the geometry needs over 80 % of it: the limit is real, multiply speed by 0.9;
   - at the limit but the geometry needs far less: chatter, halve the error weight (`q_scale`);
   - otherwise, RMS above target: multiply the error weight by 1.25.

Values used: bin 0.05 m, `alpha` 0.6, deadband 3 mm, `lead` 0.10 m, smoothing 0.25 m, correction limit 0.25 rad, RMS target 4 mm, limit share 5 %, `q_scale` 0.1 to 3, speed factor at least 0.6.

## Where it plugs in

`lap_stats.py` already keeps the per-lap numbers learning needs. To add learning back:

| Place | Change |
|---|---|
| `lap_stats.py` | Replace with `LapAdapter` below, or subclass `LapStats` and add `correction()`, the table and the tuning step |
| `run_manager.shape()` | On the path, add `adapter.correction(idx)` to the steering and multiply speed by `adapter.v_scale` |
| `run_manager.command()` | Pass `steer_ctrl` (before learning) to `record()` |
| `run_manager`, parameters | `adapt` (default false); call `on_tune(q_scale)` at the end of each lap |
| Each controller | Remember its own output, not the learned sum, as "previous steering". On `on_tune`: LQR divides `r_steer` by `q_scale` and clears its gain cache; LTV-MPC and NMPC multiply `q_cte` |
| CSV, report | Columns `steer_ctrl`, `steer_learned`; `q_scale_next`, `v_scale_next` per lap |
| Launch files | An `adapt` argument |

## What was measured

Capsule with 3 m straights and 1 m curves, 3 laps, RMS error on lap 3 in cm, off → on. MATLAB robots are invented. `max_steer` was 0.48869 rad in all of these, so repeat them at 0.4084 rad.

| Test | PID | LQR | LTV-MPC | NMPC |
|---|---|---|---|---|
| MATLAB, ideal robot | 0.64 → 0.30 | 0.13 → 0.13 | 0.05 → 0.05 | 0.26 → 0.26 |
| MATLAB, imperfect robot (gain 0.89, trim 0.02 rad, lag 0.12 s, rate 1.5 rad/s, delay 60 ms, 1 mm noise) | 0.78 → 0.35 | 0.59 → 0.24 | 0.26 → 0.26 | 0.45 → 0.40 |
| MATLAB, very imperfect robot (0.80, 0.05 rad, 0.20 s, 1.0 rad/s, 100 ms, 2 mm) | 1.19 → 0.47 | 1.17 → 1.26 | 0.54 → 0.31 | 1.03 → 0.46 |
| Gazebo, 0.30 m/s, one run each | 0.65 → 0.45 | 0.51 → 0.49 | 0.61 → 0.49 | 1.13 → 1.02 |

Largest error in Gazebo, cm: PID 2.6 → 1.5, LQR 1.7 → 1.7, LTV-MPC 2.5 → 1.2, NMPC 4.1 → 2.6.

LQR on the very imperfect robot only improved from lap 4 (0.52, then 0.35 cm at lap 5). Over 8 laps nothing ran away.

**Tried and rejected:** estimating the curvature gain online from measured yaw rate. Lag and chatter corrupted the estimate: with LQR on the imperfect robot it settled at 0.6 against a true 0.89, and errors got worse.

## Before trusting it

- Test on the real LIMO with learning off first: the gain depends on how repeatable its errors are.
- Learning cannot fix a curve tighter than the steering limit (about 0.46 m radius).
- Restart learning if the path, speed or tyres change: the table belongs to one path at one speed.
- Do not learn during a CBF detour; that effort is not a model error. The old code did not check this because it was tested with no obstacles.

## Full code

### Python: `lap_adapter.py`

```python
# #############################################################################
# [FILE]   lap_adapter.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/lap_adapter.py
# ROLE  : Lap-to-lap adaptation and per-lap statistics for any controller. Same
#         method and values as matlab/lap_study/lap_adapter.m, where it was tested.
#
# 1. Feedback-error learning: per arc-length bin, how much the controller steered
#    beyond the geometric feed-forward. At the end of a lap a share of that
#    becomes feed-forward for the next lap, shifted ahead by the steering delay:
#        D(s) <- D(s) + alpha * fb(s + lead)
#    It needs no knowledge of the controller's gains.
#    No learning below ilc_deadband; a lap worse than the best so far goes back to
#    the best table and halves alpha.
# 2. Tuning, once per lap:
#      at the limit and the geometry needs nearly all of it -> slow down (v_scale)
#      at the limit although the geometry needs far less    -> chatter, q_scale down
#      error above target with steering to spare            -> q_scale up
#
# Not a node; plain math, no ROS.
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
# #############################################################################

import numpy as np

BIN = 0.05            # arc-length bin of the learned table (m)
ALPHA = 0.6           # share of the feedback effort learned per lap
ILC_DEADBAND = 0.003  # no learning below this RMS error (m)
LEAD = 0.10           # learn this far ahead: steering delay * speed (m)
SMOOTH = 0.25         # smoothing of the learned table (m)
D_LIMIT = 0.25        # largest learned correction (rad)
RMS_TARGET = 0.004    # below this RMS error nothing is tuned (m)
SAT_HIGH = 0.05       # share of a lap at the steering limit that needs action
Q_RANGE = (0.1, 3.0)  # weight multiplier limits
V_MIN = 0.6           # smallest speed multiplier
SAT_AT = 0.98         # counted as "at the limit" above this share of max_steer


# =============================================================================
# [CORE]   class LapAdapter
# -----------------------------------------------------------------------------
# learn=False keeps the statistics and changes nothing (the baseline).
# =============================================================================
class LapAdapter:
    def __init__(self, n_points, length, max_steer, learn=True):
        self.n_points = n_points
        self.max_steer = max_steer
        self.learn = learn
        self.nb = max(8, int(round(length / BIN)))
        self.lead = int(round(LEAD / BIN))
        self.smooth = max(1, int(round(SMOOTH / BIN)))
        self.table = np.zeros(self.nb)
        self.best_table = self.table.copy()
        self.best_rms = float('inf')
        self.alpha = ALPHA
        self.q_scale = 1.0
        self.v_scale = 1.0
        self._reset_lap()

    def _reset_lap(self):
        self.fb_sum = np.zeros(self.nb)
        self.count = np.zeros(self.nb)
        self.cte2 = 0.0
        self.cte_max = 0.0
        self.scored = 0
        self.ticks = 0
        self.sat = 0
        self.rate2 = 0.0
        self.ff_peak = 0.0
        self.peak = 0.0
        self.prev = None

    def _bin(self, idx):
        return min(self.nb - 1, int(idx * self.nb / self.n_points))

    # =========================================================================
    # [CORE]   correction(idx) -> rad
    # -------------------------------------------------------------------------
    # Learned steering to add at waypoint idx.
    # =========================================================================
    def correction(self, idx):
        return float(self.table[self._bin(idx)]) if self.learn else 0.0

    # =========================================================================
    # [CORE]   record(idx, steer_ctrl, steer_ff, steer_sent, cte, dt, score=True)
    # -------------------------------------------------------------------------
    # One control tick. score=False learns from it but leaves it out of the lap's
    # error (the start-up transient).
    # =========================================================================
    def record(self, idx, steer_ctrl, steer_ff, steer_sent, cte, dt, score=True):
        b = self._bin(idx)
        self.fb_sum[b] += steer_ctrl - steer_ff
        self.count[b] += 1
        self.ff_peak = max(self.ff_peak, abs(steer_ff))
        self.peak = max(self.peak, abs(steer_sent))
        self.ticks += 1
        if score:
            self.cte2 += cte * cte
            self.cte_max = max(self.cte_max, abs(cte))
            self.scored += 1
        if abs(steer_sent) >= SAT_AT * self.max_steer:
            self.sat += 1
        if self.prev is not None:
            self.rate2 += ((steer_sent - self.prev) / max(dt, 1e-3)) ** 2
        self.prev = steer_sent

    # =========================================================================
    # [CORE]   end_lap() -> dict
    # -------------------------------------------------------------------------
    # Lap statistics, then the learning and tuning steps above.
    # =========================================================================
    def end_lap(self):
        ticks = max(self.ticks, 1)
        rms = (self.cte2 / max(self.scored, 1)) ** 0.5
        stats = {
            'rms_cte': rms, 'max_cte': self.cte_max,
            'sat_share': self.sat / ticks,
            'rms_steer_rate': (self.rate2 / ticks) ** 0.5,
            'peak_steer_use': self.peak / self.max_steer,
            'peak_geometry_use': self.ff_peak / self.max_steer,
        }
        if self.learn:
            if rms > 1.1 * self.best_rms:
                self.table = self.best_table.copy()
                self.alpha /= 2.0
            elif rms > ILC_DEADBAND:
                self.best_rms = rms
                self.best_table = self.table.copy()
                self._learn()
            if stats['sat_share'] > SAT_HIGH and self.ff_peak > 0.8 * self.max_steer:
                self.v_scale = max(V_MIN, self.v_scale * 0.9)
            elif stats['sat_share'] > SAT_HIGH:
                self.q_scale = max(Q_RANGE[0], self.q_scale * 0.5)
            elif rms > RMS_TARGET:
                self.q_scale = min(Q_RANGE[1], self.q_scale * 1.25)
        stats['q_scale_next'] = self.q_scale
        stats['v_scale_next'] = self.v_scale
        self._reset_lap()
        return stats

    def _learn(self):
        seen = self.count > 0
        if not seen.any():
            return
        fb = np.zeros(self.nb)
        fb[seen] = self.fb_sum[seen] / self.count[seen]
        if not seen.all():      # bins not visited: take from neighbours
            i = np.flatnonzero(seen)
            fb[~seen] = np.interp(np.flatnonzero(~seen), i, fb[seen], period=self.nb)
        self.table = self.table + self.alpha * np.roll(fb, -self.lead)
        w = self.smooth
        padded = np.concatenate((self.table[-w:], self.table, self.table[:w]))
        self.table = np.convolve(padded, np.ones(w) / w, mode='same')[w:-w]
        self.table = np.clip(self.table, -D_LIMIT, D_LIMIT)
```

### MATLAB: `matlab/lap_study/lap_adapter.m`

```matlab
function varargout = lap_adapter(cmd, varargin)
%LAP_ADAPTER  Lap-to-lap adaptation for any tracking controller.
%   A = lap_adapter('init', ref, P, mode)    mode: struct with logical fields .ilc .tune
%   A = lap_adapter('tick', A, idx, d_ctrl, d_ff, d_sent, cte, dt, score)
%                                  score false: learn from this tick but leave it out
%                                  of the lap's error (start-up transient)
%   d = lap_adapter('steer', A, idx, d_ctrl) steering to send for controller output d_ctrl
%   d = lap_adapter('internal', A, idx, d_sent)  inverse: what the controller "applied"
%   [A, stats] = lap_adapter('end_lap', A, P)
%
%   1. Feedback-error learning (.ilc): per arc-length bin, how much the controller
%      had to steer beyond the feed-forward, fb(s) = d_ctrl - atan(L*kappa). At the
%      end of a lap a share of that becomes feed-forward for the next lap, shifted
%      ahead by the steering delay: D(s) <- D(s) + alpha*fb(s + lead). This needs
%      no knowledge of the controller's gains, so it works for all of them.
%      Safeguards: no learning below ilc_deadband (nothing worth learning, it only adds
%      noise); if a lap comes out worse than the best so far, go back to the best table
%      and halve alpha.
%   2. Per-lap tuning (.tune), in this order:
%        at the limit and the geometry needs nearly all of it: slow down;
%        at the limit although the geometry needs far less: chatter, WEAKEN the weight;
%        error above target with authority left: STRENGTHEN the error weight.
%      Returns A.q_scale and A.v_scale for the caller.
switch cmd
    case 'init'
        ref = varargin{1}; P = varargin{2}; mode = varargin{3};
        A.mode = mode; A.n = ref.n; A.nb = max(8, round(ref.length / P.adapt.bin));
        A.bin_of = @(i) min(A.nb, floor((i - 1) / ref.n * A.nb) + 1);
        A.D = zeros(A.nb, 1); A.fb_sum = zeros(A.nb, 1); A.count = zeros(A.nb, 1);
        A.ff_peak = 0; A.q_scale = 1; A.alpha = P.adapt.alpha; A.best = inf; A.best_D = A.D; A.v_scale = 1; A.max_steer = P.max_steer;
        A.lead = round(P.adapt.lead / P.adapt.bin); A.smooth = max(1, round(P.adapt.smooth / P.adapt.bin));
        A.cte2 = 0; A.ticks = 0; A.sat = 0; A.rate2 = 0; A.prev = 0;
        varargout{1} = A;
    case 'steer'
        A = varargin{1}; idx = varargin{2}; d = varargin{3};
        if A.mode.ilc, d = d + A.D(A.bin_of(idx)); end
        varargout{1} = min(max(d, -A.max_steer), A.max_steer);
    case 'internal'
        A = varargin{1}; idx = varargin{2}; d = varargin{3};
        if A.mode.ilc, d = d - A.D(A.bin_of(idx)); end
        varargout{1} = d;
    case 'tick'
        A = varargin{1}; [idx, d_ctrl, d_ff, d_sent, cte, dt, score] = varargin{2:8};
        b = A.bin_of(idx);
        A.fb_sum(b) = A.fb_sum(b) + (d_ctrl - d_ff); A.count(b) = A.count(b) + 1;
        A.ff_peak = max(A.ff_peak, abs(d_ff));
        if score, A.cte2 = A.cte2 + cte^2; A.ticks = A.ticks + 1; end
        A.sat = A.sat + (abs(d_sent) >= 0.98 * A.max_steer);
        A.rate2 = A.rate2 + ((d_sent - A.prev) / dt)^2; A.prev = d_sent;
        varargout{1} = A;
    case 'end_lap'
        A = varargin{1}; P = varargin{2};
        stats.rms = sqrt(A.cte2 / max(A.ticks, 1));
        tick_all = max(sum(A.count), 1);
        stats.sat = A.sat / tick_all;
        stats.rate = sqrt(A.rate2 / tick_all);
        if A.mode.ilc && stats.rms > 1.1 * A.best
            A.D = A.best_D; A.alpha = A.alpha / 2;
        elseif A.mode.ilc && stats.rms > P.adapt.ilc_deadband
            A.best = stats.rms; A.best_D = A.D;
            fb = A.fb_sum ./ max(A.count, 1);
            miss = A.count == 0;
            if any(miss) && ~all(miss)      % bins not visited: take from neighbours
                i = find(~miss);
                fb(miss) = interp1([i - A.nb; i; i + A.nb], [fb(i); fb(i); fb(i)], find(miss));
            end
            fb = circshift(fb, -A.lead);
            A.D = A.D + A.alpha * fb;
            A.D = cyclic_mean(A.D, A.smooth);
            A.D = min(max(A.D, -P.adapt.d_limit), P.adapt.d_limit);
        end
        if A.mode.tune
            if stats.sat > P.adapt.sat_high && A.ff_peak > 0.8 * A.max_steer
                A.v_scale = max(P.adapt.v_min, A.v_scale * 0.9);
            elseif stats.sat > P.adapt.sat_high
                A.q_scale = max(P.adapt.q_min, A.q_scale * 0.5);
            elseif stats.rms > P.adapt.rms_target
                A.q_scale = min(P.adapt.q_max, A.q_scale * 1.25);
            end
        end
        A.fb_sum(:) = 0; A.count(:) = 0; A.cte2 = 0; A.ticks = 0; A.sat = 0; A.rate2 = 0; A.ff_peak = 0;
        varargout = {A, stats};
    otherwise
        error('lap_adapter:cmd', 'Unknown command %s', cmd);
end
end

function y = cyclic_mean(x, w)
k = ones(w, 1) / w;
y = conv([x(end-w+1:end); x; x(1:w)], k, 'same');
y = y(w+1:end-w);
end
```
