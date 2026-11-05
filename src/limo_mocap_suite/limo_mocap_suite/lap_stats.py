# #############################################################################
# [FILE]   lap_stats.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/lap_stats.py
# ROLE  : Per-lap statistics for any controller: error, time at the steering limit,
#         steering rate. Lap-to-lap learning was removed; docs/FUTURE_lap_learning.md
#         has its design and code.
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

SAT_AT = 0.98         # counted as "at the limit" above this share of max_steer


# =============================================================================
# [CORE]   class LapStats
# -----------------------------------------------------------------------------
# record() every control tick on the path, end_lap() at the end of each lap.
# =============================================================================
class LapStats:
    def __init__(self, max_steer):
        self.max_steer = max_steer
        self._reset_lap()

    def _reset_lap(self):
        self.cte2 = 0.0
        self.cte_max = 0.0
        self.scored = 0
        self.ticks = 0
        self.sat = 0
        self.rate2 = 0.0
        self.ff_peak = 0.0
        self.peak = 0.0
        self.prev = None

    # =========================================================================
    # [CORE]   record(steer_ff, steer_sent, cte, dt, score=True)
    # -------------------------------------------------------------------------
    # One control tick. score=False leaves it out of the lap's error (the start-up
    # transient).
    # =========================================================================
    def record(self, steer_ff, steer_sent, cte, dt, score=True):
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
    # [LOG]    end_lap() -> dict
    # -------------------------------------------------------------------------
    # The lap's numbers, then a fresh start for the next lap.
    # =========================================================================
    def end_lap(self):
        ticks = max(self.ticks, 1)
        stats = {
            'rms_cte': (self.cte2 / max(self.scored, 1)) ** 0.5, 'max_cte': self.cte_max,
            'sat_share': self.sat / ticks,
            'rms_steer_rate': (self.rate2 / ticks) ** 0.5,
            'peak_steer_use': self.peak / self.max_steer,
            'peak_geometry_use': self.ff_peak / self.max_steer,
        }
        self._reset_lap()
        return stats
