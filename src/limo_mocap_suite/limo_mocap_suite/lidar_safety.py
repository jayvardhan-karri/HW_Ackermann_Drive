# #############################################################################
# [FILE]   lidar_safety.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/lidar_safety.py
# ROLE  : Shared helper: LiDAR speed limiter + repulsive steering (APF).
#
# Not a node. Pure function of (command, scan) -> safe command.
# Identical file exists in the simulation package (limo_gz_sim).
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
# #############################################################################

import math
import numpy as np


# =============================================================================
# [CORE]   class LidarSafetyFilter
# -----------------------------------------------------------------------------
# Post-processes every controller command before it reaches /cmd_vel.
# =============================================================================
class LidarSafetyFilter:
    """Hardware-agnostic LiDAR safety velocity scaler and repulsive steering assist."""

    # =========================================================================
    # [LIFE]   __init__(stop_dist, caution_dist, repulse_dist, repulse_gain)
    # -------------------------------------------------------------------------
    # Stores thresholds: stop 0.28 m, caution 0.85 m, repulse 0.65 m, gain
    #   0.35.
    # =========================================================================
    def __init__(
        self,
        stop_dist: float = 0.28,
        caution_dist: float = 0.85,
        repulse_dist: float = 0.65,
        repulse_gain: float = 0.35,
    ):
        self.stop_dist = stop_dist
        self.caution_dist = caution_dist
        self.repulse_dist = repulse_dist
        self.repulse_gain = repulse_gain

    # =========================================================================
    # [CORE]   filter_command(v_cmd, steer_cmd, ranges, angle_min, angle_increment)
    # -------------------------------------------------------------------------
    # Returns (v_safe, steer_safe, min_front_dist, status).
    # 0. No scan -> pass command through, status 'NO_SCAN'.
    # 1. Clean ranges: NaN/inf/self-hits (<= 0.08 m) -> 10 m.
    # 2. Front corridor: +/-32 deg centred on 0.5*steer -> min_front.
    # 3. Side sectors [10, 75] deg left / right -> min_left, min_right.
    #    Inside repulse_dist: steer += -/+ gain*(1/d - 1/repulse_dist).
    # 4. Speed: <= stop -> 0 ('EMERGENCY_STOP'); < caution -> linear scale
    #    with 15 % floor ('CAUTION'); else unchanged ('CLEAR').
    # =========================================================================
    def filter_command(
        self,
        v_cmd: float,
        steer_cmd: float,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
    ):
        """
        Adjusts (v_cmd, steer_cmd) based on real-time LaserScan ranges.
        Returns: (v_safe, steer_safe, min_front_dist, status_str)
        """
        if ranges is None or len(ranges) == 0:
            return v_cmd, steer_cmd, float('inf'), 'NO_SCAN'

        n = len(ranges)
        angles = angle_min + np.arange(n) * angle_increment

        # Replace NaNs/Infs and ignore self-hits < 0.08m
        clean_r = np.where(np.isfinite(ranges) & (ranges > 0.08), ranges, 10.0)

        # 1. Forward corridor centered slightly toward current steering angle
        corridor_center = 0.5 * steer_cmd
        front_mask = np.abs(angles - corridor_center) <= math.radians(32.0)
        min_front = float(np.min(clean_r[front_mask])) if np.any(front_mask) else 10.0

        # 2. Lateral repulsive field (left: [10 deg, 75 deg], right: [-75 deg, -10 deg])
        left_mask = (angles >= math.radians(10.0)) & (angles <= math.radians(75.0))
        right_mask = (angles <= -math.radians(10.0)) & (angles >= -math.radians(75.0))

        min_left = float(np.min(clean_r[left_mask])) if np.any(left_mask) else 10.0
        min_right = float(np.min(clean_r[right_mask])) if np.any(right_mask) else 10.0

        steer_bias = 0.0
        if min_left < self.repulse_dist:
            # Obstacle on left -> push steering negative (right)
            steer_bias -= self.repulse_gain * (1.0 / max(min_left, 0.15) - 1.0 / self.repulse_dist)
        if min_right < self.repulse_dist:
            # Obstacle on right -> push steering positive (left)
            steer_bias += self.repulse_gain * (1.0 / max(min_right, 0.15) - 1.0 / self.repulse_dist)

        steer_safe = steer_cmd + steer_bias

        # 3. Velocity scaling
        if min_front <= self.stop_dist:
            return 0.0, steer_safe, min_front, 'EMERGENCY_STOP'
        elif min_front < self.caution_dist:
            scale = (min_front - self.stop_dist) / (self.caution_dist - self.stop_dist)
            v_safe = v_cmd * max(0.15, scale)
            return v_safe, steer_safe, min_front, 'CAUTION'

        return v_cmd, steer_safe, min_front, 'CLEAR'
