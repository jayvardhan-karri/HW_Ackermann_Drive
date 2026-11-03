function [v, delta, dmin, status] = lidar_safety_filter(v, delta, ranges, angles, S)
%LIDAR_SAFETY_FILTER  Speed limiter + repulsive steering (MATLAB port of
%   lidar_safety.LidarSafetyFilter.filter_command).
%   [v, delta, dmin, status] = lidar_safety_filter(v, delta, ranges, angles, S)
%     S = P.safety (stop, caution, repulse, gain)
%   1. Invalid / self-hit readings (<= 0.08 m, NaN, Inf) -> 10 m.
%   2. Front corridor +/-32 deg around 0.5*delta -> dmin.
%   3. Side sectors [10, 75] deg left / right: inside S.repulse add
%        delta -/+= gain * (1/max(d, 0.15) - 1/repulse)   (push away)
%   4. dmin <= stop -> v = 0 ('EMERGENCY_STOP'); dmin < caution -> scale v
%      linearly with a 15 % floor ('CAUTION'); else 'CLEAR'.
%   The returned delta is NOT clipped; the caller clips to max_steer.
if isempty(ranges)
    dmin = inf; status = 'NO_SCAN'; return
end
r = ranges(:); a = angles(:);
r(~isfinite(r) | r <= 0.08) = 10;
front = abs(a - 0.5*delta) <= deg2rad(32);
dmin = 10; if any(front), dmin = min(r(front)); end
left  = a >= deg2rad(10) & a <= deg2rad(75);
right = a <= -deg2rad(10) & a >= -deg2rad(75);
dl = 10; if any(left), dl = min(r(left)); end
dr = 10; if any(right), dr = min(r(right)); end
if dl < S.repulse, delta = delta - S.gain * (1/max(dl, 0.15) - 1/S.repulse); end
if dr < S.repulse, delta = delta + S.gain * (1/max(dr, 0.15) - 1/S.repulse); end
if dmin <= S.stop
    v = 0; status = 'EMERGENCY_STOP';
elseif dmin < S.caution
    v = v * max(0.15, (dmin - S.stop) / (S.caution - S.stop)); status = 'CAUTION';
else
    status = 'CLEAR';
end
end
