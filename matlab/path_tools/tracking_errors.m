function [err, last_idx] = tracking_errors(ref, pose, lookahead, last_idx, P)
%TRACKING_ERRORS  Cross-track / heading error to the path (MATLAB port of
%   TrajectorySubscriber.compute_tracking_errors).
%   [err, last_idx] = tracking_errors(ref, pose, lookahead, last_idx, P)
%     pose      [x y yaw] of the robot
%     lookahead heading-error look-ahead, in waypoints
%     last_idx  nearest index from the previous call (search-window centre)
%
%   1. Nearest waypoint in the window [last_idx - search_back, last_idx + search_fwd]
%      (wrapped or clamped by ref_index); global search if > 1.5 m away.
%   2. Look-ahead waypoint j = ref_index(nearest + lookahead).
%   3. cte  = lateral offset of the nearest point in the path frame
%             (positive = path lies to the robot's LEFT -> steer left).
%      herr = wrap(yaw_nearest - atan(ref_offset*kappa_nearest) - yaw_robot):
%      zero when tracking perfectly. A look-ahead yaw would be non-zero on any
%      curve and force a steady inside offset; the atan term is the angle between
%      the tracked point's travel direction and the robot's yaw.
%   4. Open path: goal = nearest is the last point and the robot is within
%      goal_tol of (or past) it along the final tangent.
%
%   err fields: cte, herr, la (1x5 row [x y yaw v kappa] of the look-ahead
%   point), nearest, goal.
idx = ref_index(ref, (last_idx - P.ref.search_back):(last_idx + P.ref.search_fwd));
d2 = (ref.x(idx) - pose(1)).^2 + (ref.y(idx) - pose(2)).^2;
[dmin, k] = min(d2);
nearest = idx(k);
if dmin > 2.25
    [~, nearest] = min((ref.x - pose(1)).^2 + (ref.y - pose(2)).^2);
end
last_idx = nearest;
j = ref_index(ref, nearest + lookahead);

yr = ref.yaw(nearest);
err.cte  = -sin(yr) * (ref.x(nearest) - pose(1)) + cos(yr) * (ref.y(nearest) - pose(2));
err.herr = wrap_angle(yr - atan(P.ref_offset * ref.kappa(nearest)) - pose(3));
err.la = ref.W(j, :);
err.nearest = nearest;
err.goal = false;
if ~ref.closed && nearest == ref.n
    along = cos(yr) * (pose(1) - ref.x(end)) + sin(yr) * (pose(2) - ref.y(end));
    err.goal = along >= -P.ref.goal_tol;
end
end
