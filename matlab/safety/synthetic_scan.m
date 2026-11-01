function [ranges, angles] = synthetic_scan(pose, obstacles)
%SYNTHETIC_SCAN  Fake 240-degree LiDAR by analytic ray-circle intersection
%   (MATLAB port of phasespace_bridge.publish_synthetic_scan).
%   [ranges, angles] = synthetic_scan(pose, obstacles)
%     pose       [x y yaw] of the robot (lab frame)
%     obstacles  [x y r] rows (lab frame)
%   Rays: -2.094 .. +2.094 rad, 0.005825 rad step (719 rays), 0.08 .. 8 m.
%   For each obstacle in the robot frame (lx, ly) and ray angle a:
%     p = |lx sin a - ly cos a| (miss distance), t = lx cos a + ly sin a - sqrt(r^2 - p^2)
%   Keep the nearest hit with t > 0.08 m.
amin = -2.094395; ainc = 0.005825; rmax = 8.0; rmin = 0.08;
n = floor((2*2.094395) / ainc);
angles = amin + (0:n-1)' * ainc;
ranges = rmax * ones(n, 1);
c = cos(pose(3)); s = sin(pose(3));
for k = 1:size(obstacles, 1)
    dx = obstacles(k, 1) - pose(1); dy = obstacles(k, 2) - pose(2); r = obstacles(k, 3);
    lx = dx*c + dy*s; ly = -dx*s + dy*c;
    if hypot(lx, ly) > rmax + r, continue; end
    p = abs(lx*sin(angles) - ly*cos(angles));
    hit = p < r;
    t = lx*cos(angles(hit)) + ly*sin(angles(hit)) - sqrt(r^2 - p(hit).^2);
    idx = find(hit);
    ok = t > rmin;
    ranges(idx(ok)) = min(ranges(idx(ok)), t(ok));
end
end
