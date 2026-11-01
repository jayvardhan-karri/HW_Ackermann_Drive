function ref = build_reference(x, y, P)
%BUILD_REFERENCE  Waypoint table used by every controller (MATLAB port of
%   trajectory_subscriber.path_cb).
%   ref = build_reference(x, y, P)
%
%   1. Re-sample the path every P.ref.ds metres of arc length.
%   2. Closed loop if the ends are closer than P.ref.closed_tol (then the
%      duplicated end point is dropped and indices wrap around).
%   3. Heading yaw = atan2(dy, dx), signed curvature
%        kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^1.5   (finite differences)
%      then a light Gaussian smoothing of kappa over P.ref.kappa_window.
%   4. Speed profile v = v_nominal / (1 + speed_gain*|kappa|).
%
%   ref fields: x, y, yaw, v, kappa (column vectors), W = [x y yaw v kappa],
%   n, ds, closed, length.
x = x(:); y = y(:);
keep = [true; hypot(diff(x), diff(y)) > 1e-4];
x = x(keep); y = y(keep);
closed = hypot(x(end) - x(1), y(end) - y(1)) < P.ref.closed_tol;
if closed      % make the loop seam continuous before re-sampling
    x(end+1) = x(1); y(end+1) = y(1);
end
s = [0; cumsum(hypot(diff(x), diff(y)))];
su = (0:P.ref.ds:s(end))';
xr = interp1(s, x, su); yr = interp1(s, y, su);
if closed && hypot(xr(end) - xr(1), yr(end) - yr(1)) < 0.5*P.ref.ds
    xr(end) = []; yr(end) = [];
end

if closed      % circular differences so the seam has no artefact
    dx = (circshift(xr, -1) - circshift(xr, 1)) / 2;
    dy = (circshift(yr, -1) - circshift(yr, 1)) / 2;
else
    dx = gradient(xr); dy = gradient(yr);
end
dsv = hypot(dx, dy); dsv(dsv == 0) = 1e-5;
tx = dx ./ dsv; ty = dy ./ dsv;
if closed
    ddx = (circshift(tx, -1) - circshift(tx, 1)) / 2 ./ dsv;
    ddy = (circshift(ty, -1) - circshift(ty, 1)) / 2 ./ dsv;
else
    ddx = gradient(tx) ./ dsv; ddy = gradient(ty) ./ dsv;
end
yaw = atan2(dy, dx);
kappa = (tx .* ddy - ty .* ddx) ./ ((tx.^2 + ty.^2).^1.5 + 1e-9);
% Light 10 cm Gaussian smoothing: second differences on a 1 cm grid amplify
% any residual ripple (the steering servo could not follow it anyway).
wk = max(3, round(P.ref.kappa_window / P.ref.ds));
if closed
    pad = min(wk, numel(kappa) - 1);
    kp = smoothdata([kappa(end-pad+1:end); kappa; kappa(1:pad)], 'gaussian', wk);
    kappa = kp(pad+1:end-pad);
else
    kappa = smoothdata(kappa, 'gaussian', wk);
end
v = P.v_nominal ./ (1 + P.ref.speed_gain * abs(kappa));

ref.x = xr; ref.y = yr; ref.yaw = yaw; ref.v = v; ref.kappa = kappa;
ref.W = [xr yr yaw v kappa];
ref.n = numel(xr); ref.ds = P.ref.ds; ref.closed = closed;
ref.length = (ref.n - ~closed) * P.ref.ds;
end
