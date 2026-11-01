function [xs, ys] = smooth_path(x, y, window, closed_tol)
%SMOOTH_PATH  Smooth a hand-drawn stroke (base MATLAB only, no toolbox).
%   [xs, ys] = smooth_path(x, y, window, closed_tol)
%     x, y       raw stroke points (m)
%     window     smoothing window in metres of arc length (e.g. 0.30)
%     closed_tol stroke counts as a closed loop if its ends are closer (m)
%
%   1. Drop points closer than 1 mm (repeated mouse samples).
%   2. Three passes of: re-sample every 1 cm of arc length, then a Gaussian
%      moving average (smoothdata) over `window` metres. Re-sampling between
%      passes keeps the window a true distance even when the raw stroke
%      zig-zags (mouse jitter makes the raw arc length much too long).
%      A closed stroke is padded circularly so the seam is smoothed too.
%   This replaces the Python builder's B-spline (scipy splprep), which would
%   need the Curve Fitting Toolbox in MATLAB.
x = x(:); y = y(:);
keep = [true; hypot(diff(x), diff(y)) > 1e-3];
x = x(keep); y = y(keep);
if numel(x) < 4
    xs = x; ys = y; return
end
closed = hypot(x(end) - x(1), y(end) - y(1)) < closed_tol;
xs = x; ys = y;
for pass = 1:3
    [xs, ys] = resample_arc(xs, ys, 0.01);
    w = max(5, round(window / 0.01));
    if closed
        n = numel(xs); pad = min(w, n - 1);
        xp = smoothdata([xs(end-pad+1:end); xs; xs(1:pad)], 'gaussian', w);
        yp = smoothdata([ys(end-pad+1:end); ys; ys(1:pad)], 'gaussian', w);
        xs = xp(pad+1:pad+n); ys = yp(pad+1:pad+n);
    else
        xs = smoothdata(xs, 'gaussian', w); ys = smoothdata(ys, 'gaussian', w);
    end
end
[xs, ys] = resample_arc(xs, ys, 0.01);
end

function [xu, yu] = resample_arc(x, y, ds)
%RESAMPLE_ARC  Points every ds metres along the polyline (linear interpolation).
s = [0; cumsum(hypot(diff(x), diff(y)))];
keep = [true; diff(s) > 0];
s = s(keep); x = x(keep); y = y(keep);
su = (0:ds:s(end))';
xu = interp1(s, x, su); yu = interp1(s, y, su);
end
