function [x, y] = capsule_path(straight, radius, cx, cy)
%CAPSULE_PATH  Running-track shaped closed path (two straights, two half circles).
%   [x, y] = capsule_path(straight, radius, cx, cy), metres, counter-clockwise.
%   Starts in the middle of the lower straight, heading +X. The curvature jumps
%   from 0 to 1/radius where a straight meets a curve: that is on purpose, it is
%   the hardest moment for the steering and what the laps should learn.
ds = 0.005;
h = straight / 2;
n = @(len) max(2, round(len / ds));
s1 = linspace(0, h, n(h))';                     % start -> end of lower straight
a1 = linspace(-pi/2, pi/2, n(pi*radius))';      % right curve
s2 = linspace(h, -h, n(straight))';             % upper straight
a2 = linspace(pi/2, 3*pi/2, n(pi*radius))';     % left curve
s3 = linspace(-h, 0, n(h))';                    % back towards the start
x = [s1; h + radius*cos(a1); s2; -h + radius*cos(a2); s3] + cx;
y = [-radius*ones(size(s1)); radius*sin(a1); radius*ones(size(s2)); radius*sin(a2); -radius*ones(size(s3))] + cy;
keep = [true; hypot(diff(x), diff(y)) > 1e-6];
x = x(keep); y = y(keep);
x(end) = []; y(end) = [];                       % closed: no duplicated end point
end
