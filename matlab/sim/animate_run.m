function animate_run(ref, log, P, speedup)
%ANIMATE_RUN  Play back one simulated run on the lab grid.
%   animate_run(ref, log, P)          real time
%   animate_run(ref, log, P, 4)       4x faster
%   Shows the reference, the growing trail, the robot glyph (arrow M1 -> M0)
%   and a status line (time, speed, steering, cte). Close the window to stop.
if nargin < 4, speedup = 1; end
fig = light_figure('Name', ['LIMO animation - ' log.label], 'Color', 'w', 'Position', [100 80 1100 850]);
ax = axes(fig);
style_lab_axes(ax, P, ['Animation: ' log.label]);
th = linspace(0, 2*pi, 60);
for k = 1:size(P.obstacles, 1)
    o = P.obstacles(k, :);
    fill(ax, o(1) + o(3)*cos(th), o(2) + o(3)*sin(th), [1 0.5 0.4], 'FaceAlpha', 0.6, ...
         'EdgeColor', [0.6 0 0], 'DisplayName', 'Obstacle');
end
plot(ax, ref.x, ref.y, 'g--', 'LineWidth', 2, 'DisplayName', 'Reference');
trail = plot(ax, nan, nan, '-', 'Color', [1 0.55 0], 'LineWidth', 2.5, 'DisplayName', 'Trail');
g = robot_glyph(ax);
info = text(ax, P.lab_x(1) + 0.1, P.lab_y(2) - 0.1, '', 'VerticalAlignment', 'top', ...
            'FontName', 'monospace', 'FontSize', 12, 'BackgroundColor', [1 1 1 0.8]);
legend(ax, 'Location', 'northeast');
frame_dt = 0.05;
t0 = tic; i = 1; n = numel(log.t);
while i <= n && isvalid(fig)
    tsim = toc(t0) * speedup;
    while i < n && log.t(i) < tsim, i = i + 1; end
    set(trail, 'XData', log.x(1:i), 'YData', log.y(1:i));
    robot_glyph(ax, g, log.x(i), log.y(i), log.yaw(i));
    info.String = sprintf('t = %5.1f s   v = %.2f m/s   steer = %+5.1f deg   cte = %+.3f m   %s', ...
        log.t(i), log.v(i), rad2deg(log.delta(i)), log.cte(i), log.status{i});
    drawnow limitrate;
    if i == n, break; end
    pause(frame_dt);
end
end
