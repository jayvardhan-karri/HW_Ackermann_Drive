function figs = plot_comparison(ref, logs, P, png_prefix)
%PLOT_COMPARISON  Trajectories on the lab grid + error / steering time plots.
%   figs = plot_comparison(ref, logs, P)                 logs = cell array of run_simulation outputs
%   figs = plot_comparison(ref, logs, P, 'results/run')  also saves run_paths.png / run_signals.png
%   Figure 1: lab grid (red axes), obstacles, reference (black dashed), one
%             coloured trail per controller, robot glyph at each final pose.
%   Figure 2: cross-track error, heading error and steering vs time.
if nargin < 4, png_prefix = ''; end
cols = lines(numel(logs));
th = linspace(0, 2*pi, 60);

figs(1) = light_figure('Name', 'LIMO tracking - paths', 'Color', 'w', 'Position', [80 80 1100 850]);
ax = axes(figs(1));
style_lab_axes(ax, P, 'Trajectory tracking (simulation)');
for k = 1:size(P.obstacles, 1)
    o = P.obstacles(k, :);
    fill(ax, o(1) + o(3)*cos(th), o(2) + o(3)*sin(th), [1 0.5 0.4], 'FaceAlpha', 0.6, ...
         'EdgeColor', [0.6 0 0], 'DisplayName', 'Obstacle');
end
xr = ref.x; yr = ref.y;
if ref.closed, xr(end+1) = xr(1); yr(end+1) = yr(1); end
plot(ax, xr, yr, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference');
for i = 1:numel(logs)
    L = logs{i}; mt = compute_metrics(L);
    plot(ax, L.x, L.y, '-', 'Color', cols(i, :), 'LineWidth', 2, ...
         'DisplayName', sprintf('%s (RMS CTE %.3f m)', L.label, mt.rms_cte));
end
g = robot_glyph(ax);
robot_glyph(ax, g, logs{end}.x(end), logs{end}.y(end), logs{end}.yaw(end));
legend(ax, 'Location', 'northeastoutside');

figs(2) = light_figure('Name', 'LIMO tracking - signals', 'Color', 'w', 'Position', [120 60 1100 850]);
tl = tiledlayout(figs(2), 3, 1, 'TileSpacing', 'compact');
names = {'cte', 'herr', 'delta'};
ylab = {'Cross-track error (m)', 'Heading error (rad)', 'Steering \delta (rad)'};
for p = 1:3
    a = nexttile(tl); hold(a, 'on'); grid(a, 'on');
    for i = 1:numel(logs)
        plot(a, logs{i}.t, logs{i}.(names{p}), 'Color', cols(i, :), 'LineWidth', 1.4, ...
             'DisplayName', logs{i}.label);
    end
    if p == 3
        yline(a, [-P.max_steer P.max_steer], 'r--', 'HandleVisibility', 'off');
        xlabel(a, 'Time (s)');
    end
    ylabel(a, ylab{p}); a.FontSize = 12;
end
legend(nexttile(tl, 1), 'Location', 'northeast');

if ~isempty(png_prefix)
    exportgraphics(figs(1), [png_prefix '_paths.png'], 'Resolution', 110);
    exportgraphics(figs(2), [png_prefix '_signals.png'], 'Resolution', 110);
end
end
