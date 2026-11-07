function plot_curvature_sweep(S, P, outdir)
%PLOT_CURVATURE_SWEEP  Error and steering use against curve tightness.
labels = {'PID', 'LQR', 'LTV-MPC', 'NMPC'};
f = light_figure('Position', [60 60 1250 480]);
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
kmax = tan(P.max_steer) / P.L;
tt = {'Max error, last lap (cm)', 'Time at the steering limit (% of lap)', 'Peak steering (% of limit)'};
cols = lines(4);
for a = 1:3
    ax = nexttile(tl); hold(ax, 'on');
    for i = 1:4
        q = S(strcmp(S.controller, labels{i}), :);
        if a == 1, y = 100*q.max_cte; elseif a == 2, y = 100*q.sat_share; else, y = 100*q.peak_use; end
        plot(ax, q.kappa, y, '-o', 'Color', cols(i, :), 'LineWidth', 1.3, 'DisplayName', labels{i});
    end
    xline(ax, kmax, 'r', 'limit', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    grid(ax, 'on'); xlabel(ax, 'curve tightness |kappa| (1/m)'); title(ax, tt{a});
    if a == 1, legend(ax, 'Location', 'northwest', 'NumColumns', 2); end
end
exportgraphics(f, fullfile(outdir, 'curvature_sweep.png'), 'Resolution', 110);
end
