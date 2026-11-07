function plot_lap_study(store, T, ref, P, outdir)
%PLOT_LAP_STUDY  Figures of the lap study; see main_lap_study.
labels = {'PID', 'LQR', 'LTV-MPC', 'NMPC'}; names = {'pid','lqr','ltvmpc','nmpc'};
cols = [0.20 0.45 0.75; 0.85 0.45 0.10; 0.20 0.65 0.35];

% --- RMS per lap
f = light_figure('Position', [60 60 1250 400]);
pl = {'ideal','mismatch','hard'};
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for a = 1:3
    ax = nexttile(tl); hold(ax, 'on');
    Y = nan(4, max(T.lap));
    for i = 1:4
        q = T(strcmp(T.plant, pl{a}) & strcmp(T.controller, labels{i}), :);
        Y(i, 1:height(q)) = 100 * q.rms_cte;
    end
    bar(ax, categorical(labels, labels), Y); grid(ax, 'on'); ylabel(ax, 'RMS error (cm)');
    title(ax, sprintf('%s robot', pl{a}));
    if a == 1, legend(ax, compose('lap %d', 1:size(Y,2)), 'Location', 'best'); end
end
exportgraphics(f, fullfile(outdir, 'laps_rms.png'), 'Resolution', 110);

% --- error along the track
f = light_figure('Position', [60 60 1250 900]);
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:4
    ax = nexttile(tl); hold(ax, 'on');
    lg = store.mismatch.(names{i});
    area(ax, (0:ref.n-1)*ref.ds, 100*max(abs(lg.cte))*(abs(ref.kappa) > 0.3), 'FaceColor', [.93 .93 .93], 'EdgeColor', 'none', 'DisplayName', 'curve');
    for j = 1:max(lg.lap)
        m = lg.lap == j; [s, o] = sort(lg.s(m)); c = 100*lg.cte(m); plot(ax, s, c(o), 'Color', cols(min(j,3), :), 'LineWidth', 1.3, 'DisplayName', sprintf('lap %d', j));
    end
    grid(ax, 'on'); xlabel(ax, 'distance along track (m)'); ylabel(ax, 'error (cm)'); title(ax, [labels{i} ', imperfect robot']);
    if i == 1, legend(ax, 'Location', 'best'); end
end
exportgraphics(f, fullfile(outdir, 'cte_vs_s.png'), 'Resolution', 110);

% --- steering authority
f = light_figure('Position', [60 60 1250 900]);
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:4
    ax = nexttile(tl); hold(ax, 'on');
    lg = store.mismatch.(names{i}); m = lg.lap == max(lg.lap);
    [s, o] = sort(lg.s(m)); d = lg.delta(m); ff = lg.d_ff(m);
    plot(ax, s, rad2deg(d(o)), 'Color', cols(1,:), 'LineWidth', 1.2, 'DisplayName', 'steering sent');
    plot(ax, s, rad2deg(ff(o)), 'k--', 'LineWidth', 1.2, 'DisplayName', 'needed by geometry');
    yline(ax, rad2deg(P.max_steer), 'r', 'LineWidth', 1.5, 'DisplayName', 'limit');
    yline(ax, -rad2deg(P.max_steer), 'r', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    grid(ax, 'on'); xlabel(ax, 'distance along track (m)'); ylabel(ax, 'steering (deg)');
    q = T(strcmp(T.plant,'mismatch') & strcmp(T.controller,labels{i}), :);
    title(ax, sprintf('%s: peak %.0f%% of limit, at limit %.1f%% of lap', labels{i}, 100*q.peak_steer_use(end), 100*q.sat_share(end)));
    if i == 1, legend(ax, 'Location', 'best'); end
end
exportgraphics(f, fullfile(outdir, 'authority.png'), 'Resolution', 110);
end
