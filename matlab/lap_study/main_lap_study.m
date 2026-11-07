%% Capsule track, 3 laps per controller: how do curve radius and an imperfect robot affect each one?
% Separate study, same folder tree. Produces matlab/results/lap_study/:
%   laps_rms.png        RMS error per lap, per plant
%   cte_vs_s.png        error along the track per lap, where the curves are
%   authority.png       steering used vs the limit vs what the curve needs
%   curvature_sweep.png error and steering use as the curve gets tighter
%   lap_metrics.csv     every number behind the figures
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here)); startup_limo();
P = lap_defaults(config_limo());
outdir = fullfile(fileparts(here), 'results', 'lap_study');
if ~exist(outdir, 'dir'), mkdir(outdir); end
if ~exist('LAPS', 'var'), LAPS = 3; end
names = {'pid', 'lqr', 'ltvmpc', 'nmpc'};
straight = 3.0; radius = 1.0; cx = 3.75; cy = 2.5;
[px, py] = capsule_path(straight, radius, cx, cy);
ref = build_reference(px, py, P);
fprintf('Track: %.2f m, closed=%d, max |kappa| = %.2f (limit %.2f)\n', ref.length, ref.closed, max(abs(ref.kappa)), tan(P.max_steer)/P.L);

plants.ideal    = struct('curv_gain', 1.00, 'bias', 0,    'tau', 0.05, 'rate', 0,   'delay', 0,    'noise', 0);
plants.mismatch = struct('curv_gain', 0.89, 'bias', 0.02, 'tau', 0.12, 'rate', 1.5, 'delay', 0.06, 'noise', 0.001);
plants.hard     = struct('curv_gain', 0.80, 'bias', 0.05, 'tau', 0.20, 'rate', 1.0, 'delay', 0.10, 'noise', 0.002);
rows = {}; store = struct();
for pn = fieldnames(plants)'
    for i = 1:numel(names)
      lg = run_laps(ref, names{i}, P, plants.(pn{1}), LAPS);
      store.(pn{1}).(names{i}) = lg;
      for j = 1:numel(lg.laps)
        st = lg.laps(j); mask = lg.lap == j;
        rows(end+1, :) = {pn{1}, lg.label, j, st.rms, max(abs(lg.cte(mask))), st.sat, st.rate, ...
                          max(abs(lg.delta(mask))) / P.max_steer, sum(mask) * lg.ctrl_dt}; %#ok<SAGROW>
        fprintf('%-9s %-8s lap %d rms %.4f sat %4.1f%% rate %.2f\n', pn{1}, lg.label, j, st.rms, 100*st.sat, st.rate);
      end
      if ~lg.completed, fprintf('!! %s %s did not finish %d laps\n', pn{1}, lg.label, LAPS); end
    end
end
T = cell2table(rows, 'VariableNames', {'plant','controller','lap','rms_cte','max_cte','sat_share','rms_rate','peak_steer_use','lap_time'});
writetable(T, fullfile(outdir, 'lap_metrics.csv'));

plot_lap_study(store, T, ref, P, outdir);

%% Curvature sweep: tighter and tighter curves, same straights
radii = [0.45 0.55 0.70 0.85 1.00 1.30 1.60];
sw = {};
for r = radii
    [sx, sy] = capsule_path(straight, r, cx, cy);
    rs = build_reference(sx, sy, P);
    for i = 1:numel(names)
        for once = 1
            lg = run_laps(rs, names{i}, P, plants.mismatch, LAPS);
            mask = lg.lap == max(lg.lap);
            sw(end+1, :) = {r, 1/r, lg.label, sqrt(mean(lg.cte(mask).^2)), max(abs(lg.cte(mask))), mean(abs(lg.delta(mask)) >= 0.98*P.max_steer), max(abs(lg.delta(mask)))/P.max_steer, lg.completed}; %#ok<SAGROW>
        end
    end
end
S = cell2table(sw, 'VariableNames', {'radius','kappa','controller','rms_cte','max_cte','sat_share','peak_use','completed'});
writetable(S, fullfile(outdir, 'curvature_sweep.csv'));
disp(S);
plot_curvature_sweep(S, P, outdir);
