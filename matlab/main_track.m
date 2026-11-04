%% LIMO trajectory tracking in MATLAB: draw a path, simulate four controllers
% Run the whole script (F5) or section by section (Ctrl+Enter). It can also be
% opened as a Live Script: right-click > Open as Live Script.
%
% Steps:
%   1. parameters         (config_limo.m)
%   2. reference path     free-hand drawing, a preset, or a saved .mat file
%   3. waypoint table     heading, curvature, speed profile (build_reference.m)
%   4. simulation         PID, LQR, LTV-MPC, NMPC on the bicycle model
%   5. metrics table + plots on the lab grid, saved to matlab/results/
%   6. animation of one run
%   7. (optional) run a controller on the real LIMO through ROS 2
%
% Variables you may set BEFORE running (otherwise the defaults below are used):
%   PATH_MODE   'draw' (default) | 'preset' | 'file'
%   PRESET      'figure8' | 'circle' | 'oval' | 'scurve' | 'straight'
%   PATH_FILE   .mat file with variables x, y (e.g. saved from the drawing window)
%   CONTROLLERS cell array, subset of {'pid','lqr','ltvmpc','nmpc'}
%   ANIMATE     true/false
%   ANIMATE_CONTROLLER  name of the controller to animate

%% 1. Parameters
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
addpath(here); startup_limo();
P = config_limo();
% Example tweaks:  P.v_nominal = 0.3;  P.obstacles = [3.5 3.9 0.2];  P.sim.steer_tau = 0;

if ~exist('PATH_MODE', 'var'), PATH_MODE = 'draw'; end
if ~exist('PRESET', 'var'), PRESET = 'figure8'; end
if ~exist('PATH_FILE', 'var'), PATH_FILE = fullfile(here, 'results', 'limo_path.mat'); end
if ~exist('CONTROLLERS', 'var'), CONTROLLERS = {'pid', 'lqr', 'ltvmpc', 'nmpc'}; end
if ~exist('ANIMATE', 'var'), ANIMATE = usejava('desktop'); end
if ~exist('ANIMATE_CONTROLLER', 'var'), ANIMATE_CONTROLLER = CONTROLLERS{end}; end
outdir = fullfile(here, 'results');
if ~exist(outdir, 'dir'), mkdir(outdir); end

%% 2. Reference path
switch PATH_MODE
    case 'draw'      % press + drag on the lab grid, "Smooth", then "Use this path"
        [px, py] = draw_path(P);
    case 'preset'
        [px, py] = preset_path(PRESET);
    case 'file'
        S = load(PATH_FILE, 'x', 'y'); px = S.x; py = S.y;
    otherwise
        error('PATH_MODE must be draw, preset or file');
end
x = px; y = py; save(fullfile(outdir, 'limo_path.mat'), 'x', 'y');   % reusable with PATH_MODE='file'

%% 3. Waypoint table
ref = build_reference(px, py, P);
kinds = {'open path', 'closed loop'};
fprintf('Reference: %d waypoints, %.2f m, %s, max |kappa| = %.2f 1/m (limit %.2f)\n', ...
        ref.n, ref.length, kinds{ref.closed + 1}, max(abs(ref.kappa)), tan(P.max_steer)/P.L);

%% 4. Simulate every controller
logs = cell(1, numel(CONTROLLERS));
for i = 1:numel(CONTROLLERS)
    logs{i} = run_simulation(ref, CONTROLLERS{i}, P);
    fprintf('  %-8s done (%.1f s simulated)\n', logs{i}.label, logs{i}.t(end));
end

%% 5. Metrics and plots
rows = cellfun(@compute_metrics, logs);
T = struct2table(rows);
T.Properties.RowNames = cellfun(@(L) L.label, logs, 'UniformOutput', false);
disp(T);
writetable(T, fullfile(outdir, 'metrics.csv'), 'WriteRowNames', true);
plot_comparison(ref, logs, P, fullfile(outdir, 'comparison'));
save(fullfile(outdir, 'logs.mat'), 'logs', 'ref', 'P');
fprintf('Saved metrics.csv, comparison_paths.png, comparison_signals.png, logs.mat in %s\n', outdir);

%% 6. Animation
if ANIMATE
    k = find(strcmp(CONTROLLERS, ANIMATE_CONTROLLER), 1);
    animate_run(ref, logs{k}, P, 3);
end

%% 7. (Optional) Run on the real LIMO through ROS 2
% Needs: ROS Toolbox, the Python phasespace_bridge running (it publishes
% /hardware_odom and /hardware_obstacles), same ROS_DOMAIN_ID as P.ros.domain_id.
% Lift the wheels for a first try. Uncomment:
%
%   result = run_on_limo(ref, 'nmpc', P);
