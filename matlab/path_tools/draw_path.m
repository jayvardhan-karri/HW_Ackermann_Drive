function [x, y] = draw_path(P, opts)
%DRAW_PATH  Free-hand path builder on the lab grid (base MATLAB, no toolbox).
%   [x, y] = draw_path(P)            opens the window and waits for "Use this path"
%   [x, y] = draw_path(P, opts)      options (struct):
%       opts.Preset     name of a preset to start from ('' = empty canvas)
%       opts.Test       Nx2 points: draw them programmatically, smooth, return
%                       (no waiting; used for automated tests)
%       opts.Snapshot   file name: save a PNG of the window (with opts.Test)
%
%   UI ELEMENTS
%     axes        lab grid (red axes, 1 m grid) + obstacles from P.obstacles
%     raw line    blue: the stroke as drawn
%     smooth line green, thicker: after "Smooth"
%     start/end   green dot / red cross
%     slider      "Smoothing window" 0.05 - 1.00 m (default P.smooth.window)
%     buttons     Smooth | Clear | Load preset (menu) | Save .mat | Use this path
%     status      points, length, closed loop / open path
%   Mouse: press + drag on the axes draws a NEW stroke (replaces the old one).
%   A stroke whose ends meet (< P.ref.closed_tol) is treated as a closed loop.
if nargin < 2, opts = struct(); end
if ~isfield(opts, 'Preset'), opts.Preset = ''; end
if ~isfield(opts, 'Test'), opts.Test = []; end
if ~isfield(opts, 'Snapshot'), opts.Snapshot = ''; end

raw = zeros(0, 2);          % stroke as drawn
smoothed = zeros(0, 2);     % after Smooth ([] until pressed)
drawing = false;
done = false;

fig = light_figure('Name', 'LIMO Path Builder (MATLAB)', 'NumberTitle', 'off', ...
             'Color', 'w', 'Units', 'normalized', 'Position', [0.08 0.08 0.8 0.82]);
ax = axes(fig, 'Position', [0.06 0.17 0.9 0.76]);
style_lab_axes(ax, P, 'Draw the reference path (press, drag, release)');
th = linspace(0, 2*pi, 60);
for k = 1:size(P.obstacles, 1)
    o = P.obstacles(k, :);
    fill(ax, o(1) + o(3)*cos(th), o(2) + o(3)*sin(th), [1 0.5 0.4], 'FaceAlpha', 0.6, ...
         'EdgeColor', [0.6 0 0], 'DisplayName', sprintf('Obstacle %d', k));
end
hRaw = plot(ax, nan, nan, 'b-', 'LineWidth', 2, 'DisplayName', 'Drawn');
hSm  = plot(ax, nan, nan, '-', 'Color', [0 0.6 0], 'LineWidth', 3, 'DisplayName', 'Smoothed');
hStart = plot(ax, nan, nan, 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0 0.7 0], ...
              'MarkerEdgeColor', 'k', 'DisplayName', 'Start');
hEnd = plot(ax, nan, nan, 'x', 'MarkerSize', 13, 'LineWidth', 3, 'Color', 'r', 'DisplayName', 'End');
legend(ax, 'Location', 'northeast');

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', 'Position', [0.06 0.085 0.14 0.03], ...
          'String', 'Smoothing window (m)', 'BackgroundColor', 'w', 'FontSize', 11);
hSlider = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
          'Position', [0.06 0.05 0.14 0.03], 'Min', 0.05, 'Max', 1.0, ...
          'Value', P.smooth.window, 'Callback', @(~, ~) onSlider());
hSliderVal = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
          'Position', [0.20 0.05 0.05 0.03], 'String', sprintf('%.2f', P.smooth.window), ...
          'BackgroundColor', 'w', 'FontSize', 11);
mkbtn('Smooth', [0.27 0.04 0.10 0.06], @(~, ~) onSmooth());
mkbtn('Clear', [0.38 0.04 0.10 0.06], @(~, ~) onClear());
hPreset = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
          'Position', [0.49 0.045 0.13 0.05], 'FontSize', 11, ...
          'String', {'Load preset...', 'figure8', 'circle', 'oval', 'scurve', 'straight'}, ...
          'Callback', @(~, ~) onPreset());
mkbtn('Save .mat', [0.63 0.04 0.10 0.06], @(~, ~) onSave());
mkbtn('Use this path', [0.76 0.04 0.18 0.06], @(~, ~) onDone(), [0.6 0.95 0.6]);
hStatus = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', 'Position', [0.06 0.115 0.6 0.035], ...
          'String', 'Draw a path with the mouse, or load a preset.', 'BackgroundColor', 'w', ...
          'FontSize', 12, 'HorizontalAlignment', 'left');

fig.WindowButtonDownFcn   = @(~, ~) onDown(currentPoint());
fig.WindowButtonMotionFcn = @(~, ~) onMove(currentPoint());
fig.WindowButtonUpFcn     = @(~, ~) onUp();
fig.CloseRequestFcn       = @(~, ~) onDone();

if ~isempty(opts.Preset)
    [px, py] = preset_path(opts.Preset); setStroke([px py]);
end

if ~isempty(opts.Test)                        % ---- automated test mode
    onDown(opts.Test(1, :));
    for k = 2:size(opts.Test, 1), onMove(opts.Test(k, :)); end
    onUp(); onSmooth();
    if ~isempty(opts.Snapshot), drawnow; exportgraphics(fig, opts.Snapshot, 'Resolution', 80); end
    onDone();
else                                          % ---- interactive
    while ~done && isvalid(fig)
        uiwait(fig);
    end
end
if isempty(smoothed), out = raw; else, out = smoothed; end
if isvalid(fig), delete(fig); end
if size(out, 1) < 4
    error('draw_path:empty', 'No path drawn (need at least 4 points).');
end
x = out(:, 1); y = out(:, 2);

    % ------------------------------------------------------------ helpers
    function b = mkbtn(label, pos, cb, color)
        if nargin < 4, color = [0.94 0.94 0.94]; end
        b = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', pos, ...
                      'String', label, 'FontSize', 12, 'FontWeight', 'bold', ...
                      'BackgroundColor', color, 'Callback', cb);
    end
    function p = currentPoint()
        c = ax.CurrentPoint; p = c(1, 1:2);
    end
    function inside = inAxes(p)
        inside = p(1) >= P.lab_x(1) && p(1) <= P.lab_x(2) && p(2) >= P.lab_y(1) && p(2) <= P.lab_y(2);
    end
    function p = clip(p)
        p = [min(max(p(1), P.lab_x(1)), P.lab_x(2)), min(max(p(2), P.lab_y(1)), P.lab_y(2))];
    end
    % [UI] mouse press: start a new stroke
    function onDown(p)
        if ~inAxes(p), return; end
        drawing = true; raw = clip(p); smoothed = zeros(0, 2);
        refresh();
    end
    % [UI] mouse drag: append points
    function onMove(p)
        if ~drawing, return; end
        raw(end+1, :) = clip(p);
        refresh();
    end
    % [UI] mouse release: stop drawing
    function onUp()
        drawing = false; refresh();
    end
    % [UI] "Smooth" button
    function onSmooth()
        if size(raw, 1) < 4, setStatus('Need at least 4 points to smooth.'); return; end
        [xs, ys] = smooth_path(raw(:, 1), raw(:, 2), hSlider.Value, P.ref.closed_tol);
        smoothed = [xs ys]; refresh();
    end
    % [UI] smoothing slider moved: show the value, re-smooth if already smoothed
    function onSlider()
        hSliderVal.String = sprintf('%.2f', hSlider.Value);
        if ~isempty(smoothed), onSmooth(); end
    end
    % [UI] "Clear" button
    function onClear()
        raw = zeros(0, 2); smoothed = zeros(0, 2); refresh();
    end
    % [UI] preset menu
    function onPreset()
        names = hPreset.String; k = hPreset.Value;
        if k == 1, return; end
        [px, py] = preset_path(names{k}); setStroke([px py]); hPreset.Value = 1;
    end
    function setStroke(pts)
        raw = pts; smoothed = pts; refresh();
    end
    % [UI] "Save .mat" button: saves x, y of the current path
    function onSave()
        if isempty(smoothed), pts = raw; else, pts = smoothed; end
        if size(pts, 1) < 4, setStatus('Nothing to save.'); return; end
        [f, d] = uiputfile('*.mat', 'Save path', 'limo_path.mat');
        if isequal(f, 0), return; end
        x = pts(:, 1); y = pts(:, 2); %#ok<NASGU>
        save(fullfile(d, f), 'x', 'y'); setStatus(['Saved ' fullfile(d, f)]);
    end
    % [UI] "Use this path" button / window closed
    function onDone()
        done = true;
        if isvalid(fig), uiresume(fig); end
    end
    function setStatus(msg)
        hStatus.String = msg;
    end
    % [UI] redraw lines, start/end markers and the status line
    function refresh()
        set(hRaw, 'XData', raw(:, 1), 'YData', raw(:, 2));
        if isempty(smoothed)
            set(hSm, 'XData', nan, 'YData', nan); pts = raw;
        else
            set(hSm, 'XData', smoothed(:, 1), 'YData', smoothed(:, 2)); pts = smoothed;
        end
        if size(pts, 1) >= 2
            set(hStart, 'XData', pts(1, 1), 'YData', pts(1, 2));
            set(hEnd, 'XData', pts(end, 1), 'YData', pts(end, 2));
            L = sum(hypot(diff(pts(:, 1)), diff(pts(:, 2))));
            closed = hypot(pts(end, 1) - pts(1, 1), pts(end, 2) - pts(1, 2)) < P.ref.closed_tol;
            kinds = {'open path (robot stops at the end)', 'closed loop'};
            setStatus(sprintf('%d points | %.2f m | %s%s', size(pts, 1), L, kinds{closed + 1}, ...
                      repmat(' | smoothed', 1, ~isempty(smoothed))));
        else
            set(hStart, 'XData', nan, 'YData', nan); set(hEnd, 'XData', nan, 'YData', nan);
        end
        drawnow limitrate;
    end
end
