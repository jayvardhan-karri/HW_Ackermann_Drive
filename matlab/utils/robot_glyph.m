function h = robot_glyph(ax, h, x, y, yaw)
%ROBOT_GLYPH  Draw / move the LIMO glyph with an unambiguous heading.
%   h = robot_glyph(ax)               creates the glyph (hidden) and returns handles
%   h = robot_glyph(ax, h, x, y, yaw) moves it to pose (x, y, yaw)
%
%   Grey body = LIMO footprint (0.32 x 0.22 m); black arrow from the REAR
%   marker M1 (blue) to the FRONT marker M0 (red). Same convention as the
%   PhaseSpace bridge: heading = vector marker 1 -> marker 0.
BL = 0.32; BW = 0.22; MO = 0.12;
if nargin < 2 || isempty(h)
    h.body  = patch(ax, nan(4,1), nan(4,1), [0.85 0.85 0.85], 'EdgeColor', [0.3 0.3 0.3], ...
                    'LineWidth', 1.2, 'DisplayName', 'LIMO');
    h.arrow = quiver(ax, nan, nan, nan, nan, 0, 'k', 'LineWidth', 2.5, 'MaxHeadSize', 2, ...
                     'HandleVisibility', 'off');
    h.front = plot(ax, nan, nan, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', ...
                   'MarkerSize', 7, 'DisplayName', 'M0 front marker');
    h.rear  = plot(ax, nan, nan, 'o', 'MarkerFaceColor', [0.25 0.41 0.88], ...
                   'MarkerEdgeColor', [0.25 0.41 0.88], 'MarkerSize', 7, 'DisplayName', 'M1 rear marker');
    return
end
c = cos(yaw); s = sin(yaw);
cx = [ BL  -BL  -BL   BL] / 2;
cy = [ BW   BW  -BW  -BW] / 2;
h.body.XData = x + c*cx - s*cy;
h.body.YData = y + s*cx + c*cy;
rx = x - c*MO; ry = y - s*MO;
L = MO + BL/2 + 0.08;
set(h.arrow, 'XData', rx, 'YData', ry, 'UData', c*L, 'VData', s*L);
set(h.front, 'XData', x + c*MO, 'YData', y + s*MO);
set(h.rear,  'XData', rx, 'YData', ry);
end
