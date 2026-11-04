function style_lab_axes(ax, P, ttl)
%STYLE_LAB_AXES  Projector-friendly lab grid (same look as the ROS GUIs).
%   Lab extents P.lab_x / P.lab_y, equal aspect, 1 m major + 0.5 m minor
%   grid, thick RED x/y axes through (0,0) labelled +X / +Y, large fonts.
hold(ax, 'on');
axis(ax, 'equal');
xlim(ax, P.lab_x); ylim(ax, P.lab_y);
ax.XTick = ceil(P.lab_x(1)):1:floor(P.lab_x(2));
ax.YTick = ceil(P.lab_y(1)):1:floor(P.lab_y(2));
ax.XMinorTick = 'on'; ax.YMinorTick = 'on';
ax.XAxis.MinorTickValues = P.lab_x(1):0.5:P.lab_x(2);
ax.YAxis.MinorTickValues = P.lab_y(1):0.5:P.lab_y(2);
grid(ax, 'on'); grid(ax, 'minor');
ax.GridColor = [0.35 0.35 0.35]; ax.GridAlpha = 0.6;
ax.MinorGridColor = [0.7 0.7 0.7]; ax.MinorGridAlpha = 0.5;
ax.FontSize = 13; ax.FontWeight = 'bold';
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
title(ax, ttl, 'FontSize', 15);
red = [0.9 0 0];
plot(ax, P.lab_x, [0 0], '-', 'Color', red, 'LineWidth', 3.5, 'HandleVisibility', 'off');
plot(ax, [0 0], P.lab_y, '-', 'Color', red, 'LineWidth', 3.5, 'HandleVisibility', 'off');
text(ax, P.lab_x(2) - 0.1, 0.12, '+X', 'Color', red, 'FontSize', 16, 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'BackgroundColor', 'w', 'EdgeColor', red);
text(ax, 0.1, P.lab_y(2) - 0.6, '+Y', 'Color', red, 'FontSize', 16, 'FontWeight', 'bold', ...
     'VerticalAlignment', 'top', 'BackgroundColor', 'w', 'EdgeColor', red);
text(ax, 0.03, -0.05, '(0,0)', 'Color', red, 'FontSize', 12, 'FontWeight', 'bold', ...
     'VerticalAlignment', 'top');
end
