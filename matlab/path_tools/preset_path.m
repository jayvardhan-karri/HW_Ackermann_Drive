function [x, y] = preset_path(name)
%PRESET_PATH  Built-in reference paths in the lab (PhaseSpace) frame, metres.
%   [x, y] = preset_path(name), name is one of
%     'figure8'  closed  x = 3.75 + 2.05 sin t, y = 3.0 + 0.9 sin 2t
%     'circle'   closed  radius 1.5 m around (4.0, 3.0), counter-clockwise
%     'oval'     closed  ellipse 3.2 x 1.8 m around (3.75, 3.0)
%     'scurve'   open    left-right S from (0.5, 2.5) to (6.5, 2.5)
%     'straight' open    (0.5, 2.5) -> (6.5, 2.5)
t = linspace(0, 2*pi, 1201)'; t(end) = [];
switch lower(name)
    case 'figure8'
        x = 3.75 + 2.05*sin(t);   y = 3.0 + 0.9*sin(2*t);
    case 'circle'
        x = 4.0 + 1.5*sin(t);     y = 3.0 - 1.5*cos(t);
    case 'oval'
        x = 3.75 + 3.2*sin(t);    y = 3.0 - 1.8*cos(t);
    case 'scurve'
        x = linspace(0.5, 6.5, 600)';  y = 2.5 + 1.0*sin(2*pi*(x - 0.5)/6);
    case 'straight'
        x = linspace(0.5, 6.5, 600)';  y = 2.5*ones(size(x));
    otherwise
        error('preset_path:unknown', 'Unknown preset "%s"', name);
end
end
