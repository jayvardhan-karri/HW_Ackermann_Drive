function fig = light_figure(varargin)
%LIGHT_FIGURE  figure(...) forced to the light theme (white background).
%   MATLAB R2025a+ follows the desktop dark theme; the lab-grid plots are
%   meant for the projector, so every suite window uses the light theme.
fig = figure(varargin{:});
try
    theme(fig, 'light');
catch
    % older releases have no themes: already light
end
fig.Color = 'w';
end
