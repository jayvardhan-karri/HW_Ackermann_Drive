function startup_limo()
%STARTUP_LIMO  Add every folder of the MATLAB LIMO suite to the path.
%   Called automatically by main_track.m and run_on_limo.m.
root = fileparts(mfilename('fullpath'));
addpath(root);
for d = {'path_tools', 'controllers', 'safety', 'sim', 'ros2', 'utils', 'lap_study'}
    addpath(fullfile(root, d{1}));
end
end
