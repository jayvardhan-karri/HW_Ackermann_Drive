function P = config_limo()
%CONFIG_LIMO  All parameters of the MATLAB LIMO controller suite in one struct.
%
%   P = config_limo() returns the defaults. Change fields after the call,
%   e.g.  P = config_limo(); P.v_nominal = 0.3; P.pid.kp = 2.0;
%
%   The values match the ROS 2 package (src/limo_mocap_suite) so results are
%   comparable between the Python nodes and this MATLAB code.

% ---------------------------------------------------------------- robot
P.L          = 0.20;        % wheelbase (m)
P.ref_offset = 0.10;        % tracked point (marker midpoint) ahead of the rear axle (m); measure it
P.max_steer  = 0.48869;     % steering limit (rad) = 28 deg (limo_driver.h)
P.v_nominal  = 0.40;        % cruise speed (m/s), reduced in curves

% ---------------------------------------------------------------- lab / plots
P.lab_x = [-0.5 8.0];       % lab floor covered by PhaseSpace (m)
P.lab_y = [-1.0 6.0];

% ---------------------------------------------------------------- reference path
P.ref.ds          = 0.01;   % resampling step along the path (m)
P.ref.speed_gain  = 0.75;   % v_ref = v_nominal / (1 + speed_gain*|kappa|)
P.ref.closed_tol  = 0.15;   % first/last point closer than this -> closed loop (m)
P.ref.search_back = 25;     % nearest-point window behind last index (samples)
P.ref.search_fwd  = 160;    % ... and ahead (samples)
P.ref.goal_tol    = 0.05;   % open path: stop within this of / past the end (m)
P.ref.kappa_window = 0.10;  % curvature smoothing window (m)
P.smooth.window   = 0.30;   % smoothing window for drawn paths (m of arc length)

% ---------------------------------------------------------------- controllers
P.pid  = struct('kp', 1.8, 'ki', 0.05, 'kd', 0.45, 'kh', 1.15, ...
                'i_limit', 0.5, 'lookahead', 10, 'rate_hz', 50);
P.lqr  = struct('q_cte', 450, 'q_heading', 35, 'r_steer', 0.5, ...
                'lookahead', 8, 'rate_hz', 50);
P.mpc  = struct('N', 16, 'dt', 0.04, 'q_cte', 520, 'q_dcte', 35, 'q_heading', 42, ...
                'r_steer', 0.28, 'rd_steer', 0.45, 'ki', 0.06, 'i_limit', 0.5, ...
                'lookahead', 9, 'rate_hz', 50);
P.nmpc = struct('N', 10, 'dt', 0.08, 'q_cte', 40, 'q_heading', 8, 'rd_steer', 2.5, ...
                'maxiter', 8, 'lookahead', 5, 'rate_hz', 25);

% ---------------------------------------------------------------- safety (CBF on known obstacles)
P.cbf = struct('robot_radius', 0.20, 'safety_margin', 0.10, 'alpha', 0.6, ...
               'max_steer', P.max_steer, 'steer_weight', 0.15);
P.obstacles = zeros(0, 3);               % [x y r] per row, lab frame (m); default none

% ---------------------------------------------------------------- simulation
P.sim.dt           = 0.01;               % plant integration step (s)
P.sim.start_offset = [0 0.10 deg2rad(10)];  % initial error vs path start [dx dy dyaw]
P.sim.steer_tau    = 0.05;               % steering servo lag (s), 0 = ideal
P.sim.laps         = 1;                  % closed paths: laps to drive
P.sim.t_max        = 120;                % hard time limit (s)

% ---------------------------------------------------------------- ROS 2 (hardware)
P.ros.domain_id    = 0;                  % must match the LIMO / base station
P.ros.cmd_topic    = '/limo1/cmd_vel';   % LIMO's velocity topic (namespaced robot); '/cmd_vel' if none
P.ros.rezero_odom  = false;              % false = path in the PhaseSpace frame
P.ros.publish_path = true;               % also publish the path on /reference_path
P.ros.duration     = 0;                  % 0 = until the path end or figure closed
end
