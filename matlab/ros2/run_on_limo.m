function result = run_on_limo(ref, name, P, opts)
%RUN_ON_LIMO  Run a MATLAB controller on the real LIMO through ROS 2.
%   result = run_on_limo(ref, name, P)
%   result = run_on_limo(ref, name, P, opts)
%     ref    reference from build_reference (lab / PhaseSpace frame)
%     name   'pid' | 'lqr' | 'ltvmpc' | 'nmpc'
%     opts.Plot  live figure (default: true when the MATLAB desktop is running)
%
%   Requires ROS Toolbox. The Python phasespace_bridge must be running: it
%   turns the PhaseSpace markers into /hardware_odom (and /hardware_scan).
%   Same ROS_DOMAIN_ID as P.ros.domain_id. Do NOT run a Python controller at
%   the same time (both would publish /cmd_vel).
%
%   Loop (at the controller rate, 50 Hz / NMPC 25 Hz):
%     1. latest /hardware_odom -> pose (optionally re-zeroed, P.ros.rezero_odom)
%     2. tracking_errors -> controller step -> delta, v
%     3. LiDAR safety filter on the latest /hardware_scan (if any)
%     4. publish P.ros.cmd_topic (default /limo1/cmd_vel, the namespaced
%        LIMO): linear.x = v, angular.z = v*tan(delta)/L
%   Ends when an open path is finished, P.ros.duration elapses (0 = no
%   limit), the live figure is closed, or Ctrl-C. A zero Twist is ALWAYS
%   sent at the end (onCleanup), whatever the reason.
%
%   result: log struct like run_simulation (t, x, y, yaw, v, delta, omega,
%   cte, herr, x_ref, y_ref, dmin, status, solve_ms) + result.metrics.
if nargin < 4, opts = struct(); end
if ~isfield(opts, 'Plot'), opts.Plot = usejava('desktop'); end
c = controller_registry(name, P);
dt = 1 / c.rate_hz;

node = ros2node('/matlab_limo_controller', P.ros.domain_id);
odomSub = ros2subscriber(node, '/hardware_odom', 'nav_msgs/Odometry');
scanSub = ros2subscriber(node, '/hardware_scan', 'sensor_msgs/LaserScan');
cmdPub  = ros2publisher(node, P.ros.cmd_topic, 'geometry_msgs/Twist');   % e.g. /limo1/cmd_vel
cleanup = onCleanup(@() stop_robot(cmdPub)); %#ok<NASGU>
if P.ros.publish_path
    pathPub = ros2publisher(node, '/reference_path', 'nav_msgs/Path');
end

% Wait for odometry (bridge running, robot tracked)
fprintf('[run_on_limo] waiting for /hardware_odom ...\n');
t_wait = tic;
while isempty(odomSub.LatestMessage)
    if toc(t_wait) > 15
        error('run_on_limo:noOdom', 'No /hardware_odom after 15 s. Is phasespace_bridge running (domain %d)?', P.ros.domain_id);
    end
    pause(0.1);
end
if P.ros.publish_path
    pause(0.5); publish_path_ros2(pathPub, ref);
end

origin = [];
if opts.Plot, V = make_view(ref, P, c.label); end
n_max = 200000; Z = nan(n_max, 1);
L = struct('t', Z, 'x', Z, 'y', Z, 'yaw', Z, 'v', Z, 'delta', Z, 'omega', Z, ...
           'cte', Z, 'herr', Z, 'x_ref', Z, 'y_ref', Z, 'dmin', Z, 'solve_ms', Z);
L.status = cell(n_max, 1);
mem = struct(); last_idx = 1; k = 0; goal = false;
fprintf('[run_on_limo] %s running at %d Hz. Close the figure or press Ctrl-C to stop.\n', c.label, c.rate_hz);
t0 = tic; next = 0;
while true
    t = toc(t0);
    if t < next, pause(max(0, next - t - 0.001)); continue; end
    next = next + dt;
    if P.ros.duration > 0 && t >= P.ros.duration, break; end
    if opts.Plot && ~isvalid(V.fig), break; end

    od = odomSub.LatestMessage;
    q = od.pose.pose.orientation;
    raw = [od.pose.pose.position.x, od.pose.pose.position.y, ...
           atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2))];
    if isempty(origin)
        if P.ros.rezero_odom, origin = raw; else, origin = [0 0 0]; end
    end
    c0 = cos(origin(3)); s0 = sin(origin(3)); d = raw(1:2) - origin(1:2);
    pose = [c0*d(1) + s0*d(2), -s0*d(1) + c0*d(2), wrap_angle(raw(3) - origin(3))];

    [err, last_idx] = tracking_errors(ref, pose, c.lookahead, last_idx, P);
    if err.goal, goal = true; break; end
    tic_s = tic;
    [delta, v, mem] = c.step(err, pose, ref, mem, P, dt);
    solve_ms = toc(tic_s) * 1000;
    dmin = inf; status = 'NO_SCAN';
    sc = scanSub.LatestMessage;
    if P.safety.enabled && ~isempty(sc)
        ang = double(sc.angle_min) + (0:numel(sc.ranges)-1)' * double(sc.angle_increment);
        [v, delta, dmin, status] = lidar_safety_filter(v, delta, double(sc.ranges(:)), ang, P.safety);
    end
    delta = min(max(delta, -P.max_steer), P.max_steer);
    mem.prev_steer = delta;
    omega = 0; if abs(v) > 1e-4, omega = v * tan(delta) / P.L; end
    msg = ros2message(cmdPub); msg.linear.x = v; msg.angular.z = omega;
    send(cmdPub, msg);

    k = k + 1;
    L.t(k) = t; L.x(k) = pose(1); L.y(k) = pose(2); L.yaw(k) = pose(3);
    L.v(k) = v; L.delta(k) = delta; L.omega(k) = omega; L.cte(k) = err.cte; L.herr(k) = err.herr;
    L.x_ref(k) = err.la(1); L.y_ref(k) = err.la(2); L.dmin(k) = dmin; L.status{k} = status;
    L.solve_ms(k) = solve_ms;
    if opts.Plot && mod(k, max(1, round(c.rate_hz / 10))) == 0
        update_view(V, L, k);
    end
    if k >= n_max, break; end
end
stop_robot(cmdPub);
for f = fieldnames(L)'
    L.(f{1}) = L.(f{1})(1:k);
end
L.label = c.label; L.name = c.name; L.goal = goal; L.ctrl_dt = dt;
result = L;
if k > 1, result.metrics = compute_metrics(L); end
fprintf('[run_on_limo] finished after %.1f s (%d ticks, goal=%d)\n', toc(t0), k, goal);
end

function stop_robot(pub)
%STOP_ROBOT  Publish a zero Twist (robot halts).
try
    send(pub, ros2message(pub));
catch
end
end

function V = make_view(ref, P, label)
%MAKE_VIEW  [UI] live figure: lab grid, reference, trail, robot glyph, status.
V.fig = light_figure('Name', ['LIMO on hardware - ' label], 'Position', [100 80 1100 850]);
V.ax = axes(V.fig);
style_lab_axes(V.ax, P, ['MATLAB ' label ' on the LIMO']);
plot(V.ax, ref.x, ref.y, 'g--', 'LineWidth', 2, 'DisplayName', 'Reference');
V.trail = plot(V.ax, nan, nan, '-', 'Color', [1 0.55 0], 'LineWidth', 2.5, 'DisplayName', 'Robot');
V.glyph = robot_glyph(V.ax);
V.info = text(V.ax, P.lab_x(1) + 0.1, P.lab_y(2) - 0.1, '', 'VerticalAlignment', 'top', ...
              'FontName', 'monospace', 'FontSize', 12, 'BackgroundColor', [1 1 1 0.8]);
legend(V.ax, 'Location', 'northeast');
end

function update_view(V, L, k)
%UPDATE_VIEW  [UI] redraw trail, glyph and status at about 10 Hz.
set(V.trail, 'XData', L.x(1:k), 'YData', L.y(1:k));
robot_glyph(V.ax, V.glyph, L.x(k), L.y(k), L.yaw(k));
V.info.String = sprintf('t = %5.1f s   v = %.2f m/s   steer = %+5.1f deg   cte = %+.3f m   %s', ...
    L.t(k), L.v(k), rad2deg(L.delta(k)), L.cte(k), L.status{k});
drawnow limitrate;
end
