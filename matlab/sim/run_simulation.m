function log = run_simulation(ref, name, P)
%RUN_SIMULATION  Closed-loop simulation of one controller on the bicycle model.
%   log = run_simulation(ref, name, P)
%     ref   from build_reference;  name 'pid' | 'lqr' | 'ltvmpc' | 'nmpc'
%
%   Plant   : kinematic bicycle (bicycle_step) every P.sim.dt, with a
%             first-order steering servo lag P.sim.steer_tau.
%   Control : runs at the controller's rate (50 Hz, NMPC 25 Hz), zero-order
%             hold in between - like the ROS timers.
%   Each control tick (same order as the Python control_loop):
%     1. tracking_errors  -> cte, herr, look-ahead row, nearest
%     2. controller step  -> delta, v
%     3. LiDAR safety (synthetic scan of P.obstacles) if P.safety.enabled
%     4. clip delta, remember it as the applied steering (mem.prev_steer)
%   Start   : path start + P.sim.start_offset ([along, left, yaw] error).
%   End     : open path -> goal reached; closed -> P.sim.laps laps (time
%             estimate); always <= P.sim.t_max.
%
%   log fields (one row per control tick): t, x, y, yaw, v, delta, omega,
%   cte, herr, x_ref, y_ref, dmin, status (cellstr), solve_ms; plus
%   controller label, goal flag, ref.
c = controller_registry(name, P);
ctrl_dt = 1 / c.rate_hz;
hold_steps = max(1, round(ctrl_dt / P.sim.dt));

y0 = ref.yaw(1); o = P.sim.start_offset;
pose = [ref.x(1) + o(1)*cos(y0) - o(2)*sin(y0), ref.y(1) + o(1)*sin(y0) + o(2)*cos(y0), y0 + o(3)];

vmean = mean(ref.v);
if ref.closed
    t_end = min(P.sim.t_max, P.sim.laps * ref.length / vmean * 1.25);
else
    t_end = min(P.sim.t_max, ref.length / vmean * 1.5 + 10);
end
n_ctrl = ceil(t_end / ctrl_dt) + 1;
Z = nan(n_ctrl, 1);
log = struct('t', Z, 'x', Z, 'y', Z, 'yaw', Z, 'v', Z, 'delta', Z, 'omega', Z, ...
             'cte', Z, 'herr', Z, 'x_ref', Z, 'y_ref', Z, 'dmin', Z, 'solve_ms', Z);
log.status = cell(n_ctrl, 1);

mem = struct(); last_idx = 1; delta_act = 0; v_cmd = 0; delta_cmd = 0;
k = 0; step = 0; t = 0; goal = false;
while t <= t_end
    if mod(step, hold_steps) == 0
        [err, last_idx] = tracking_errors(ref, pose, c.lookahead, last_idx, P);
        if err.goal, goal = true; break; end
        tic;
        [delta_cmd, v_cmd, mem] = c.step(err, pose, ref, mem, P, ctrl_dt);
        solve_ms = toc * 1000;
        dmin = inf; status = 'OFF';
        if P.safety.enabled
            [rg, an] = synthetic_scan(pose, P.obstacles);
            [v_cmd, delta_cmd, dmin, status] = lidar_safety_filter(v_cmd, delta_cmd, rg, an, P.safety);
        end
        delta_cmd = min(max(delta_cmd, -P.max_steer), P.max_steer);
        mem.prev_steer = delta_cmd;
        k = k + 1;
        log.t(k) = t; log.x(k) = pose(1); log.y(k) = pose(2); log.yaw(k) = pose(3);
        log.v(k) = v_cmd; log.delta(k) = delta_cmd; log.omega(k) = v_cmd * tan(delta_cmd) / P.L;
        log.cte(k) = err.cte; log.herr(k) = err.herr;
        log.x_ref(k) = err.la(1); log.y_ref(k) = err.la(2);
        log.dmin(k) = dmin; log.status{k} = status; log.solve_ms(k) = solve_ms;
    end
    if P.sim.steer_tau > 0
        delta_act = delta_act + (delta_cmd - delta_act) * min(1, P.sim.dt / P.sim.steer_tau);
    else
        delta_act = delta_cmd;
    end
    pose = bicycle_step(pose, v_cmd, delta_act, P.L, P.sim.dt);
    step = step + 1; t = step * P.sim.dt;
end
for f = fieldnames(log)'
    log.(f{1}) = log.(f{1})(1:k);
end
log.label = c.label; log.name = c.name; log.goal = goal; log.ctrl_dt = ctrl_dt;
end
