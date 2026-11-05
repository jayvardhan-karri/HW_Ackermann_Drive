function log = run_laps(ref, name, P, plant, laps)
%RUN_LAPS  One controller on a closed path for a number of laps, against a plant that
%   is not the controller's model.
%   log = run_laps(ref, name, P, plant, laps)
%     plant struct .curv_gain .bias (rad) .tau (s) .rate (rad/s, 0 = none) .delay (s) .noise (m)
%   Laps are counted by forward progress along the path, as on the robot.
%   log: one row per control tick (t x y yaw v delta cte herr s lap kappa d_ff), plus lap
%   statistics log.laps.
c = controller_registry(name, P);
ctrl_dt = 1 / c.rate_hz;
hold_steps = max(1, round(ctrl_dt / P.sim.dt));
y0 = ref.yaw(1); o = P.sim.start_offset;
pose = [ref.x(1) + o(1)*cos(y0) - o(2)*sin(y0), ref.y(1) + o(1)*sin(y0) + o(2)*cos(y0), y0 + o(3)];

t_end = laps * ref.length / mean(ref.v) * 3;
n_ctrl = ceil(t_end / ctrl_dt) + 1;
f = {'t','x','y','yaw','v','delta','cte','herr','s','lap','kappa','d_ff'};
for i = 1:numel(f), log.(f{i}) = nan(n_ctrl, 1); end
n_buf = max(1, round(plant.delay / P.sim.dt) + 1);
buf = zeros(n_buf, 1);
mem = struct(); last_idx = 1; lap_start = 1; progress = 0; lap = 1;
delta_act = 0; v_cmd = 0; delta_cmd = 0; k = 0; step = 0; t = 0;
stats = []; rng(7);
while t <= t_end && lap <= laps
    if mod(step, hold_steps) == 0
        meas = pose + plant.noise * [randn randn 0];
        [err, new_idx] = tracking_errors(ref, meas, c.lookahead, last_idx, P);
        dv = mod(new_idx - last_idx + ref.n/2, ref.n) - ref.n/2;
        if dv > 0, progress = progress + dv; end
        last_idx = new_idx;
        kap = ref.kappa(err.nearest);
        d_ff = atan(P.L * kap);
        [delta_cmd, v_cmd, mem] = c.step(err, meas, ref, mem, P, ctrl_dt);
        delta_cmd = min(max(delta_cmd, -P.max_steer), P.max_steer);
        mem.prev_steer = delta_cmd;
        k = k + 1;
        log.t(k) = t; log.x(k) = pose(1); log.y(k) = pose(2); log.yaw(k) = pose(3);
        log.v(k) = v_cmd; log.delta(k) = delta_cmd; log.cte(k) = err.cte; log.herr(k) = err.herr;
        log.s(k) = (err.nearest - 1) * ref.ds; log.lap(k) = lap; log.kappa(k) = kap;
        log.d_ff(k) = d_ff;
        if progress >= lap * ref.n
            m = lap_start:k;
            if lap == 1, m = m(log.s(m) > 1.0 | (1:numel(m))' > numel(m) * 0.5); end   % leave out the start-up
            st = struct('lap', lap, 'rms', sqrt(mean(log.cte(m).^2)), ...
                        'sat', mean(abs(log.delta(lap_start:k)) >= 0.98 * P.max_steer), ...
                        'rate', sqrt(mean((diff(log.delta(lap_start:k)) / ctrl_dt).^2)));
            stats = [stats; st]; %#ok<AGROW>
            lap = lap + 1; lap_start = k + 1;
        end
    end
    % plant: delay, rate limit, servo lag, curvature gain and steering trim
    buf = [delta_cmd; buf(1:end-1)];
    want = buf(end);
    if plant.tau > 0
        rate = (want - delta_act) / plant.tau;
    else
        rate = (want - delta_act) / P.sim.dt;
    end
    if plant.rate > 0, rate = min(max(rate, -plant.rate), plant.rate); end
    delta_act = delta_act + rate * P.sim.dt;
    eff = atan(plant.curv_gain * tan(delta_act + plant.bias));
    rear = [pose(1) - P.ref_offset*cos(pose(3)), pose(2) - P.ref_offset*sin(pose(3)), pose(3)];
    rear = bicycle_step(rear, v_cmd, eff, P.L, P.sim.dt);
    pose = [rear(1) + P.ref_offset*cos(rear(3)), rear(2) + P.ref_offset*sin(rear(3)), rear(3)];
    step = step + 1; t = step * P.sim.dt;
end
for i = 1:numel(f), log.(f{i}) = log.(f{i})(1:k); end
log.laps = stats; log.label = c.label; log.name = c.name; log.ctrl_dt = ctrl_dt;
log.completed = lap > laps;
end
