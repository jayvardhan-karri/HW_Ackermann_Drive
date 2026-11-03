function [delta, v, mem] = ctrl_pid(err, ~, ~, mem, P, dt)
%CTRL_PID  Kinematic Ackermann PID with curvature feed-forward.
%   [delta, v, mem] = ctrl_pid(err, pose, ref, mem, P, dt)
%
%   delta = atan(L*kappa) + kp*cte + ki*integral(cte) + kd*d(cte)/dt + kh*herr
%   - integral clamped to +/- P.pid.i_limit (anti-windup)
%   - derivative = backward difference over the control period dt
%   - kappa is taken at the look-ahead point (err.la)
%   mem (persistent between calls): I, prev_cte.
C = P.pid;
if ~isfield(mem, 'I')
    mem.I = 0; mem.prev_cte = err.cte;     % no derivative kick on the first call
end
mem.I = min(max(mem.I + err.cte * dt, -C.i_limit), C.i_limit);
D = (err.cte - mem.prev_cte) / dt;
mem.prev_cte = err.cte;

ff = atan(P.L * err.la(5));
delta = ff + C.kp*err.cte + C.ki*mem.I + C.kd*D + C.kh*err.herr;
delta = min(max(delta, -P.max_steer), P.max_steer);
v = speed_law(err.la(4), delta, P);
end
