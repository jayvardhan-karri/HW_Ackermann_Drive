function [delta, v, mem] = ctrl_nmpc(err, pose, ref, mem, P, ~)
%CTRL_NMPC  Nonlinear MPC on the bicycle model (no Optimization Toolbox needed).
%   [delta, v, mem] = ctrl_nmpc(err, pose, ref, mem, P, dt)
%
%   Decision variables: steering u(1..N) (speed is not optimised).
%   Cost (same as the Python node):
%     J(u) = sum_k q_cte*cte_k^2 + q_heading*herr_k^2 + rd_steer*(u_k - u_{k-1})^2
%   where the robot is forward-simulated with the NONLINEAR model
%     x += v_k cos(th) dt,  y += v_k sin(th) dt,  th += v_k/L tan(u_k) dt
%   and compared with reference sample k. Bounds |u_k| <= max_steer.
%   J is a sum of squares, so it is minimised with a projected
%   Levenberg-Marquardt method (nmpc_solve below): the job SLSQP does in the
%   Python node. Warm start = previous solution shifted by one step.
%   mem: u (warm start), prev_steer (set by the caller to the APPLIED steering).
C = P.nmpc;
if ~isfield(mem, 'u')
    mem.u = zeros(C.N, 1); mem.prev_steer = 0;
end
Hz = sample_horizon(ref, err.nearest, C.N + 1, C.dt);
u0 = [mem.u(2:end); atan(P.L * Hz(end, 5))];
% Residual k scores the state AFTER step k, so compare it with the point one step on
u = nmpc_solve(u0, pose, Hz(2:end, :), mem.prev_steer, P);
mem.u = u;
delta = u(1);
v = speed_law(Hz(1, 4), delta, P);
end

function u = nmpc_solve(u, pose, Hz, u_prev, P)
%NMPC_SOLVE  Projected Levenberg-Marquardt on the stacked residual r(u).
%   r = [sqrt(q_cte)*cte_k; sqrt(q_heading)*herr_k; sqrt(rd)*(u_k - u_{k-1})]
%   Jacobian by forward differences (N+1 cheap roll-outs). Each iteration:
%   solve (J'J + lambda*I) du = -J'r, project u + du onto the bounds, accept
%   if the cost drops (lambda /= 3), otherwise reject (lambda *= 5).
C = P.nmpc; N = C.N; lo = -P.max_steer; hi = P.max_steer;
u = min(max(u, lo), hi);
r = residual(u); f = r' * r;
lambda = 1e-2; hstep = 1e-4;
for it = 1:C.maxiter
    J = zeros(numel(r), N);
    for k = 1:N
        uk = u; step = hstep;
        if uk(k) + step > hi, step = -hstep; end
        uk(k) = uk(k) + step;
        J(:, k) = (residual(uk) - r) / step;
    end
    g = J' * r;
    Hm = J' * J;
    improved = false;
    for tries = 1:6
        du = -(Hm + lambda * eye(N)) \ g;
        un = min(max(u + du, lo), hi);
        rn = residual(un); fn = rn' * rn;
        if fn < f
            u = un; r = rn; f0 = f; f = fn; lambda = max(lambda / 3, 1e-6); improved = true;
            break
        end
        lambda = lambda * 5;
    end
    if ~improved || (f0 - f) < 1e-6 * max(f0, 1e-9), break; end
end

    % Nested: shares variables with nmpc_solve, so its locals use their own names.
    function rr = residual(uu)
        % Roll out the rear axle; errors are taken at the tracked point ahead of it
        roff = P.ref_offset;
        th = pose(3); x = pose(1) - roff*cos(th); y = pose(2) - roff*sin(th);
        rr = zeros(3*N, 1);
        up = u_prev;
        for kk = 1:N
            vk = Hz(kk, 4);
            x = x + vk * cos(th) * C.dt;
            y = y + vk * sin(th) * C.dt;
            th = th + vk / P.L * tan(uu(kk)) * C.dt;
            yr = Hz(kk, 3);
            cte = -sin(yr) * (Hz(kk, 1) - (x + roff*cos(th))) + cos(yr) * (Hz(kk, 2) - (y + roff*sin(th)));
            ydes = yr - atan(roff * Hz(kk, 5));
            he = atan2(sin(ydes - th), cos(ydes - th));
            rr(3*kk-2:3*kk) = [sqrt(C.q_cte)*cte; sqrt(C.q_heading)*he; sqrt(C.rd_steer)*(uu(kk) - up)];
            up = uu(kk);
        end
    end
end
