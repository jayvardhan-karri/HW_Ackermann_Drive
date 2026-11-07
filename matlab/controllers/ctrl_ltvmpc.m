function [delta, v, mem] = ctrl_ltvmpc(err, ~, ref, mem, P, dt)
%CTRL_LTVMPC  Linear time-varying MPC solved by backward Riccati recursion
%   (MATLAB port of mpc_tracker_node._solve_ltv_mpc, same matrices/tuning).
%   [delta, v, mem] = ctrl_ltvmpc(err, pose, ref, mem, P, dt)
%
%   State  z = [cte; d(cte)/dt; herr; previous steering deviation]
%   1. Sample N reference points ahead (sample_horizon, step P.mpc.dt).
%   2. For k = N..1 build A_k, B_k from (v_k, kappa_k) and run
%        K_k = (R + B'PB)^-1 B'PA,   P = Q + A'P(A - B K_k),   P_N = 3Q
%      (closed form of the unconstrained finite-horizon QP).
%   3. Preview feed-forward = weighted mean of atan(L*kappa_k), k = 1..5.
%   4. delta = ff - K_1 z + ki*integral(cte), clipped; speed from speed_law.
%   mem: prev_cte, I, prev_steer (set by the caller to the APPLIED steering).
C = P.mpc;
if ~isfield(mem, 'I')
    mem.I = 0; mem.prev_cte = err.cte; mem.prev_steer = 0;
end
dcte = (err.cte - mem.prev_cte) / dt;
mem.prev_cte = err.cte;
mem.I = min(max(mem.I + err.cte * dt, -C.i_limit), C.i_limit);

Hz = sample_horizon(ref, err.nearest, C.N, C.dt);
Q = diag([C.q_cte, C.q_dcte, C.q_heading, C.rd_steer]);
Pk = 3 * Q;
R = C.r_steer + C.rd_steer;
Ks = zeros(C.N, 4);
ff = zeros(C.N, 1);
h = C.dt;
for k = C.N:-1:1
    vk = max(Hz(k, 4), 0.15);
    kk = Hz(k, 5);
    ff(k) = atan(P.L * kk);
    b = vk * h * (1 + (P.L*kk)^2) / P.L;
    A = [1, 0.35*h, vk*h, 0;
         0, 0.20,   vk,   0;
         0, 0,      1,    0;
         0, 0,      0,    0];
    B = [-0.5*vk*h*b; -vk*b/h; -b; 1];
    S = R + B' * Pk * B;
    Kk = (B' * Pk * A) / S;
    Ks(k, :) = Kk;
    Pk = Q + A' * Pk * (A - B * Kk);
end
w = [0.45 0.25 0.15 0.10 0.05];
nff = min(5, C.N);
ffp = w(1:nff) * ff(1:nff);
z = [err.cte; dcte; err.herr; mem.prev_steer - ffp];
delta = ffp - Ks(1, :) * z + C.ki * mem.I;
delta = min(max(delta, -P.max_steer), P.max_steer);
v = speed_law(Hz(1, 4), delta, P);
end
