function K = solve_lqr_gain(v, kappa, P)
%SOLVE_LQR_GAIN  Continuous LQR gain for the 2-state path-error model.
%   K = solve_lqr_gain(v, kappa, P)  -> 1x2 gain, law u = -K*[cte; herr]
%
%   Error model (linearised about the feed-forward steering):
%     d/dt [cte; herr] = A [cte; herr] + B (delta - delta_ff)
%     A = [0 v; 0 0],  B = [0; -v*(1 + (L*kappa)^2)/L]     (physical sign)
%   Q = diag(q_cte, q_heading), R = r_steer.
%   Uses LQR from the Control System Toolbox when available, otherwise the
%   built-in Hamiltonian/Schur solver below (base MATLAB).
%   (The Python node writes B with a + sign and applies +K; that is the same
%   closed loop, see Theory_Explanation section 5.2.)
v = max(v, 0.05);                     % B -> 0 as v -> 0 (not stabilisable)
A = [0 v; 0 0];
B = [0; -v * (1 + (P.L*kappa)^2) / P.L];
Q = diag([P.lqr.q_cte, P.lqr.q_heading]);
R = P.lqr.r_steer;
persistent have_cst
if isempty(have_cst)
    have_cst = exist('lqr', 'file') == 2 && license('test', 'Control_Toolbox');
end
if have_cst
    K = lqr(A, B, Q, R);
else
    K = care_schur(A, B, Q, R);
end
end

function K = care_schur(A, B, Q, R)
%CARE_SCHUR  Solve A'X + XA - XBR^-1B'X + Q = 0 via the stable invariant
%   subspace of the Hamiltonian matrix (ordered real Schur form).
n = size(A, 1);
H = [A, -B*(R\B'); -Q, -A'];
[U, S] = schur(H, 'real');
[U, ~] = ordschur(U, S, 'lhp');
X = U(n+1:end, 1:n) / U(1:n, 1:n);
X = (X + X') / 2;
K = R \ (B' * X);
end
