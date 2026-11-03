function [delta, v, mem] = ctrl_lqr(err, ~, ~, mem, P, ~)
%CTRL_LQR  Gain-scheduled infinite-horizon LQR on (cte, herr).
%   [delta, v, mem] = ctrl_lqr(err, pose, ref, mem, P, dt)
%
%   Each call: K = solve_lqr_gain(v_ref, kappa) at the look-ahead point
%   (memoised on v rounded to 1 mm/s and kappa to 0.01 1/m), then
%     delta = atan(L*kappa) - K*[cte; herr],  clipped to +/- max_steer.
%   mem: gain cache (containers.Map).
if ~isfield(mem, 'cache'), mem.cache = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
vr = round(err.la(4), 3); kr = round(err.la(5), 2);
key = sprintf('%.3f_%.2f', vr, kr);
if isKey(mem.cache, key)
    K = mem.cache(key);
else
    if mem.cache.Count >= 5000, remove(mem.cache, keys(mem.cache)); end
    K = solve_lqr_gain(vr, kr, P);
    mem.cache(key) = K;
end
ff = atan(P.L * err.la(5));
delta = ff - K * [err.cte; err.herr];
delta = min(max(delta, -P.max_steer), P.max_steer);
v = speed_law(err.la(4), delta, P);
end
