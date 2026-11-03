function m = compute_metrics(log)
%COMPUTE_METRICS  Tracking metrics (same definitions as compare_results.py).
%   m.rms_cte, m.max_cte (m), m.rms_heading (rad), m.rms_steer_rate (rad/s,
%   RMS of d(delta)/dt between control ticks), m.mean_solve_ms, m.duration (s),
%   m.goal (open path reached its end).
dt = max(diff(log.t), 1e-3);
m.rms_cte = sqrt(mean(log.cte.^2));
m.max_cte = max(abs(log.cte));
m.rms_heading = sqrt(mean(log.herr.^2));
m.rms_steer_rate = sqrt(mean((diff(log.delta) ./ dt).^2));
m.mean_solve_ms = mean(log.solve_ms);
m.duration = log.t(end);
m.goal = log.goal;
end
