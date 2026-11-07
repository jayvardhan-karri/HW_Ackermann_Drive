function H = sample_horizon(ref, nearest, N, dt)
%SAMPLE_HORIZON  Reference rows the robot should reach at the next N steps.
%   H = sample_horizon(ref, nearest, N, dt)  -> N x 5 [x y yaw v kappa]
%   Starting at the nearest waypoint, each step advances by the arc length
%   driven in dt at the local reference speed: idx += v*dt / ds. The index
%   is fractional so slow segments can repeat a waypoint; it wraps on a
%   closed loop and stops at the last point of an open path (ref_index).
H = zeros(N, 5);
c = nearest;
for k = 1:N
    i = ref_index(ref, c);
    j = ref_index(ref, i + 1);
    ds = max(hypot(ref.x(j) - ref.x(i), ref.y(j) - ref.y(i)), 1e-3);
    H(k, :) = ref.W(i, :);
    c = c + ref.v(i) * dt / ds;
end
end
