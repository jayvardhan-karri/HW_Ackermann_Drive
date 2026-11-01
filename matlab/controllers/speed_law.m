function v = speed_law(v_ref, delta, P)
%SPEED_LAW  Shared speed command of all controllers (same as the Python nodes):
%   v = v_ref * max(0.4, 1 - 0.5*|delta|/max_steer)   -> half speed at full lock.
v = v_ref * max(0.4, 1 - 0.5 * abs(delta) / P.max_steer);
end
