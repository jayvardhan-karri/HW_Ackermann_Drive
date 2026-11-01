function pose = bicycle_step(pose, v, delta, L, dt)
%BICYCLE_STEP  One integration step of the kinematic bicycle model (RK2).
%   pose = [x y yaw];  x' = v cos(yaw), y' = v sin(yaw), yaw' = v/L tan(delta)
%   Midpoint rule: the heading at half the step is used for the translation.
w = v / L * tan(delta);
ym = pose(3) + 0.5 * w * dt;
pose = [pose(1) + v*cos(ym)*dt, pose(2) + v*sin(ym)*dt, pose(3) + w*dt];
end
