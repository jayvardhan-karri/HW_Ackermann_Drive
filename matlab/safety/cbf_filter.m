function [v, delta, clearance, status, side] = cbf_filter(v_nom, delta_nom, pose, obstacles, P, side)
%CBF_FILTER  Control Barrier Function safety filter on known obstacles (no LiDAR).
%   MATLAB port of obstacle_cbf.CbfFilter.filter in the ROS package.
%   [v, delta, clearance, status, side] = cbf_filter(v_nom, delta_nom, pose, obstacles, P, side)
%     pose       [x y yaw] of the tracked point (P.ref_offset ahead of the rear axle)
%     obstacles  [x y r] rows, same frame as pose (cylinders)
%     side       committed passing side per obstacle: start with [], pass back each call
%
%   For the tracked point, with h = |p - o| - R and R = r + robot_radius + margin:
%     barrier : (d.t) v + a (d.n) omega + alpha h >= 0     (half-plane in v, omega)
%     input   : 0 <= v <= v_nom, |omega| <= kappa_max v    (Ackermann: no turn on the spot)
%     approach: within trigger_distance(R), if the heading is inside the obstacle's
%               collision cone, turn to the committed side (slowing while turning hard).
%               A distance barrier alone reacts too late for a robot that cannot turn
%               on the spot: it would arrive at the boundary pointing inwards and stop.
%   Safe command = closest point of the safe polygon to the reference (closed form).
%   status: NO_OBSTACLES | CLEAR | CBF_ACTIVE | CBF_STOP; clearance = distance from the
%   tracked point to the nearest cylinder edge (m).
C = P.cbf;
kappa_max = tan(C.max_steer) / P.L;
v = v_nom; delta = delta_nom;
n = size(obstacles, 1);
if isempty(side), side = zeros(n, 1); end
if numel(side) ~= n, side = zeros(n, 1); end
if n == 0
    clearance = inf; status = 'NO_OBSTACLES'; return
end
clearance = min(hypot(pose(1) - obstacles(:, 1), pose(2) - obstacles(:, 2)) - obstacles(:, 3));
if v_nom <= 1e-4
    status = 'CLEAR'; return
end
c = cos(pose(3)); s = sin(pose(3));
w_nom = v_nom * tan(delta_nom) / P.L;
w_ref = w_nom; v_ref = v_nom; worst = 0;
cons = zeros(0, 3);
rho = 1 / (0.85 * kappa_max);
for i = 1:n
    dx = pose(1) - obstacles(i, 1); dy = pose(2) - obstacles(i, 2);
    dist = max(hypot(dx, dy), 1e-6);
    R = obstacles(i, 3) + C.robot_radius + C.safety_margin;
    h = dist - R;
    if h > sqrt(R^2 + 2*R*rho) - R + 0.25    % turning-radius trigger + servo-lag buffer
        side(i) = 0; continue
    end
    cons(end+1, :) = [(dx*c + dy*s)/dist, P.ref_offset*(-dx*s + dy*c)/dist, C.alpha*h]; %#ok<AGROW>
    bearing = wrap_angle(atan2(-dy, -dx) - pose(3));   % obstacle direction vs heading
    if side(i) == 0                                    % away from it; dead ahead: nominal's way
        if abs(bearing) > 0.02
            side(i) = -sign(bearing);
        elseif w_nom >= 0
            side(i) = 1;
        else
            side(i) = -1;
        end
    end
    violation = 0;
    if abs(bearing) < pi/2
        violation = asin(min(1, R/dist)) + side(i)*bearing;
    end
    if violation > worst
        worst = violation;
        strength = min(1, violation/0.25);
        w_turn = side(i)*kappa_max*v_nom*strength;
        if side(i) > 0, w_ref = max(w_nom, w_turn); else, w_ref = min(w_nom, w_turn); end
        v_ref = v_nom*(1 - 0.5*strength);
    end
end
if isempty(cons)
    status = 'CLEAR'; return
end
if worst <= 0 && all(cons(:,1)*v_nom + cons(:,2)*w_nom + cons(:,3) >= 0)
    status = 'CLEAR'; return
end
u = project_safe(v_ref, w_ref, v_nom, cons, kappa_max, C.steer_weight);
if isempty(u) || u(1) < 1e-3
    v = 0; status = 'CBF_STOP'; return
end
v = u(1);
delta = min(max(atan(P.L*u(2)/u(1)), -C.max_steer), C.max_steer);
status = 'CBF_ACTIVE';
end

function u = project_safe(v_ref, w_ref, v_max, cons, kappa_max, weight)
%PROJECT_SAFE  Closest point (cost (v-v_ref)^2 + weight (omega-w_ref)^2) in the triangle
%   (0,0), (v_max, +-kappa_max v_max) clipped by every half-plane A v + B omega + C >= 0.
k = sqrt(weight);                                      % scaled coordinates: (v, k omega)
poly = [0 0; v_max -k*kappa_max*v_max; v_max k*kappa_max*v_max];
for i = 1:size(cons, 1)
    poly = clip_poly(poly, cons(i,1), cons(i,2)/k, cons(i,3));
    if isempty(poly), u = []; return; end
end
ref = [v_ref, k*w_ref];
if size(poly, 1) >= 3 && inpolygon(ref(1), ref(2), poly(:,1), poly(:,2))
    best = ref;
else
    best = poly(1, :); dbest = inf; m = size(poly, 1);
    for j = 1:m
        a = poly(j, :); e = poly(mod(j, m) + 1, :) - a;
        t = 0; if e*e' > 0, t = max(0, min(1, ((ref - a)*e')/(e*e'))); end
        q = a + t*e; dq = (q - ref)*(q - ref)';
        if dq < dbest, best = q; dbest = dq; end
    end
end
u = [best(1), best(2)/k];
end

function out = clip_poly(poly, a, b, c)
%CLIP_POLY  Convex polygon clipped by a x + b y + c >= 0 (Sutherland-Hodgman).
out = zeros(0, 2); m = size(poly, 1);
for j = 1:m
    p = poly(j, :); q = poly(mod(j, m) + 1, :);
    fp = a*p(1) + b*p(2) + c; fq = a*q(1) + b*q(2) + c;
    if fp >= 0, out(end+1, :) = p; end %#ok<AGROW>
    if (fp >= 0) ~= (fq >= 0)
        out(end+1, :) = p + fp/(fp - fq)*(q - p); %#ok<AGROW>
    end
end
end
