function P = lap_defaults(P)
%LAP_DEFAULTS  Settings for the lap study.
P.max_steer = 0.4084;   % centre angle the driver allows (rad); see docs/README.md
P.cbf.max_steer = P.max_steer;
P.sim.start_offset = [0 0.03 deg2rad(3)];
end
