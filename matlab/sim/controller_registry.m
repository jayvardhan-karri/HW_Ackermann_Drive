function c = controller_registry(name, P)
%CONTROLLER_REGISTRY  Look up a controller by name.
%   c = controller_registry(name, P) with name = 'pid' | 'lqr' | 'ltvmpc' | 'nmpc'
%   returns  c.name, c.step (function handle), c.rate_hz, c.lookahead, c.label.
%   Every step function has the signature
%       [delta, v, mem] = step(err, pose, ref, mem, P, dt)
%   so the simulation and the ROS runner treat all controllers alike.
switch lower(name)
    case 'pid',    c = mk('pid',    @ctrl_pid,    P.pid,  'PID');
    case 'lqr',    c = mk('lqr',    @ctrl_lqr,    P.lqr,  'LQR');
    case 'ltvmpc', c = mk('ltvmpc', @ctrl_ltvmpc, P.mpc,  'LTV-MPC');
    case 'nmpc',   c = mk('nmpc',   @ctrl_nmpc,   P.nmpc, 'NMPC');
    otherwise, error('controller_registry:unknown', 'Unknown controller "%s"', name);
end
end

function c = mk(name, fh, C, label)
c = struct('name', name, 'step', fh, 'rate_hz', C.rate_hz, 'lookahead', C.lookahead, 'label', label);
end
