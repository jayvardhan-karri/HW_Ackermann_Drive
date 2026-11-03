function i = ref_index(ref, i)
%REF_INDEX  Map any (1-based) waypoint index onto the table.
%   Closed loop: wraps around (mod). Open path: clamps to [1, n].
%   MATLAB port of TrajectorySubscriber.index().
i = round(i);
if ref.closed
    i = mod(i - 1, ref.n) + 1;
else
    i = min(max(i, 1), ref.n);
end
end
