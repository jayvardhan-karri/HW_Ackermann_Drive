function a = wrap_angle(a)
%WRAP_ANGLE  Wrap angle(s) to (-pi, pi].
a = atan2(sin(a), cos(a));
end
