function [a b c] = q_getEulerAngles(q)
%Q_GETEULERANGLES Euler angles from quaternion.
%   [A B C] = Q_GETEULERANGLES(Q) returns the angles (in radians) a/b/c
%   corresponding to rotations around the x/y/z axis resp., which, applied
%   in that order, correspond to the given quaternion Q.

%   Author: Damien Teney

a = atan2(2 * q(1) * q(2) + 2 * q(3) * q(4), ...
          1 - 2 * q(2)^2 - 2 * q(3)^2);
b = asin(2 * q(1) * q(3) - 2 * q(2) * q(4));
c = atan2(2 * q(1) * q(4) + 2 * q(2) * q(3), ...
          1 - 2 * q(3)^2 - 2 * q(4)^2);
