function [axis angle] = q_getEulerAxisAngle(q)
%Q_GETEULERAXISANGLE Quaternion to Euler's axis/angle representation.
%   [AXIS ANGLE] = Q_geteuleraxisangle(Q) returns Euler's axis (1x3) /
%   angle (in radians) representation of a given quaternion (1x4).

%   Author: Damien Teney

angle = 2 * acos(q(1));
axis(1) = q(2) / sqrt(1 - q(1) * q(1));
axis(2) = q(3) / sqrt(1 - q(1) * q(1));
axis(3) = q(4) / sqrt(1 - q(1) * q(1));
