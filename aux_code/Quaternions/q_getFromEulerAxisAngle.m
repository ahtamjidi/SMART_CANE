function q = q_getFromEulerAxisAngle(axis, angle)
%Q_GETFROMEULERAXISANGLE Quaternion from Euler's axis/angle representation.
%   Q = Q_GETFROMEULERAXISANGLE(AXIS, ANGLE) returns the quaternion
%   corresponding to a rotation around the given axis (1x3), by a given
%   angle (in radians).

%   Author: Damien Teney

axis = axis / norm(axis); % Normalize the axis

q = [cos(angle / 2) axis * sin(angle / 2)];
