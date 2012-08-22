function q = q_getFromEulerAngles(a, b, c)
%Q_GETFROMEULERANGLES Quaternion from Euler angles.
%   Q = Q_GETFROMEULERANGLES(A, B, C) returns the quaternion equivalent to
%   rotations of angles a/b/c (in radians) around the x/y/z axis resp.,
%   applied that order.

%   Author: Damien Teney

qX = [cos(a / 2) sin(a / 2)          0          0];
qY = [cos(b / 2)          0 sin(b / 2)          0];
qZ = [cos(c / 2)          0          0 sin(c / 2)];

q = q_mult(qZ, q_mult(qY, qX));
