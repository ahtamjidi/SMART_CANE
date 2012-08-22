function q = q_getFromHopfAxisAngle(axis, angle)
%Q_GETFROMAXISANGLE Quaternion from Hopf's axis/angle representation.
%   Q = Q_GETFROMEULERAXISANGLE(AXIS, ANGLE) returns the quaternion
%   corresponding to a rotation around the X axis of a given angle (in
%   radians), followed by a rotation setting the X axis in a given
%   orientation (1x3).

%   Author: Damien Teney

q1 = q_getFromEulerAxisAngle([1 0 0], angle);
q2 = q_getRotationBetweenVectors([1 0 0], axis);

q = q_mult(q2, q1);

q = q_getPositive(q);
