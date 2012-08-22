function q = q_getFromTwoVectors3(v1, v2)
%Q_GETFROMTWOVECTORS3 Quaternion from two 3-vectors.
%   Q = Q_GETFROMTWOVECTORS3(V1, V2) returns the quaternion that applies V1
%   onto V2 (1x3 vectors).

%   Author: Damien Teney

% Make the vectors orthogonal
%displayVectors3([v1 ; v2]); % Debug display
vMean = (v1 + v2);      vMean  = vMean  ./ norm(vMean);
vCross = cross(v1, v2); vCross = vCross ./ norm(vCross);
v1 = q_rotatePoint(vMean, q_getFromEulerAxisAngle(vCross, -pi / 4));
v2 = q_rotatePoint(vMean, q_getFromEulerAxisAngle(vCross, +pi / 4));
v1 = v1 ./ norm(v1); % Normalize (should be 1, but sometimes not because of numerical errors)
v2 = v2 ./ norm(v2);
assert(isAlmostEqual(dot(v1, v2), 0));
%displayVectors3([v1 ; v2]); % Debug display

% Find the quaternion that applies the X axis to v1
xAxis = [1 0 0];
q1 = q_getRotationBetweenVectors(xAxis, v1);
% Find the quaternion that applies the Y axis to v2
yAxis = q_rotatePoint([0 1 0], q1); % Current position of the Y axis (after transformation by q1)
if yAxis == -v2 % Special case
  q2 = q_getFromEulerAxisAngle(v1, pi); % Easier to do than q_getRotationBetweenVectors, because we already know an orthogonal vector (v1) that can serve as the rotation axis (and we know the angle too, 180deg)
else
  q2 = q_getRotationBetweenVectors(yAxis, v2);
end

q = q_mult(q2, q1);
