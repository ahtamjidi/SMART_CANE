function [axis angle] = q_getHopfAxisAngle(q)
%Q_GETHOPFAXISANGLE Quaternion to Hopf's axis/angle representation.
%   [AXIS ANGLE] = Q_GETEULERAXISANGLE(AXIS, ANGLE) returns Hopf's axis
%   (1x3) / angle (in radians) representation of a given quaternion (1x4).

%   Author: Damien Teney

%--------------------------------------------------------------------------
% Fast code
%--------------------------------------------------------------------------
% Do the following operation:
%axis = q_rotatePoint([1 0 0], q);
axis(1) = 2 * -(q(3)*q(3) + q(4)*q(4)) + 1;
axis(2) = 2 *  (q(1)*q(4) + q(2)*q(3));
axis(3) = 2 *  (q(2)*q(4) - q(1)*q(3));

% Do the following operation:
% q2 = q_getRotationBetweenVectors([1 0 0], axis);
if      isAlmostEqual(axis, [1 0 0])
  q2 = [1 0 0 0];
elseif isAlmostEqual(axis, [-1 0 0])
  error('Special case not supported !');
else
  xAxis = [1 0 0];

  % See:
  % http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/index.htm
  % Heavy simplifications are possible since the first vector is [1 0 0]
  q2(1) = 1 + axis(1);
  q2(2) = 0;
  q2(3) = -axis(3);
  q2(4) =  axis(2);

  % Normalize q2
  q2 = q2 ./ norm(q2);
end

q1 = q_mult(q_getInv(q2), q);

% q1 is a rotation around [1 0 0], get the angle of that rotation
% [xAxis angle] = q_getEulerAxisAngle(q1)
if q1(2) >= 0
  angle = 2 * acos(q1(1));
else
  angle = -2 * acos(q1(1)) + 2 * pi;
end

% Debug check
%{
xAxis(1) = q1(2) / sqrt(1 - q1(1) * q1(1));
xAxis(2) = q1(3) / sqrt(1 - q1(1) * q1(1));
xAxis(3) = q1(4) / sqrt(1 - q1(1) * q1(1));
if ~isAlmostEqual(xAxis, [1 0 0])
  xAxis
  error('q_getHopfAxisAngle failed !'); % May fail due to numerical errors
end
%}

%--------------------------------------------------------------------------
% Easy to read code
%--------------------------------------------------------------------------
%{
axis = q_rotatePoint([1 0 0], q);

q2 = q_getRotationBetweenVectors([1 0 0], axis);

q1 = q_mult(q_getInv(q2), q);

% q1 is a rotation around [1 0 0], get the angle of that rotation
%[xAxis angle] = q_getEulerAxisAngle(q1);
if q1(2) >= 0
  angle = 2 * acos(q1(1));
else
  angle = -2 * acos(q1(1)) + 2 * pi;
end

%assert(isAlmostEqual(xAxis, [1 0 0])); % Often fails due to numerical errors
%}