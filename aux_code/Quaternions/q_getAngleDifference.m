function angle = q_getAngleDifference(q1, q2)
%Q_GETANGLEDIFFERENCE Shortest angle between quaternions.
%   ANGLE = Q_GETANGLEDIFFERENCE(Q1, Q2) returns the angle (in radians) of
%   the shortest transformation between Q1 and Q2.

%   Author: Damien Teney

if isequal(q1, q2) || isequal(q1, -q2)
  % Special case
  angle = 0;
else
  dotProduct = abs(dot(q1,  q2)); % Take the absolute value to ensure we keep the shortest arc

  %assert(dotProduct <= 1.01), assert(dotProduct >= -1.01); % Allow some numerical errors
  % Correct numerical errors
  if dotProduct > 1
    dotProduct = 1;
  elseif dotProduct < -1
    dotProduct = -1;
  end
    
  angle = 2.0 * acos(dotProduct);
end
