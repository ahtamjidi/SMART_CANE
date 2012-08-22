function q = q_getRotationBetweenVectors(v1, v2)
% Q_GETROTATIONBETWEENVECTORS  Shortest rotation from one vector onto another.

%   Author: Damien Teney

% Get an axis/angle representation of the orientation
if isequal(v1, v2)
  % Special case
  q = [1 0 0 0];
elseif isAlmostEqual(v1, -v2)
  % Special case
  error('Special case not handled in q_getRotationBetweenVectors() !');
else
  % See:
  % http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/index.htm
  q(1) = norm(v1) * norm(v2) + dot(v1, v2);
  %q(1) = sqrt(norm(v1)^2 * norm(v2)^2) + dot(v1, v2); % Same
  q(2:4) = cross(v1, v2);

  q = q ./ norm(q);
end
