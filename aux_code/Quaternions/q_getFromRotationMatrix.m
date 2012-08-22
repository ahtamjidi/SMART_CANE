function q = q_getFromRotationMatrix(m)
%Q_GETFROMROTATIONMATRIX Quaternion from rotation matrix.
%   Q= Q_GETFROMROTATIONMATRIX(M) returns the quaternion corresponding to
%   the given rotation matrix.

%   Author: Damien Teney

%det(m) % Should be 1
%inv(m) * m % Should be eye(3)

q = zeros(1, 3); % Empty initialization

q(1) = sqrt(1.0 + m(1, 1) + m(2, 2) + m(3, 3)) / 2.0;
tmp = 4.0 * q(1);
q(2) = (m(3, 2) - m(2, 3)) / tmp;
q(3) = (m(1, 3) - m(3, 1)) / tmp;
q(4) = (m(2, 1) - m(1, 2)) / tmp;

q = q_getUnit(q);
