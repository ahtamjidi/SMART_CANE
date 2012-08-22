function rotatedPoint = q_rotatePoint(point, q)
%Q_ROTATEPOINT Point rotation.
%   ROTATEDPOINT = Q_ROTATEPOINT(POINT, Q) rotates the given point (1x3
%   vector) by the rotation contained in the quaternion q (1x4 vector).
%
%   The returned point is also a 1x3 vector.

%   Author: Damien Teney

% Rename the elements of the parameters
Qw = q(1);  Qi = q(2);  Qj = q(3);  Qk = q(4);
v1 = point(1);  v2 = point(2);  v3 = point(3);

% Perform the computations
t2  =  Qw*Qi;
t3  =  Qw*Qj;
t4  =  Qw*Qk;
t5  = -Qi*Qi;
t6  =  Qi*Qj;
t7  =  Qi*Qk;
t8  = -Qj*Qj;
t9  =  Qj*Qk;
t10 = -Qk*Qk;
rotatedPointX = 2 * ( (t8 + t10)*v1 + (t6 -  t4)*v2 + (t3 + t7)*v3 ) + v1;
rotatedPointY = 2 * ( (t4 +  t6)*v1 + (t5 + t10)*v2 + (t9 - t2)*v3 ) + v2;
rotatedPointZ = 2 * ( (t7 -  t3)*v1 + (t2 +  t9)*v2 + (t5 + t8)*v3 ) + v3;

% Put the result in the output vector
rotatedPoint = [rotatedPointX rotatedPointY rotatedPointZ];
