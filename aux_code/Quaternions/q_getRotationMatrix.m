function rotationMatrix = q_getRotationMatrix(q)
%Q_GETROTATIONMATRIX Transform quaternion to rotation matrix.
%   ROTATIONMATRIX = Q_GETROTATIONMATRIX(Q) returns the 3x3 rotation
%   matrix corresponding to the given quaternion (1x4 vector).
%
%   We assume that the given quaternion has unit norm.

%   Author: Damien Teney

% Rename variables for convenience
w = q(1); x = q(2); y = q(3); z = q(4);
w2 = w^2; x2 = x^2; y2 = y^2; z2 = z^2;
xy = x*y; xz = x*z; yz = y*z;
wx = w*x; wy = w*y; wz = w*z;

rotationMatrix = [w2+x2-y2-z2 2*(xy - wz) 2*(wy + xz)
                  2*(wz + xy) w2-x2+y2-z2 2*(yz - wx)
                  2*(xz - wy) 2*(wx + yz) w2-x2-y2+z2];
