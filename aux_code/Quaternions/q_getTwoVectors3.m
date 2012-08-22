function [v1 v2] = q_getTwoVectors3(q)
%Q_GETTWOVECTORS3 Quaternion to 2 (orthogonal, unit length) 3-vectors.
%   [V1 V2] = Q_GETTWOVECTORS3(q) returns two (orthogonal, unit length)
%   vectors, defining a unique quaternion.

%   Author: Damien Teney

v1 = q_rotatePoint([1 0 0], q);
v2 = q_rotatePoint([0 1 0], q);
