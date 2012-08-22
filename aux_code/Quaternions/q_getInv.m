function q2 = q_getInv(q)
%Q_GETINV Inverse of quaternion.

%   Author: Damien Teney

q2 = q_getConj(q) ./ q_getSquaredModulus(q);
