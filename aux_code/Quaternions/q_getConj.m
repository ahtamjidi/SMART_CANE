function q2 = q_getConj(q)
%Q_GETCONJ Quaternion conjugate.

%   Author: Damien Teney

q2 = q;
q2(2:4) = -q(2:4);
