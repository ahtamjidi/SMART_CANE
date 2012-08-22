function q = q_mult(q1, q2)
%Q_MULT Multiplication of two quaternions.
%   Q = Q_MULT(Q1, Q2) returns Q so that Q = Q1 * Q2.

%   Author: Damien Teney

q = zeros(1, 4);

q(1) = q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
q(2) = q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
q(3) = q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
q(4) = q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1);
