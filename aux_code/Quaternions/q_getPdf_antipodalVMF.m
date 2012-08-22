function p = q_getPdf_antipodalVMF(q, mu, kappa)
%Q_GETPDF_ANTIPODALVMF Probability in a pair of antipodal von Mises-Fisher distributions.
%   P = Q_GETPDF_ANTIPODELVMF(Q, MU, KAPPA) returns the probability density
%   of a unit quaternion (1x4 vector) in a pair of antipodal von Mises-
%   Fisher distributions of parameters MU (1x4 vector) and KAPPA (scalar).
%
%   Q can contain several quaternions (Nx4), P is then of dimension 1xN.
%
%   Q and MU must be of unit length.

%   Author: Damien Teney

p1 = q_getPdf_VMF(q, mu, kappa);
p2 = q_getPdf_VMF(q, -mu, kappa);

p = (p1 + p2) / 2;
