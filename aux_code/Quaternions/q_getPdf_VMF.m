function p = q_getPdf_VMF(q, mu, kappa)
%Q_GETPDF_VMF Probability in von Mises-Fisher distribution.
%   P = Q_GETPDF_VMF(Q, MU, KAPPA) returns the probability density of a
%   unit quaternion (1x4 vector) in a von Mises-Fisher distribution of
%   parameters MU (1x4 vector) and KAPPA (scalar).
%
%   Q can contain several quaternions (Nx4), P is then of dimension 1xN.
%
%   Q and MU must be of unit length.

%   Author: Damien Teney

% Normalize input quaternions
%mu = q_getUnit(mu);
%q = q_getUnit(q);

% Get the normalization constant
c = 1 / (exp(1)^kappa - exp(1)^(-kappa)); % Not correct, but smaller values
%c = kappa / (2*pi) * 1 / (exp(1)^kappa - exp(1)^(-kappa)); % Good
%p = 3, c = (1/(((2*pi)^(p/2))*besseli((p/2)-1, kappa)))*(kappa^((p/2)-1)); % Good too

% Get the probability
p = c * exp(kappa * (mu * q'));
