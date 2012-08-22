function q = q_getRandom_VMF(mu, kappa)
% Q_GETRANDOM_VMF   Random quaternion from von Mises-Fisher distribution.
%    q = q_getRandom_VMF(mu, kappa) returns a unit quaternion distributed
%    on the 4-sphere according to a von Mises-Fisher distribution. 'mu' is
%    the mean, a quaternion (1x4), kappa is the concentration parameter (a
%    scalar).

%   Author: Stephen J. Sangwine, Nicolas Le Bihan (Matlab quaternion,
%   toolbox), modified by Damien Teney

% Special case
if kappa == +inf
  q = mu;
  return;
end

b = -kappa + sqrt(kappa^2 + 1);
x0 = (1 - b) / (1 + b);
c = kappa * x0 + 2 * log(1 - x0^2);

while true
  z = beta();
  u = rand();
  w = (1 - (1 + b) * z) / (1 - (1 - b) * z);
  t = kappa * w + 2 * log(1 - x0 * w) - c;
  if t >= log(u)
    break;
  end
end

% We now need to create a point on the unit 3-sphere, uniformly
% distributed. This is done with the v3_getRandom() function. We then
% combine this value with w, keeping the modulus to unity, and we have the
% result we need, apart from a final rotation.

q = zeros(1, 4);
q(1) = w;
q(2:4) = sqrt(1 - w^2) * v3_getRandom();

% q's mean direction is now (1,0,0,0); we rotate it to mu
q = q_mult(mu, q);

%==========================================================================

function z = beta()
% Generate a random value with the beta(a, a) distribution with a = 1.5.
% (1.5 because the parameters for beta are (m - 1)/2 and m = 4.

while true
  u = getRandom_uniform(-1, 1);
  v = getRandom_uniform(0, 1);

  s = u^2 + v^2;

  if s <= 1
    break;
  end
end

z = 0.5 + u * v * sqrt(1 - s) / s;

%==========================================================================
