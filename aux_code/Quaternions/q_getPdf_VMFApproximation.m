function p = q_getPdf_VMFApproximation(q, mu, kernelHalfSize)
%Q_GETPDF_VMFAPPROXIMATION Approximation of Q_GETPDF_VMF().
%   See Q_GETPDF_VMF().
%
%   Approximation of the kernel size: kernelHalfSize = (8 / kappa)

%   Author: Damien Teney

innerProduct = mu * q'; % Arccos of the angle between the two quaternions
distanceToMean = acos(innerProduct);
%distanceToMean = (pi / 2) - (innerProduct * pi / 2); % Linear approximation of the arccos

if distanceToMean < kernelHalfSize;
  p = distanceToMean / kernelHalfSize;
else
  p = 0;
end
