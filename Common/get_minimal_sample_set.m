function [s, Theta_hat] = get_minimal_sample_set(k, X, P_s, est_fun, validateMSS_fun, ind_tabu)

% [s, Theta_hat] = get_minimal_sample_set(k, X, P_s, est_fun, validateMSS_fun, ind_tabu)
%
% DESC:
% select the minimal sample set using different sampling strategies
%
% VERSION:
% 1.0.5
%
% INPUT:
% k                 = minimal sample set cardinality
% X                 = input data
% P_s               = sampling probabilities
% est_fun           = function that estimates Theta. Should be in the form of:
%
%                     Theta = est_fun(X, s)
%
% validateMSS_fun     = function that validates a MSS. Should be in the form of:
%
%                     flag = validateMSS_foo(X, s)
%
% ind_tabu          = indices of elements excluded from the sample set
%
% OUTPUT:
% s                 = minimal sample set
% Theta_hat         = estimated parameter vector on s


% AUTHOR:
% Marco Zuliani, email: marco.zuliani@gmail.com
% Copyright (C) 2009 by Marco Zuliani 
% 
% LICENSE:
% This toolbox is distributed under the terms of the GNU GPL.
% Please refer to the files COPYING.txt for more information.


% HISTORY:
%
% 1.0.0             - ??/??/05 - Initial version
% 1.0.1             - ??/??/05 - Added probabilistic sampling
% 1.0.2             - 06/25/08 - Handles seed of the random number
%                     generator
% 1.0.3             - 11/17/08 - Handles the MSS validation function
% 1.0.4             - 03/27/09 - Fixed a bug in ind_tabu. Thanks to Chris
%                     Volpe
% 1.0.5             - 06/21/09 - Added further check. Thanks to Chris
%                     Volpe

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check input parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin < 3
    P_s = [];
end

if nargin < 4
    est_fun = [];
end

if nargin < 5
    validateMSS_fun = [];
end

if nargin < 6
    ind_tabu = [];
end

N = size(X, 2);

% remove tabu elements (if any)
ind = 1:N;
if ~isempty(ind_tabu)
    ind = ind(~ind_tabu);
    NN = length(ind);

    % check if we are left with enough elements for finding a MSS
    if (NN < k)

        error('RANSACToolbox:get_minimal_sample_set', ...
            'Too few input input elements after removing tabu elements');
    end;
end;

% get the seed for the random number generator
global RANSAC_TWISTER_STATE_UPDATED;

% if we have an estimation function then loop until the MSS actually
% produces an estimate
while true

    % update the seed
    if ~isempty(RANSAC_TWISTER_STATE_UPDATED)
        RANSAC_TWISTER_STATE_UPDATED = RANSAC_TWISTER_STATE_UPDATED + 1;
    end;

    % uniform sampling
    if isempty(P_s)
        % uniform sampling
        mask = get_rand(k, N, RANSAC_TWISTER_STATE_UPDATED);
    else
        % probabilistic sampling
        mask = get_rand_prob(k, P_s(ind), RANSAC_TWISTER_STATE_UPDATED);
    end;

    s = ind(mask);
        
    % check if we are done
    if isempty(est_fun)
        Theta_hat = [];
        break;
    end;
    
    % validate the MSS
    if ~isempty(validateMSS_fun) && ~feval(validateMSS_fun, X, s)
        continue;
    end;

    % estimate the parameter and the residual error
    Theta_hat = feval(est_fun, X, s);

    % verify that the estimation produced something
    if ~isempty(Theta_hat)
        break;
    end;

end;

return