%-----------------------------------------------------------------------
% 1-point RANSAC EKF SLAM from a monocular sequence
%-----------------------------------------------------------------------

% Copyright (C) 2010 Javier Civera and J. M. M. Montiel
% Universidad de Zaragoza, Zaragoza, Spain.

% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation. Read http://www.gnu.org/copyleft/gpl.html for details

% If you use this code for academic work, please reference:
%   Javier Civera, Oscar G. Grasa, Andrew J. Davison, J. M. M. Montiel,
%   1-Point RANSAC for EKF Filtering: Application to Real-Time Structure from Motion and Visual Odometry,
%   to appear in Journal of Field Robotics, October 2010.

%-----------------------------------------------------------------------
% Authors:  Javier Civera -- jcivera@unizar.es
%           J. M. M. Montiel -- josemari@unizar.es

% Robotics, Perception and Real Time Group
% Arag�n Institute of Engineering Research (I3A)
% Universidad de Zaragoza, 50018, Zaragoza, Spain
% Date   :  May 2010
%-----------------------------------------------------------------------

function [ X_km1_k, P_km1_k ] = predict_state_and_covariance( X_k, P_k, type, SD_A_component_filter, SD_alpha_component_filter )
persistent time_stamp
global myCONFIG

if isempty(time_stamp)
    load([myCONFIG.PATH.SOURCE_FOLDER,'/TimeStamp/TimeStamp.mat'],'time_stamp')
end

delta_t = 0.1;
% delta_t = 1/10;
% delta_t = 0.1205;
global step_global
% camera motion prediction
% [Xv_km1_k,v_for_noise,w_for_noise,dq__,dX__] = fv( X_k(1:13,:), delta_t, type, SD_A_component_filter, SD_alpha_component_filter  );
destination_folder = myCONFIG.PATH.KEYFRAMES_FOLDER;
destination_velocity_file1 = sprintf('%s/vel_%04d.mat',[destination_folder,'VelocityData'],step_global-1);
destination_velocity_file2 = sprintf('%s/vel_%04d.mat',[destination_folder,'VelocityData'],step_global);
  [dX__,dq__,R,State_RANSAC]=Calculate_V_Omega_RANSAC_dr_ye_key(step_global-2,step_global-1);
load(destination_velocity_file1,'current_v','current_w','time_elapsed');

time_elapsed_1 = time_elapsed;


load(destination_velocity_file2,'time_elapsed');

time_elapsed_2 = time_elapsed;
if time_elapsed_1 == 0 
    time_elapsed_1 = time_elapsed_2; %% just for the first step
end
current_v_ = dX__/time_elapsed_1;
[a_,u_] = q2au(dq__);
current_w_ = (a_/time_elapsed_1)*u_;


% previous_time_difference = ((time_stamp(1,step_global-1) - time_stamp(1,step_global-2) )/1000);
% if previous_time_difference<0.001
%     previous_time_difference=0.001;
%     disp('Time stamp difference 0 ');
% end
% 
% current_time_difference = ((time_stamp(1,step_global) - time_stamp(1,step_global-1) )/1000);
% if current_time_difference<0.001
%     current_time_difference=0.001;
%     disp('Time stamp difference 0 ');
% end
dX__1 = current_v*time_elapsed;
dq__1 = v2q(current_w*time_elapsed)';
dq__1 = dq__1/norm(dq__1);

dX__ = current_v_*time_elapsed;
dq__ = v2q(current_w_*time_elapsed)';
dq__ = dq__/norm(dq__);
  [dX__2,dq__2,R,State_RANSAC]=Calculate_V_Omega_RANSAC_dr_ye_key(step_global-1,step_global);
  [dX__,dq__,R,State_RANSAC]=Calculate_V_Omega_RANSAC_dr_ye_key(step_global-2,step_global-1);
u =[dX__;dq__];
[Xv_km1_k, Xo_x, Xo_u] = odometry_model(X_k(1:13,:), u, time_elapsed);
dX__-dX__2
R2e((q2R(dq__2)*q2R(dq__))')*180/pi






% v__ = dX__/previous_time_difference;
% 
% [a__,u__]=q2au(dq__);
% angular_velocity = (a__/previous_time_difference)*u__;
% 
% 
% 
% u = [v__-X_k(8:10);angular_velocity-X_k(11:13)];
% [Xv_km1_k, Xo_x, Xo_u] = constVel(X_k(1:13,:), u, current_time_difference);
% 

% features prediction
X_km1_k = [ Xv_km1_k;[0 0 0 0 0 0]'; X_k( 14:end,: ) ];

% state transition equation derivatives
% F = sparse( dfv_by_dxv( X_k(1:13,:),zeros(6,1),delta_t, type ) );
F = [Xo_x,zeros(7,6);
    zeros(6,7) eye(6)];
G = [Xo_u;...
    zeros(6,7)];
% state noise
%% original motion model uncertainty (symetric)
% linear_acceleration_noise_covariance = (SD_A_component_filter*current_time_difference)^2;
% angular_acceleration_noise_covariance = (SD_alpha_component_filter*current_time_difference)^2;
% dX_global = q2R(X_k(4:7))*dX__;
% dX_global_noise = [0.1; 0.01 ;0.1].*dX_global/2;
% dX_local_noise =  q2R(X_km1_k(4:7))'*dX_global_noise;
% cov_dX = diag((dX_local_noise).^2);
% [dq_noise,Qe] = e2q((q2e(dq__)));
% cov_dq = Qe*diag((0.1*q2e(dq__)).^2)*Qe';

% cov_dX = q2R(X_k(4:7))'*diag((0.005/2*[1,0.5,1]).^2);
% cov_dX = diag(((1/3)*[0.1;0.1;0.1].*dX__).^2);
% % [dq_noise,Qe] = e2q((0.24/2*pi/180*[1,0.1,1]));
% % cov_dq = Qe*diag((0.24/2*pi/180*[1,0.1,1]).^2)*Qe';

% error_euler = q2e(dq__);

% [dq_noise,Qe] = e2q(0.5*q2e(dq__)/2);
% cov_dq = Qe*diag((0.5*q2e(dq__)/2).^2)*Qe';

cov_dX = diag((0.01/(2*3)*[1,1,1]).^2);
[dq_noise,Qe] = e2q((0.24/2*pi/180*[0,0,0]));
cov_dq = Qe*diag((0.24/(2)*pi/180*[1,1,1]).^2)*Qe';

% [dq_noise,Qe] = e2q((0.24/2*pi/180*[1,1,1]));
% cov_dq = Qe*diag((0.24/2*pi/180*[1,1,1]).^2)*Qe';
% 
% cov_dX = diag((0.01/2*[1,1,1]).^2);
% a1 = max((1/3)*0.1*dX__(1),0.01/2);
% a2 = max((1/3)*0.1*dX__(2),0.01/2);
% a3 = max((1/3)*0.1*dX__(3),0.01/2);
% cov_dX = diag(([a1,a2,a3]).^2);



Pn = [cov_dX ,    zeros(3,4);...
      zeros(4,3), cov_dq   ];
  
Pn = bootstrap_cov_calc(step_global-2,step_global-1);
  
% Pn = sparse (diag( [linear_acceleration_noise_covariance linear_acceleration_noise_covariance linear_acceleration_noise_covariance...
%     angular_acceleration_noise_covariance angular_acceleration_noise_covariance angular_acceleration_noise_covariance] ) );

% v_norm = (v_for_noise+1)/norm((v_for_noise+1));
% w_norm = (w_for_noise+1)/norm((w_for_noise+1));

% v_norm = (v_for_noise+1)/norm((v_for_noise+1));
% w_norm = (w_for_noise+1)/norm((w_for_noise+1));
% u
% 


% Pn = calc_cov_RANSAC_dr_ye(step_global-2,step_global-1);
% Jn = [q2R(X_km1_k(4:7)),zeros(3,4);...
%       zeros(4,3) eye(4) ];

% Q = G*Jn*Pn*Jn'*G';
Q = G*Pn*G';






% Q = func_Q( X_k(1:13,:), zeros(6,1), Pn, delta_t, type);

size_P_k = size(P_k,1);

P_km1_k = [ F*P_k(1:13,1:13)*F' + Q         F*P_k(1:13,14:size_P_k);
    P_k(14:size_P_k,1:13)*F'        P_k(14:size_P_k,14:size_P_k)];



% normalize the quaternion
% Jnorm = normJac( X_km1_k( 4:7 ) );
% size_P_km1_k = size(P_km1_k,1);
% P_km1_k = [   P_km1_k(1:3,1:3)              P_km1_k(1:3,4:7)*Jnorm'               P_km1_k(1:3,8:size_P_km1_k);
%     Jnorm*P_km1_k(4:7,1:3)        Jnorm*P_km1_k(4:7,4:7)*Jnorm'         Jnorm*P_km1_k(4:7,8:size_P_km1_k);
%     P_km1_k(8:size_P_km1_k,1:3)     P_km1_k(8:size_P_km1_k,4:7)*Jnorm'      P_km1_k(8:size_P_km1_k,8:size_P_km1_k)];
% 
% X_km1_k( 4:7 ) = X_km1_k( 4:7 ) / norm( X_km1_k( 4:7 ) );



