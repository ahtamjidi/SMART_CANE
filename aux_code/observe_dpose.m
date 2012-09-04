function [z,h,H,R] = observe_dpose(X_k_k)
global myCONFIG step_global
persistent time_stamp
if ~isempty(time_stamp) || size(time_stamp,2)==0
    load([myCONFIG.PATH.SOURCE_FOLDER,'/TimeStamp/TimeStamp.mat'],'time_stamp');
end

dt = (time_stamp(1,step_global)-time_stamp(1,step_global-1))*0.001;
% r_ =X_k_k(1:3,1);
q_=X_k_k(4:7,1);
v_ =X_k_k(8:10,1);
w_ =X_k_k(11:13,1);
[dX_z,dq_z,R_z,State_RANSAC_z]=Calculate_V_Omega_RANSAC_dr_ye_key(step_global-1,step_global);
dX_ = q2R(q_)'*v_*dt;
[dq_ , dq_dv ]= v2q_(w_*dt);
h = [dX_;dq_];
z = [dX_z;dq_z];
R = bootstrap_cov_calc(step_global-1,step_global);
z33 = zeros(3,3);  z43 = zeros(4,3); z44 = zeros(4,4);
dX_obs_dq_robot = dX_obs_dq(q2qc(q_),v_,dt);
H = [z33  dX_obs_dq_robot  q2r(q_)'*dt  z33;...
     z43  z44              z43         dq_dv*dt];



