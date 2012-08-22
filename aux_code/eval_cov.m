
clc
clear all
close all
config_file
% step_global = 102;
global myCONFIG
sqrt_pn = zeros(6,197);
% myCONFIG.STEP.END =100
% myCONFIG.STEP.END = 200;
dx= zeros(10,myCONFIG.STEP.END);
axis_angle=zeros(1,myCONFIG.STEP.END);
linear_v=zeros(1,myCONFIG.STEP.END);
angular_v=zeros(1,myCONFIG.STEP.END);
load([myCONFIG.PATH.DATA_FOLDER,'/TimeStamp/TimeStamp.mat'],'time_stamp')
for step_global =4:myCONFIG.STEP.END-1
    pn = calc_cov_RANSAC_dr_ye(step_global-2,step_global-1);
    [dX_gt,dq_calc,R,State_RANSAC]=Calculate_V_Omega_RANSAC_dr_ye(step_global-2,step_global-1);
    [eul_,dQe]=q2e((dq_calc));
    [a,u] = q2au(dq_calc);
    axis_angle(step_global) = a;
    cov_eul = dQe*(pn(4:7,4:7))*dQe';
    sqrt_pn(:,step_global) = sqrt([diag(pn(1:3,1:3));diag(cov_eul)]);
    dx(:,step_global) = [dX_gt;dq_calc;q2e(dq_calc)];
    time_difference = (time_stamp(1,step_global-1)-time_stamp(1,step_global-2))/1000;
    linear_v(step_global)= norm(dX_gt)/time_difference;
    angular_v(step_global) = (180/pi)*a/time_difference;
    
    step_global
end
for i=5:size(dx,2)
    dx_avg(:,i) = [mean(dx(1:3,i-3:i)')';q_getAverage(dx(4:7,i-3:i)')'];
    time_difference = (time_stamp(1,i-1)-time_stamp(1,i-2))/1000;
    linear_v_avg(i)= norm(dx_avg(1:3,i))/time_difference;
%     dq_avg(:,i) = q_getAverage(dx_avg(4:7,i))
    [a,u] = q2au(dx_avg(4:7,i));
    axis_angle(step_global) = a;
    angular_v_avg(i) = (180/pi)*a/time_difference;
end
% figure
% subplot(311)
% plot(dx(1,:),'b');hold on;plot(dx(1,:)+2.*sqrt_pn(1,:),'r');plot(dx(1,:)-2.*sqrt_pn(1,:),'r');
% legend('X','Y','Z'); title('Translation');xlabel('step');ylabel('translation (meter)');
figure
plot(2.*sqrt_pn(1,:),'r');hold on;plot(-2.*sqrt_pn(1,:),'r');grid on
plot(2.*sqrt_pn(2,:),'g');hold on;plot(-2.*sqrt_pn(2,:),'g');
plot(2.*sqrt_pn(3,:),'b');hold on;plot(-2.*sqrt_pn(3,:),'b');
title('translational 2*std');xlabel('step');ylabel('std ');
figure
plot(dx(1,:),'r');hold on;plot(dx(2,:),'g');plot(dx(3,:),'b');grid on
legend('X','Y','Z'); title('Translation');xlabel('step');ylabel('translation (meter)');
figure
subplot(311);plot((180/pi)*dx(8,:),'r');hold on;xlabel('step');ylabel('degree');title('roll');grid on
subplot(312);plot((180/pi)*dx(9,:),'g');hold on;xlabel('step');ylabel('degree');title('pitch');grid on
subplot(313);plot((180/pi)*dx(10,:),'b');hold on;xlabel('step');ylabel('degree');title('yaw');grid on
figure
subplot(311);plot((180/pi)*sqrt_pn(4,:),'r');hold on;xlabel('step');ylabel('degree');title('roll');grid on
subplot(312);plot((180/pi)*sqrt_pn(5,:),'g');hold on;xlabel('step');ylabel('degree');title('pitch');grid on
subplot(313);plot((180/pi)*sqrt_pn(6,:),'b');hold on;xlabel('step');ylabel('degree');title('yaw');grid on

figure
subplot(211)
plot(linear_v);hold on;xlabel('step');ylabel('m/s');grid on
title('linear velocity')
subplot(212)
plot(angular_v); hold on;xlabel('step');ylabel('deg/s');grid on
title('angular velocity')
figure
plot(time_stamp(1,2:end)-time_stamp(1,1:end-1))

figure;subplot(211);plot(linear_v_avg,'b');hold on;plot(linear_v,'r')
subplot(212);plot(angular_v_avg,'b');hold on;plot(angular_v,'r')


