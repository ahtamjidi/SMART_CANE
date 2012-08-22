%% testing quaternion and angular velocity related issues
clear all
close all
clc
w=[1; 2; 3]*pi/180; %% angular velocity
q0 = [1;0;0;0];
W = w2omega(w);
dt= 1;
q1 = q0 + 0.5*dt*W*q0;
q1 = q1/norm(q1);
q2 = qProd(q0,v2q(w*dt));
q2 = q2/norm(q2);
q2e(q1)*180/pi;
q2e(q2)*180/pi;
[a1,u1] = q2au(q1);


