function d2E_dqdT = d2E_dqdT_4cov(in1,in2,in3,in4)
%D2E_DQDT_4COV
%    D2E_DQDT = D2E_DQDT_4COV(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 5.8.
%    27-Jun-2012 13:14:16

f_n1 = in2(1,:);
f_n2 = in2(2,:);
f_n3 = in2(3,:);
q1 = in3(1,:);
q2 = in3(2,:);
q3 = in3(3,:);
q4 = in3(4,:);
t2 = f_n3.*q2.*2.0;
t3 = f_n2.*q2.*2.0;
t4 = f_n3.*q1.*2.0;
t5 = f_n1.*q2.*2.0;
t6 = f_n2.*q3.*2.0;
t7 = f_n3.*q4.*2.0;
t8 = t5+t6+t7;
t9 = f_n1.*q1.*2.0;
t10 = f_n3.*q3.*2.0;
t16 = f_n2.*q4.*2.0;
t11 = t9+t10-t16;
t12 = f_n1.*q3.*2.0;
t13 = f_n2.*q1.*2.0;
t14 = f_n1.*q4.*2.0;
t15 = -t2+t13+t14;
d2E_dqdT = reshape([t11,t15,t3+t4-t12,t8,-t3-t4+t12,t15,t3+t4-f_n1.*q3.*2.0,t8,-t9-t10+t16,t2-f_n2.*q1.*2.0-f_n1.*q4.*2.0,t11,t8],[3, 4]);
