function dE_dq = dE_dq_4cov(in1,in2,in3,in4)
%DE_DQ_4COV
%    DE_DQ = DE_DQ_4COV(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 5.8.
%    27-Jun-2012 13:14:08

T1 = in4(1,:);
T2 = in4(2,:);
T3 = in4(3,:);
f_n1 = in2(1,:);
f_n2 = in2(2,:);
f_n3 = in2(3,:);
f_p1 = in1(1,:);
f_p2 = in1(2,:);
f_p3 = in1(3,:);
q1 = in3(1,:);
q2 = in3(2,:);
q3 = in3(3,:);
q4 = in3(4,:);
t2 = q1.^2;
t3 = q2.^2;
t4 = q3.^2;
t5 = q4.^2;
t6 = q1.*q4.*2.0;
t7 = q1.*q3.*2.0;
t8 = q2.*q4.*2.0;
t9 = q1.*q2.*2.0;
t10 = t2+t3-t4-t5;
t11 = q2.*q3.*2.0;
t12 = t7+t8;
t13 = t2-t3+t4-t5;
t14 = t6+t11;
t15 = q3.*q4.*2.0;
t16 = t2-t3-t4+t5;
t17 = t7-t8;
t18 = t9+t15;
t19 = f_n2.*q2;
t20 = f_n3.*q1;
t52 = f_n1.*q3;
t21 = t19+t20-t52;
t22 = f_n2.*t13;
t23 = f_n1.*t14;
t24 = t9-t15;
t25 = f_n1.*t10;
t26 = t6-t11;
t27 = f_n3.*t12;
t28 = f_n2.*q1;
t29 = f_n1.*q4;
t77 = f_n3.*q2;
t30 = t28+t29-t77;
t31 = f_n3.*t16;
t32 = f_n2.*t18;
t62 = f_n1.*t17;
t33 = T3-f_p3+t31+t32-t62;
t34 = f_n2.*q2.*2.0;
t35 = f_n3.*q1.*2.0;
t63 = f_n1.*q3.*2.0;
t36 = t34+t35-t63;
t37 = T2.*(1.0./2.0);
t38 = f_n2.*t13.*(1.0./2.0);
t39 = f_n1.*t14.*(1.0./2.0);
t70 = f_p2.*(1.0./2.0);
t71 = f_n3.*t24.*(1.0./2.0);
t40 = t37+t38+t39-t70-t71;
t41 = T1.*(1.0./2.0);
t42 = f_n1.*t10.*(1.0./2.0);
t43 = f_n3.*t12.*(1.0./2.0);
t64 = f_p1.*(1.0./2.0);
t65 = f_n2.*t26.*(1.0./2.0);
t44 = t41+t42+t43-t64-t65;
t45 = f_n2.*q1.*2.0;
t46 = f_n1.*q4.*2.0;
t81 = f_n3.*q2.*2.0;
t47 = t45+t46-t81;
t48 = T3.*(1.0./2.0);
t49 = f_n3.*t16.*(1.0./2.0);
t50 = f_n2.*t18.*(1.0./2.0);
t75 = f_p3.*(1.0./2.0);
t76 = f_n1.*t17.*(1.0./2.0);
t51 = t48+t49+t50-t75-t76;
t78 = f_n2.*t26;
t53 = T1-f_p1+t25+t27-t78;
t54 = f_n1.*q2;
t55 = f_n2.*q3;
t56 = f_n3.*q4;
t57 = t54+t55+t56;
t80 = f_n3.*t24;
t58 = T2-f_p2+t22+t23-t80;
t59 = f_n1.*q1;
t60 = f_n3.*q3;
t79 = f_n2.*q4;
t61 = t59+t60-t79;
t66 = f_n1.*q2.*2.0;
t67 = f_n2.*q3.*2.0;
t68 = f_n3.*q4.*2.0;
t69 = t66+t67+t68;
t72 = f_n1.*q1.*2.0;
t73 = f_n3.*q3.*2.0;
t82 = f_n2.*q4.*2.0;
t74 = t72+t73-t82;
dE_dq = [t30.*(T2-f_p2+t22+t23-f_n3.*(t9-q3.*q4.*2.0))+t61.*(T1-f_p1+t25+t27-f_n2.*(t6-q2.*q3.*2.0))+t21.*t33+t36.*t51+t40.*t47+t44.*t74;t30.*t33-t36.*t40-t21.*t58+t47.*t51+t53.*t57+t44.*t69;t21.*t53+t36.*t44-t33.*t61+t40.*t69+t57.*t58-t51.*t74;-t30.*t53+t33.*t57-t44.*t47+t40.*t74+t58.*t61+t51.*t69];
