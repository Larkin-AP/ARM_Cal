%ARM_FK

%{
改进DH方法建系
这里的DH表是正确的，经过验证了
这里的表是根据实际的机器人建立的，也就是7是7a，没有考虑8，考虑8的话姿态是一样的，就是加上偏置
这里的T_07应该和T_08是一样的
关节        a_i         alpha_i     d_i         theta_i
1           0           0           d_bs        theta_1+pi                  
2           0           pi/2        0           theta_2-pi/2
3           0           pi/2        d_se        theta_3+pi/2
4           a_se       -pi/2        0           theta_4
5           0           pi/2        d_ew        theta_5
6           0           pi/2        0           theta_6+pi/2
7           a_wf        pi/2        0           theta_7
8           a_ee        0           0           0
d_bs=158.3
d_se=359.6
d_ew=256.9
d_wf=5
a_wf=12
a_se=9

r11=c7(s6(s4(s1s3+c1c2c3)+c1c4s2)+c6(c5(c4(s1s3+c1c2c3)−c1s2s4)+s5(c3s1−c1c2s3)))
+s7(s5(c4(s1s3+c1c2c3)−c1s2s4)−c5(c3s1−c1c2s3))

r12=c7(s5(c4(s1s3+c1c2c3)−c1s2s4)−c5(c3s1−c1c2s3))
−s7(s6(s4(s1s3+c1c2c3)+c1c4s2)+c6(c5(c4(s1s3+c1c2c3)−c1s2s4)+s5(c3s1−c1c2s3)))

r13=s6(c5(c4(s1s3+c1c2c3)−c1s2s4)+s5(c3s1−c1c2s3))
−c6(s4(s1s3+c1c2c3)+c1c4s2)


%}

clear
clc

% d_bs=158.3;
% d_se=359.6;
% d_ew=256.9;
% a_wf=12;
% d_wf=0;
% a_se=9;
% a_ee = 39.5;
% 
% theta1 = 0;
% theta2 = 0;
% theta3 = 0;
% theta4 = 0;
% theta5 = 0;
% theta6 = 0;
% theta7 = 0;


syms d_bs d_se d_ew d_wf a_wf a_se a_ee theta1 theta2 theta3 theta4 theta5 theta6 theta7 
A1=modified_DH_transform(theta1+pi,d_bs,0,0);
A2=modified_DH_transform(theta2-pi/2,0,0,pi/2);
A3=modified_DH_transform(theta3+pi/2,d_se,0,pi/2);
A4=modified_DH_transform(theta4,0,a_se,-pi/2);
A5=modified_DH_transform(theta5,d_ew,0,pi/2);
A6=modified_DH_transform(theta6+pi/2,0,0,pi/2);
A7=modified_DH_transform(theta7,0,a_wf,pi/2);
A7v=modified_DH_transform(theta7,0,0,pi/2);
A8=modified_DH_transform(0,0,a_ee,0);

T_07=A1*A2*A3*A4*A5*A6*A7
T_04=A1*A2*A3*A4
T_01 = A1;
T_06=A1*A2*A3*A4*A5*A6
T_08=A1*A2*A3*A4*A5*A6*A7*A8
T_47 = simplify(A5*A6*A7v)

% pw = T_04*[0,0,d_ew,1]';
% pw = simplify(pw(1:3))
