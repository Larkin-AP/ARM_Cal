% ARM_FK_CAL
function T_08 = ARM_FK_CAL(theta)
addpath('E:\Research\Research\0000_matlab_math_lib\code');
% 这里theta是实际传给电机的角度
theta1 =theta(1);
theta2 =theta(2);
theta3 =theta(3);
delta1 =theta(4);
theta5 =theta(5);
delta2 =theta(6);
delta3 =theta(7);

% 常量
d_bs=158.3;
d_se=359.6;
d_ew=256.9;
a_wf=12;
d_wf=0;
a_se=9;
a_ee = 39.5;

A1=modified_DH_transform(theta1+pi/2,d_bs,0,0);
A2=modified_DH_transform(theta2-pi/2,0,0,pi/2);
A3=modified_DH_transform(theta3+pi/2,d_se,0,pi/2);



% delta1，关节4处的直线电机
l_bx = 214.6;
l_by = 15;
l_ofsx = 23.4;
l_ofsy = 15;
lm10 = 191.2;
lm1 = lm10+delta1;

len_lb = sqrt(l_bx^2+l_by^2);
len_lofs = sqrt(l_ofsx^2+l_ofsy^2);
beta40 = 28.66/180*pi;
beta4 = acos((len_lb^2+len_lofs^2-lm1^2)/(2*len_lb*len_lofs));
theta4 = beta4-beta40;

A4=modified_DH_transform(theta4,0,a_se,-pi/2);
A5=modified_DH_transform(theta5,d_ew,0,pi/2);

T_05 = A1*A2*A3*A4*A5;

% 根据已知条件，要表示出A,E,F点在5坐标系下的坐标

d_ax = 0;
d_ay = 31;
d_az = 150.3;
P_5_E = [d_ax,d_ay,-d_az]'; % E在5坐标系下的表示
P_5_A = [d_ax,-d_ay,-d_az]'; % A在5坐标系下的表示
P_5_F = [0,0,0]';

l_m20=151.53;
l_m30=151.53;
lm2 = l_m20+delta2;
lm3 = l_m30+delta2;
len_CD = 56;

d_cx =19;
d_cy =-28;
d_cz = 0;

len_CF = sqrt(d_cx^2+d_cy^2+d_cz^2);
len_DF = len_CF;

len_CG = sqrt(12^2+19^2+28^2);
len_DG = len_CG;
% 新增1个变量
syms theta6 real
P_6_G = [12,0,0]';
P_5_G = [-12*sin(theta6),0,12*cos(theta6)]';
A6=modified_DH_transform(theta6+pi/2,0,0,pi/2);

% 求解点C和点D
syms xc yc zc xd yd zd real

% 点C到点A的距离
eq1 = sqrt((xc - P_5_A(1))^2 + (yc - P_5_A(2))^2 + (zc - P_5_A(3))^2) == lm2;

% 点D到点E的距离
eq2 = sqrt((xd - P_5_E(1))^2 + (yd - P_5_E(2))^2 + (zd - P_5_E(3))^2) == lm3;

% 点C到点D的距离
eq3 = sqrt((xc - xd)^2 + (yc - yd)^2 + (zc - zd)^2) == len_CD;

% 点C到F点距离固定
eq4 = sqrt((xc - P_5_F(1))^2 + (yc - P_5_F(2))^2 + (zc - P_5_F(3))^2) == len_CF;

% 点D到F点距离固定
eq5 = sqrt((xd - P_5_F(1))^2 + (yd - P_5_F(2))^2 + (zd - P_5_F(3))^2) == len_DF;

% 点C到G点距离固定
eq6 = sqrt((xc - P_5_G(1))^2 + (yc - P_5_G(2))^2 + (zc - P_5_G(3))^2) == len_CG;

% 点D到G点距离固定
eq7 = sqrt((xc - P_5_G(1))^2 + (yc - P_5_G(2))^2 + (zc - P_5_G(3))^2) == len_DG;


% 求解
[sol_xc, sol_yc, sol_zc, sol_xd, sol_yd, sol_zd,sol_theta6] = solve([eq1, eq2, eq3, eq4,eq5,eq6,eq7], ...
    [xc, yc, zc, xd, yd, zd,theta6]);

% 提取解
C = double([sol_xc, sol_yc, sol_zc]); % 点C的解
D = double([sol_xd, sol_yd, sol_zd]); % 点D的解
theta6= double(sol_theta6);
theta6

% 由此求出了CD坐标，这个坐标是在5坐标系下的坐标

end