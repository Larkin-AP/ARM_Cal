% ARM_IK_CAL2
% 这个函数是第七关节带偏置的运动学反解，参考2012年的会议论文的方法
% ARM_IK_CAL1 应该还是有问题，因为那个地方也带偏置，但是是把theta7当作0去处理，所以求解起来是错误的

function theta = ARM_IK_CAL2(T,theta7)
addpath('E:\Research\Research\0000_matlab_math_lib\code');
% theta 是所以电机的角度，theta = [theta1,theta2,theta3,delta_1,theta5,delta2,delta3];
% phi 是臂型角，这里要求解一次臂型角和theta7的关系，这里是用代数方法求解的
% T是末端位姿
% 测试用的零位T
%{
T_08 =[  0         0         1    9.000;
         -1         0        0    -668.0000;
         0   -1.0000         0  158.3000;
         0         0         0    1.0000];
%}

% 常量
d_bs=158.3;
d_se=359.6;
d_ew=256.9;
a_wf=12;
a_se=9;
a_ee=39.5;

% 第一步，根据末端位姿和theta7，求出虚拟球腕坐标的位姿
T_08 = T;
A7=modified_DH_transform(theta7,0,a_wf,pi/2);

R_08 = T(1:3,1:3);
P_8_wa = [-a_ee,0,0,1]'; % 实际腕关节在8坐标系下的表示
P_0_wa = T_08*P_8_wa;
T_07a = [R_08,P_0_wa(1:3);
    0,0,0,1];
R_07 = R_08;

% 虚拟腕关节和56 原点重合
T_06=T_07a/(A7);
P_0_wv = T_06(:,4);

% 现在有了虚拟腕关节，可以使用SRS的公式
len_vec_SE = sqrt(d_se^2+a_se^2);
len_vec_EW = d_ew;
angle_ESEv = atan2(a_se,d_se);
beta1 = angle_ESEv;

P_0_S = [0,0,d_bs,1]'; % 肩关节在Base下的坐标
vec_d = P_0_wv -P_0_S; % 肩腕关节向量（虚拟腕关节）
vec_d = vec_d(1:3);
len_vec_d = norm(vec_d);

% 肘关节向上的情况，这种情况下零位可能不太行，因此零位可能要测试在肘关节向下的情况
% 这里腕关节分为向上和向下，这里先写向上的情况
angle_SEW = acos((len_vec_SE^2+len_vec_EW^2-len_vec_d^2)/(2*len_vec_SE*len_vec_EW));
beta2 = angle_SEW;
theta4_up = beta1+pi-beta2;

% 肩腕关节向量的单位向量
vec_d_hat = vec_d/len_vec_d;
% 参考向量
vec_v_hat = [0,1,0]';

% 虚拟的z4，和参考平面重合时候的z4
z4 = cross(vec_d_hat,vec_v_hat);
z4_hat = z4/norm(z4);
angle_ESW = acos((len_vec_SE^2+len_vec_d^2-len_vec_EW^2)/(2*len_vec_SE*len_vec_d));
angle_EvSW = angle_ESW + beta1;
gamma_up = angle_EvSW;

% 肘关节向下的情况
beta20 = atan(d_se/a_se)+pi/2;
theta4_down = beta2-beta20;
gamma_down = beta1-angle_ESW;

gamma=gamma_down;
% theta4 =theta4_up;
theta4 =theta4_down;

% 这里是新增的内容，因为要求参考平面下的theta10，theta20，theta30，
% 所以这里要以臂型角为0时计算一下theta10，theta20，theta30

R_41 = eye(3)+sin(-gamma)*vecToLieAlgebra(z4_hat)+(1-cos(-gamma))*vecToLieAlgebra(z4_hat)^2;
R_42 = eye(3)+sin(pi/2)*vecToLieAlgebra(z4_hat)+(1-cos(pi/2))*vecToLieAlgebra(z4_hat)^2;
x4 = R_42*R_41*vec_d;
x4_hat = x4/norm(x4);
y4_hat = cross(z4_hat,x4_hat);

% 在不考虑臂型角的情况下的R_04
R_04_v = [x4_hat,y4_hat,z4_hat];
R_03_v = [x4_hat,z4_hat,-y4_hat]; % 这个是在参考平面下的时候的R_03
phi0=0;
R_0phi = eye(3)+sin(phi0)*vecToLieAlgebra(vec_d_hat)+(1-cos(phi0))*vecToLieAlgebra(vec_d_hat)^2;
R_03 = R_0phi*R_03_v;

% 这里根据正运动学可以反解出theta1，2，3，按理说这里theta3应该是指定的，
% 但是似乎感觉也可以完全反解出来，两者有什么区别呢
% 这里theta10，theta20，theta30都是当手臂位于参考平面内的时候的关节角，这个地方不知道有没有唯一性
% ？？？------这里theta3指定和不指定有什么区别------？？？
theta20 = -asin(R_03_v(3,3));
theta30 = atan(R_03_v(3,1)/R_03_v(3,2));
theta10 = atan(-R_03_v(1,3)/R_03_v(2,3));

% 当知道默认的theta角的时候，就可以表示出R47
As = vecToLieAlgebra(vec_d_hat)*R_03_v;
Bs = -vecToLieAlgebra(vec_d_hat)*vecToLieAlgebra(vec_d_hat)*R_03_v;
Cs = vec_d_hat*vec_d_hat'*R_03_v;

R_032 = As*sin(phi0)+Bs*cos(phi0)+Cs

A4=modified_DH_transform(theta4,0,a_wf,pi/2);
R4 = A4(1:3,1:3);

Aw = R4'*As'*R_07;
Bw = R4'*Bs'*R_07;
Cw = R4'*Cs'*R_07;

% tan(theta7)= -r22/r21
P = tan(theta7)*Aw(2,1)+Aw(2,2);
Q = tan(theta7)*Bw(2,1)+Bw(2,2);
R = -tan(theta7)*Cw(2,1)-Cw(2,2);

% Psin(phi)+Qcos(phi)=R
% sin(phi) = tan(phi)/sqrt(1+tan(phi)^2),cos(phi) = 1/sqrt(1+tan(phi)^2)
% 整理成二次方程(P^2-R^2)tan(phi)^2+2PQtan(phi)+(Q^2-R^2) = 0
% 求解二次方程后，得到结果如下
% tan_phi_1 = (-2*P*Q+sqrt((2*P*Q)^2-4*(P^2-R^2)*(Q^2-R^2)))/(2*(P^2-R^2));
% tan_phi_2 = (-2*P*Q-sqrt((2*P*Q)^2-4*(P^2-R^2)*(Q^2-R^2)))/(2*(P^2-R^2));

phi1 = atan2(-2*P*Q+sqrt((2*P*Q)^2-4*(P^2-R^2)*(Q^2-R^2)),2*(P^2-R^2));
phi2 = atan2(-2*P*Q-sqrt((2*P*Q)^2-4*(P^2-R^2)*(Q^2-R^2)),2*(P^2-R^2));

% 这里还不知道phi如何选择，可能要看phi的值，可能有的值是坏掉的
phi = phi1;

theta1 = -atan2(-(As(1,3)*sin(phi)+Bs(1,3)*cos(phi)+Cs(1,3)),As(2,3)*sin(phi)+Bs(2,3)*cos(phi)+Cs(2,3))+pi
theta2 = -asin(As(3,3)*sin(phi)+Bs(3,3)*(cos(phi)+Cs(3,3)))
theta3 = -atan2(As(3,1)*sin(phi)+Bs(3,1)*cos(phi)+Cs(3,1),As(3,2)*sin(phi)+Bs(3,2)*cos(phi)+Cs(3,2))
theta4
theta5 = atan2(Aw(3,3)*sin(phi)+Bw(3,3)*cos(phi)+Cw(3,3),Aw(1,3)*sin(phi)+Bw(1,3)*cos(phi)+Cw(1,3))
theta6 = -asin(Aw(2,3)*sin(phi)+Bw(2,3)*cos(phi)+Cw(2,3))
theta6
