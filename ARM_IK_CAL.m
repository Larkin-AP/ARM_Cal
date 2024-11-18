% 整理过后的ARM_IK，这个需要输出正确的关节角度，符号对应整理的文档
function theta = ARM_IK_CAL(T,phi)
addpath('E:\Research\Research\0000_matlab_math_lib\code');

% theta 是所以电机的角度，theta = [theta1,theta2,theta3,delta_1,theta5,delta2,delta3];
% phi 是臂型角，认为要给定臂型角
% T是末端位姿
% 测试用的零位T
%{
T_08 =[1.0000         0         0  668.000;
         0         0    1.0000    9.0000;
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

T_08 = T;
% 坐标系7的姿态和T_08一致，位置相差一个偏置,没带v认为是实际机械臂
R_08 = T(1:3,1:3);
P_8_wa = [-a_ee,0,0,1]'; % 实际腕关节在8坐标系下的表示
P_0_wa = T_08 * P_8_wa;
T_07 = [R_08,P_0_wa(1:3);
    0,0,0,1];

% 这个地方为了套用SRS，所以这里不用T_07
% 这里把theta7a挪到theta7v，theta7v和theta6的原点重合，这里是虚拟的腕关节
P_8_wv = [-a_ee-a_wf,0,0,1]';
P_0_wv = T_08 * P_8_wv;
T_07_v = [R_08,P_0_wv(1:3);
    0,0,0,1];

% 这里有虚拟腕关节，就可以采用SRS的方式
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

% 虚拟的z4，这个地方的虚拟是指不考虑臂型角
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


%表示x4
R_41 = eye(3)+sin(-gamma)*vecToLieAlgebra(z4_hat)+(1-cos(-gamma))*vecToLieAlgebra(z4_hat)^2;
R_42 = eye(3)+sin(pi/2)*vecToLieAlgebra(z4_hat)+(1-cos(pi/2))*vecToLieAlgebra(z4_hat)^2;
x4 = R_42*R_41*vec_d;
x4_hat = x4/norm(x4);
y4_hat = cross(z4_hat,x4_hat);

% 在不考虑臂型角的情况下的R_04
R_04_v = [x4_hat,y4_hat,z4_hat];
R_03_v = [x4_hat,z4_hat,-y4_hat];
R_0phi = eye(3)+sin(phi)*vecToLieAlgebra(vec_d_hat)+(1-cos(phi))*vecToLieAlgebra(vec_d_hat)^2;
R_03 = R_0phi*R_03_v;

% 这里根据正运动学可以反解出theta1，2，3
theta2 = -asin(R_03(3,3));
theta3 = atan(R_03(3,1)/R_03(3,2));
theta1 = atan(R_03(2,3)/R_03(1,3));

% theta4 =theta4_up;
theta4 =theta4_down;









% 至此，求出了theta1，2，3，4
A1=modified_DH_transform(theta1+pi,d_bs,0,0);
A2=modified_DH_transform(theta2-pi/2,0,0,pi/2);
A3=modified_DH_transform(theta3+pi/2,d_se,0,pi/2);
A4=modified_DH_transform(theta4,0,a_se,-pi/2);

T_04 = A1*A2*A3*A4;

%这种方法下，前6个关节都是没问题的，7是虚拟的转轴
T_47v = (T_04)\T_07_v;
R_47v = T_47v(1:3,1:3);
theta6 = asin(-R_47v(2,3));
theta7v = atan(-R_47v(2,2)/R_47v(2,1));
theta5 =atan(R_47v(3,3)/R_47v(1,3));

A5=modified_DH_transform(theta5,d_ew,0,pi/2);
A6=modified_DH_transform(theta6+pi/2,0,0,pi/2);

T_06 = T_04*A5*A6;
T_67 = (T_06)\T_07;

% 根据A7=T_67,可以求出
theta7a = asin(T_67(3,1));
theta7 = theta7a;

% 上面的theta7a可能会超角度限制，需要进一步验算

% 至此，该机构等效的串联机械臂的反解已经解出
% 进一步需要解出直线电机的进给量

% delta1，关节4处的直线电机
l_bx = 214.6;
l_by = 15;
l_ofsx = 23.4;
l_ofsy = 15;

len_lb = sqrt(l_bx^2+l_by^2);
len_lofs = sqrt(l_ofsx^2+l_ofsy^2);
beta3 = 147.34/180*pi;
beta40 = 28.66/180*pi;
beta4 = theta4+beta40;
lm1 = sqrt(len_lb^2+len_lofs^2-cos(beta4)*2*len_lb*len_lofs);
lm10 = 191.2;
delta1 = lm1-lm10;

A7=modified_DH_transform(theta7,0,a_wf,pi/2);

% 求解腕关节的电机
d_cx =12;
d_cy =28;
d_cz = 19;
P_7_C = [-d_cx,d_cy,d_cz,1]'; % C点在7坐标系下的坐标
T_57 = A6*A7;
P_5_C = T_57*P_7_C;  % C点在5坐标系下的坐标

d_ax = 0;
d_ay = 31;
d_az = 150.3;
P_5_A = [d_ax,-d_ay,-d_az,1]';
%表示向量l_AC，在5坐标系下
vec_AC = P_5_C-P_5_A;
vec_AC = vec_AC(1:3);

l_m20=151.53;
l_m30=151.53;

delta2 = norm(vec_AC)-l_m20;

P_7_D = [-d_cx,-d_cy,d_cz,1]'; % D点在7坐标系下的坐标
P_5_D = T_57*P_7_D;
P_5_E = [d_ax,d_ay,-d_az,1]';
vec_ED = P_5_D-P_5_E;
vec_ED = vec_ED(1:3);
delta3 = norm(vec_ED)-l_m30;

theta = [theta1,theta2,theta3,delta1,theta5,delta2,delta3];


end