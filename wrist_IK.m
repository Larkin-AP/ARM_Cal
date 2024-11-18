% 计算手腕的正逆运动学
% IK
% 已知手腕末端位姿，通过支链1位置方程表示关节角度
% 现在认为末端坐标系在Ry处，G点，末端为Pe点
% 先写末端位置
% 在支链1中有，从B到P点有，T_PB(把B移到P，P在B下表示)
% T_PB = T(y,-d1)T(x,d2)R(z,theta_11)T(x,d3)T(y,d4)R(y,theta_12)
% 知道末端位姿信息（在局部坐标系下{B}）(x,y,z,alpha, beta, gamma)，末端的坐标应该是在局部坐标系下表示的，这里暂时认为P点坐标是在B下
% 实际运行过程中，Rz位置是固定的，他和固定支链固定，Ry是会动的，Ry是绕着Rz转动的。对于P点来说，P点固定在Ry的中心，所以P点会绕着Rz转动，以及P点绕着Ry转动
% 也就是说对于P点来说，其坐标会变化的只有(x,y)(绕着Rz转动)，z不变。
% 对于姿态来说，alpha是绕x的转动，beta是绕y的转动，gama是绕z的转动，具有2个转动自由度，分别是绕着y转动（yaw），和绕着z转动，（pitch），其中pitch是和xy坐标耦合的
% 这样看来，P点的独立自由度，是pitch和yaw，pitch过程中，会影响xy坐标，yaw过程中xz坐标，所以说，xyz beta
% gama都会变动， 但是row不变

% 先计算正解，反解过程中，运动存在耦合关系
% 假定知道末端位姿(x,y,z,alpha, beta,gamma),
% alpha永远不变，其他五个变量存在耦合关系，末端坐标是在局部坐标系{B}下表示的
% 那么P点在{B}下的坐标可以表示为 P_B=T_BG*P_G,P_G是P点在G坐标系下的表示，是固定参数
% T_BG是G在B坐标系下的表示，把B转到G
% T_BG = T(y,-d1)T(x,d2)R(z,theta_11)T(x,d3)T(y,d4)R(y,theta_12)
% T_BG = T_11*R(z,theta11)*T_12*R(y,theta_12)


clc
clear all

% d1 = 39.5;
% d2 = 2.5;
% d3 = 157.7;
% d4 = 12;
% d5 = -2.5;
syms d1 d2 d3 d4 d5 real

% 单位 mm
Pp_G = [d1,d2,0,1]'; % P在G坐标系下的表示
T_11 = [1,0,0,d3;
    0,1,0,0;
    0,0,1,0;
    0,0,0,1];
T_12 = [1,0,0,d4;
    0,1,0,d5;
    0,0,1,0;
    0,0,0,1];

% x y z 是P点在B坐标系下的坐标，认为是已知量
syms theta_11 theta_12 x y z
T_BG = T_11*homogeneousRotation('z',theta_11)*T_12*homogeneousRotation('y',theta_12);
Pp_B = T_BG*Pp_G % Pp_B是P点在B坐标系下的坐标，这里可以求出theta_11和theta_12
x = Pp_B(1);
y = Pp_B(2);
z = Pp_B(3);

% P_B是知道了末端在局部坐标系下的位置。但是这个地方还无法直接求出反解
% 同理知道P_B,可以知道G点的位置，知道G点位置，可以表示出C和D点位置
% 知道P点位置，可以直接求出beta（绕y转动的角度）

Pg_G=[0,0,0,1]'; % G点在G坐标系下的表示
Pg_B=T_BG*Pg_G; % G点在B坐标系下的表示
Vcg_B=[12,-19.1,-28,0]'; % vector_CG在B下的表示
Pc_B = Pg_B-Vcg_B;
Pa_B = [5,0,32,1]';
V_l2 = Pc_B-Pa_B;
% 到这里表示出来了支链2的长度，通过求模可以求出最后的推杆变化量
% 类似地，可以求支链3的长度

Vdg_B = [12,-19.1,28,0]';% vector_DG在B下的表示
Pd_G = Pg_B-Vdg_B;
Pe_B = [5,0,-32,1]';
V_l3 = Pd_G-Pe_B;

% 在计算V_l2和V_l3的过程中，唯一的变量是T_BG，T_BG只关于theta_11和theta12,这个可以通过




