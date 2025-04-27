%% 日期4.25重新开始尝试完成正解，采用坐标变换完整消除的方式试试看


%% 以手腕关节的图为准
% 坐标系5的朝向与手腕基座的朝向一致。
% clc
clear

a_wf = 12;   % 赋值示例
ax = 0;     ay = -31;    az = -150.3;
cx = -12;     cy = 28;     cz = 19;
dx = -12;     dy = -28;     dz = 19;

% 根据条件 ax = bx, ay = -by, az = bz
bx = ax;    by = -ay;   bz = az;

% 角度变量（单位：弧度）
theta6 = pi/4;  % 赋值示例
theta7 = pi/6;


% syms theta6 theta7 a_wf ax ay az bx by bz cx cy cz dx dy dz real

% 首先计算 T_57
A6=modified_DH_transform(theta6+pi/2,0,0,pi/2);
A7=modified_DH_transform(theta7,0,a_wf,pi/2);
% A6 = simplify(A6);
% A7 = simplify(A7);
T_57  = A6*A7;

% A,B 点在5系下的坐标，与基坐标系固定
P_5A = [ax, ay, az,1]';
P_5B = [bx, by, bz,1]';

% C D 在7系下的坐标是固定的，且已知
P_7C = [cx, cy, cz,1]';
P_7D = [dx, dy, dz,1]';

% 把C D 转到5系下
P_5C = T_57*P_7C;
P_5D = T_57*P_7D;

l1 = norm(P_5C(1:3) - P_5A(1:3));
l2 = norm(P_5D(1:3) - P_5B(1:3));

% 约束方程
l1_sq = (norm(P_5C(1:3) - P_5A(1:3)))^2
l2_sq = (norm(P_5D(1:3) - P_5B(1:3)))^2


% disp(l1_sq);
% disp(l2_sq);
% l1_sq_ex = expand(l1_sq);
% l2_sq_ex = expand(l2_sq);
% disp(l1_sq_ex);
% disp(l2_sq_ex);
% 
% l1_sq_ex_spl = simplify(l1_sq_ex);
% l2_sq_ex_spl = simplify(l2_sq_ex);
% disp(l1_sq_ex_spl);
% disp(l2_sq_ex_spl);

% l1_sq = simplify(l1_sq);
% l2_sq = simplify(l2_sq);

% l1_plus_l2_sq = l1_sq + l2_sq;
% l1_minu_l2_sq = l1_sq - l2_sq;
% 
% l1_plus_l2_sq = simplify(l1_plus_l2_sq);
% l1_minu_l2_sq = simplify(l1_minu_l2_sq);

% disp(l1_sq);
% disp(l2_sq);
% disp(l1_plus_l2_sq);
% disp(l1_minu_l2_sq);






