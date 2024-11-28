% 这里测试推导手腕机构的运动学正解
% 根据前面的推算，目前已知的信息有T_05,F点是固定点
% AC，CF，CD的距离都是已知值

% 这里认为T_05是已知的
% 设C点坐标是(xc,yc,zc), D点坐标(xd,yd,zd)
% 本质上是六个参数，所以应该有六个方程才具有唯一解
% 目前来看CD还是耦合变量，更不好解了
% 还有一点是CD的中点应该存在几何约束，但是目前看起来CD的中点有一个值是定值，其他两个值是不确定的，不知道这样是否可以确定
% 从图中可以确定，CD的中点z坐标始终为0
% 这样算下来正解大概率是数值解了，不过到也不重要，因为正解无序实时计算


% length_CD = 100;
% length_AC = 100;
% length_ED = 100;
% length_CF = 100;
% length_DF = 100;

% syms xa ya za xe ye ze xf yf zf real
% 
% % 已知点A, E, F的三维坐标
% A = [xa, ya, za]; % 点A的坐标
% E = [xe, ye, ze]; % 点E的坐标
% F = [xf, yf, zf]; % 点F的坐标
% 
% syms d_ac d_ed d_cd real
% 
% 
% % 已知约束
% AC_dist = d_ac; % 点A到点C的距离
% ED_dist = d_ed; % 点E到点D的距离
% CD_dist = d_cd; % 点C到点D的距离

A = [0, 0, 0];
E = [5, 5, 0];
F = [10, 10, 10];
AC_dist = 5;
ED_dist = 5;
CD_dist = 6;


% 求解点C和点D
syms xc yc zc xd yd zd real

% 点C到点A的距离
eq1 = sqrt((xc - A(1))^2 + (yc - A(2))^2 + (zc - A(3))^2) == AC_dist;

% 点D到点E的距离
eq2 = sqrt((xd - E(1))^2 + (yd - E(2))^2 + (zd - E(3))^2) == ED_dist;

% 点C到点D的距离
eq3 = sqrt((xc - xd)^2 + (yc - yd)^2 + (zc - zd)^2) == CD_dist;

% 点C和点D中点的z坐标为0
eq4 = (zc + zd) / 2 == 0;

% 求解
[sol_xc, sol_yc, sol_zc, sol_xd, sol_yd, sol_zd] = solve([eq1, eq2, eq3, eq4], ...
    [xc, yc, zc, xd, yd, zd]);

% 提取解
C = double([sol_xc, sol_yc, sol_zc]); % 点C的解
D = double([sol_xd, sol_yd, sol_zd]); % 点D的解

% 显示结果
disp('点C的坐标:');
disp(C);

disp('点D的坐标:');
disp(D);


