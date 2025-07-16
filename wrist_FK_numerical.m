%% 机械臂双关节角求解系统
% 文件: solve_robot_equations.m
% 作者: MATLAB助手
% 描述: 求解包含theta6和theta7的非线性方程组

clc; clear; close all;


%% 参数初始化 (根据实际系统参数修改)
% 机械臂几何参数
% 基本参数
a_wf = 12;   % 赋值示例
ax = 0;     ay = -31;    az = -150.3;

% c 和 d都是在7系下固定的点的坐标，所以这里不会变
cx = -12;     cy = 28;     cz = 19;
dx = -12;     dy = -28;     dz = 19;

% 根据条件 ax = bx, ay = -by, az = bz
bx = ax;    by = -ay;   bz = az;


G1 = 2*ax^2 + 2*ay^2 + 2*az^2 + cx^2 + cy^2 + cz^2 + dx^2 + dy^2 + dz^2 + 2*a_wf^2;  % 绝对常数项
G2 = -2*az*cz - 2*az*dz + 4*ax*a_wf;       % sin(theta6)系数
G3 = -2*ax*cz - 2*ax*dz - 4*az*a_wf;       % cos(theta6)系数
G5 = +2*ay*cy - 2*ay*dy + 2*cx*a_wf + 2*dx*a_wf;  % cos(theta7)系数
G6 = -2*az*cx - 2*az*dx;                   % cos(theta6)cos(theta7)系数
H4 = +2*ay*cx + 2*ay*dx - 2*cy*a_wf + 2*dy*a_wf;  % sin(theta7)系数
H8 = +2*az*cy - 2*az*dy;      % cos(theta6)sin(theta7)系数

% 目标长度平方 (根据测量值修改)
l1_sq = 2.467176180591793e+04;
l2_sq = 3.055133816180886e+04;




%% 方程系统定义
% 方程1: l1^2 + l2^2 的表达式差
F1 = @(theta6, theta7) G1 + G2*sin(theta6) + G3*cos(theta6) + ...
     G5*cos(theta7) + G6*cos(theta6).*cos(theta7) - (l1_sq + l2_sq);

% 方程2: l1^2 - l2^2 的表达式差
F2 = @(theta6, theta7) H4*sin(theta7) + H8*cos(theta6).*sin(theta7) - (l1_sq - l2_sq);

% 组合方程系统
equations = @(x) [ 
    F1(x(1), x(2)); 
    F2(x(1), x(2)) 
];

%% 数值求解配置
initial_guess = [0, 0];  % 初始猜测值 (根据机械臂姿态调整)
options = optimoptions('fsolve', ...
    'Algorithm', 'trust-region-dogleg', ...
    'Display', 'final-detailed', ...
    'FunctionTolerance', 1e-8, ...
    'StepTolerance', 1e-8);

%% 执行求解
[theta_sol, fval, exitflag] = fsolve(equations, initial_guess, options);

%% 结果验证
if exitflag > 0
    fprintf('===== 求解成功 =====\n');
    fprintf('theta6 = %.4f rad (%.2f°)\n', theta_sol(1), rad2deg(theta_sol(1)));
    fprintf('theta7 = %.4f rad (%.2f°)\n', theta_sol(2), rad2deg(theta_sol(2)));
    fprintf('残差范数: %.2e\n', norm(fval));
else
    error('求解失败，请尝试调整初始猜测或检查参数');
end

%% 可视化误差曲面
figure('Name', '方程误差分析');
[Theta6, Theta7] = meshgrid(linspace(-pi, pi, 50), linspace(-pi, pi, 50));

% 计算误差场
Error1 = arrayfun(@(x,y) F1(x,y), Theta6, Theta7);
Error2 = arrayfun(@(x,y) F2(x,y), Theta6, Theta7);
TotalError = sqrt(Error1.^2 + Error2.^2);

% 绘制3D误差曲面
surf(Theta6, Theta7, TotalError);
xlabel('\theta_6 (rad)'); ylabel('\theta_7 (rad)'); zlabel('总误差');
title('双关节角解空间误差分布');
hold on;
plot3(theta_sol(1), theta_sol(2), 0, 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
colorbar;

%% 多解性分析 (可选)
% 使用不同初始猜测寻找其他解
alternative_guesses = [
    3*pi/4, 3*pi/4;
    -pi/2, -pi/3;
    pi/3, -pi/4
];

solutions = [];
for k = 1:size(alternative_guesses, 1)
    [sol, ~, flag] = fsolve(equations, alternative_guesses(k,:), options);
    if flag > 0
        solutions = [solutions; sol];
    end
end

% 显示所有可行解
if ~isempty(solutions)
    disp('===== 发现多组解 =====');
    disp('theta6(rad)   theta7(rad)');
    disp(solutions);
end