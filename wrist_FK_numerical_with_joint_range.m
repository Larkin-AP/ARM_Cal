%% 腕关节双关节角约束求解系统（修正版）
% 文件: constrained_robot_solver_fixed.m
% 更新内容: 1.修正可视化维度错误 2.应用新的关节角度约束

clc; clear; close all;

%% 参数初始化 (根据实际系统参数修改)
% 机械臂几何参数
a_wf = 12;        % 腕部特征参数
ax = 0;          ay = -31;      az = -150.3;
cx = -12;        cy = 28;       cz = 19;
dx = -12;        dy = -28;      dz = 19;

% 目标长度平方
l1_sq = 2.467176180591793e+04;  % 第一连杆平方长度
l2_sq = 3.055133816180886e+04;  % 第二连杆平方长度

% 新的关节角度约束
theta6_limits = [-pi/2, pi/2];   % theta6运动范围 (原[-pi, pi])
theta7_limits = [-pi/4, pi/4];   % theta7运动范围 (原[0, 2pi])

%% 系数矩阵计算
G1 = 2*ax^2 + 2*ay^2 + 2*az^2 + cx^2 + cy^2 + cz^2 + dx^2 + dy^2 + dz^2 + 2*a_wf^2;
G2 = -2*az*(cz + dz) + 4*ax*a_wf;
G3 = -2*ax*(cz + dz) - 4*az*a_wf;
G5 = 2*ay*(cy - dy) + 2*a_wf*(cx + dx);
G6 = -2*az*(cx + dx);
H4 = 2*ay*(cx + dx) - 2*a_wf*(cy - dy);
H8 = 2*az*(cy - dy);

%% 方程系统定义
equations = @(x) [
    G1 + G2*sin(x(1)) + G3*cos(x(1)) + G5*cos(x(2)) + G6*cos(x(1))*cos(x(2)) - (l1_sq + l2_sq);  % F1
    H4*sin(x(2)) + H8*cos(x(1))*sin(x(2)) - (l1_sq - l2_sq)                                      % F2
];

%% 求解器配置
options = optimoptions('lsqnonlin',...
    'Algorithm', 'levenberg-marquardt',...
    'Display', 'off',... %iter-detailed
    'MaxIterations', 1000,...
    'FunctionTolerance', 1e-10,...
    'StepTolerance', 1e-10);

% 初始猜测 (在约束范围内随机生成)
initial_guess = [
    theta6_limits(1) + diff(theta6_limits)*rand(), 
    theta7_limits(1) + diff(theta7_limits)*rand()
];

%% 显式约束求解
[theta_sol, resnorm, residual, exitflag] = lsqnonlin(...
    equations, initial_guess,...
    [theta6_limits(1), theta7_limits(1)],...  % Lower bounds
    [theta6_limits(2), theta7_limits(2)],...  % Upper bounds
    options);

%% 结果后处理与验证
% 角度修正
theta6_final = max(min(theta_sol(1), theta6_limits(2)), theta6_limits(1));  % 硬约束截断
theta7_final = max(min(theta_sol(2), theta7_limits(2)), theta7_limits(1));

% 约束验证
assert(theta6_final >= theta6_limits(1) && theta6_final <= theta6_limits(2),...
    'theta6越界! 解:%.4f rad 范围:[%.2f, %.2f]', theta6_final, theta6_limits(1), theta6_limits(2));
assert(theta7_final >= theta7_limits(1) && theta7_final <= theta7_limits(2),...
    'theta7越界! 解:%.4f rad 范围:[%.2f, %.2f]', theta7_final, theta7_limits(1), theta7_limits(2));

% 残差验证
tolerance = 1e-6;
assert(all(abs(residual) < tolerance),...
    '残差超限! F1:%.2e F2:%.2e', residual(1), residual(2));

%% 结果输出
fprintf('\n===== 求解成功 =====\n');
fprintf('迭代解: theta6 = %.4f rad (%.1f°)\n', theta_sol(1), rad2deg(theta_sol(1)));
fprintf('         theta7 = %.4f rad (%.1f°)\n', theta_sol(2), rad2deg(theta_sol(2)));
fprintf('修正解: theta6 = %.4f rad (%.1f°)\n', theta6_final, rad2deg(theta6_final));
fprintf('         theta7 = %.4f rad (%.1f°)\n', theta7_final, rad2deg(theta7_final));
fprintf('残差范数: %.2e\n', resnorm);

%% 修正后的可视化模块
figure('Position', [100 100 1400 600]);

% 解空间误差分布 (修正版)
subplot(1,2,1);
[Theta6_grid, Theta7_grid] = meshgrid(...
    linspace(theta6_limits(1), theta6_limits(2), 50),...
    linspace(theta7_limits(1), theta7_limits(2), 50));

% 安全计算每个网格点的残差
Z = zeros(size(Theta6_grid));
for i = 1:numel(Theta6_grid)
    Z(i) = log10(norm(equations([Theta6_grid(i), Theta7_grid(i)])));
end

surf(Theta6_grid, Theta7_grid, Z, 'EdgeColor','none');
xlabel('\theta_6 (rad)'); ylabel('\theta_7 (rad)');
title('对数尺度残差分布');
hold on;
plot3(theta6_final, theta7_final, min(Z(:)), 'rp', 'MarkerSize',15, 'MarkerFaceColor','r');
colorbar; 
colormap(jet(256));
view(-30, 30);  % 优化视角

% 约束边界标识 (适配新范围)
subplot(1,2,2);
rectangle('Position', [theta6_limits(1) theta7_limits(1) diff(theta6_limits) diff(theta7_limits)],...
         'EdgeColor', 'b', 'LineWidth', 2);
hold on; grid on;
scatter(theta6_final, theta7_final, 100, 'r', 'filled');
xlabel('\theta_6 (rad)'); ylabel('\theta_7 (rad)');
title('解在约束空间中的位置');
axis([theta6_limits(1)-0.1, theta6_limits(2)+0.1,...
      theta7_limits(1)-0.1, theta7_limits(2)+0.1]);

%% 多解性分析 (适配新约束范围)
fprintf('\n===== 多解性分析 =====\n');
search_points = 20;  % 每个维度采样点数
solutions = [];

% 生成约束范围内的采样网格
[theta6_init, theta7_init] = meshgrid(...
    linspace(theta6_limits(1), theta6_limits(2), search_points),...
    linspace(theta7_limits(1), theta7_limits(2), search_points));

parfor i = 1:numel(theta6_init)
    try
        [sol, ~, residual, exitflag] = lsqnonlin(equations, [theta6_init(i), theta7_init(i)],...
                                                theta6_limits, theta7_limits, options);
        
        if exitflag > 0 && all(abs(residual) < 1e-6)
            sol_clipped = [...
                max(min(sol(1), theta6_limits(2)), theta6_limits(1)),...
                max(min(sol(2), theta7_limits(2)), theta7_limits(1))...
            ];
            solutions = [solutions; sol_clipped];
        end
    catch
        continue;
    end
end

% 去除重复解 (四舍五入到小数点后4位)
solutions = round(solutions, 4);
[~, unique_idx] = uniquetol(solutions, 1e-4, 'ByRows',true);
solutions = solutions(unique_idx, :);

% 结果输出
if size(solutions,1) > 1
    fprintf('发现 %d 组有效解:\n', size(solutions,1));
    disp('   Theta6(rad)   Theta7(rad)   Theta6(°)   Theta7(°)');
    disp([solutions, rad2deg(solutions)]);
else
    fprintf('唯一解\n');
end