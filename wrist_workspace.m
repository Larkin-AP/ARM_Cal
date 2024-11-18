% workspace analysis

% 蒙特卡洛法生成工作空间
num_points = 1000000; % 生成点的数量
x_range = [150, 220]; % x坐标范围
y_range = [-55, 55]; % y坐标范围
z_range = [-25, 25]; % z坐标范围

valid_coords = NaN(num_points, 3); % 预分配空间
valid_count = 0;

for i = 1:num_points
    % 生成随机坐标
    x = (x_range(2)-x_range(1)).*rand() + x_range(1);
    y = (y_range(2)-y_range(1)).*rand() + y_range(1);
    z = (z_range(2)-z_range(1)).*rand() + z_range(1);
    
    coords = [x, y, z];
    output = wrist_IK_cal(coords);
    
    if ~isempty(output)
        valid_count = valid_count + 1;
        valid_coords(valid_count, :) = coords;
    end
end

% 移除预分配矩阵中的NaN行
valid_coords = valid_coords(~isnan(valid_coords(:,1)), :);

% 根据z坐标生成颜色数据
z_values = valid_coords(:,2);
color_data = z_values;

%%
% 绘制工作空间
figure;
scatter3(valid_coords(:,1), valid_coords(:,3), valid_coords(:,2), 10, color_data, 'filled');
xlabel('X (mm)');
ylabel('Z (mm)');
zlabel('Y (mm)');
title('Workspace of the wrist');
colorbar; % 显示颜色条
grid on;
set(gca, 'XDir', 'reverse'); % 反向X轴
set(gca, 'YDir', 'reverse'); % 反向Z轴
set(gca, 'ZDir', 'reverse'); % 反向Y轴

% rotate3d on;
azimuth = 55; % 方位角
elevation = 30; % 仰角
view(azimuth, elevation);