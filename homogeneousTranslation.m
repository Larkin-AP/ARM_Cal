function T = homogeneousTranslation(axis, distance)
    % 这个函数接受一个平移轴（'x'、'y'或'z'）和一个平移距离（可以是符号或数字），
    % 并返回相应的4x4齐次变换矩阵。

    % 使用sym函数确保矩阵是符号类型，以便处理符号和数字
%     T = sym(eye(4)); % 初始化为符号单位矩阵
    T = eye(4); % 初始化为数字单位矩阵

    % 根据轴向设置平移分量
    switch axis
        case 'x'
            T(1, 4) = distance; % 沿x轴平移
        case 'y'
            T(2, 4) = distance; % 沿y轴平移
        case 'z'
            T(3, 4) = distance; % 沿z轴平移
        otherwise
            error('Invalid translation axis. Choose ''x'', ''y'', or ''z''.');
    end
end
