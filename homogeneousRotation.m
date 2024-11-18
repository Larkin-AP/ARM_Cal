function T = homogeneousRotation(axis, theta)
    % 这个函数接受一个旋转轴（'x'、'y'或'z'）和一个旋转角度theta（可以是符号或数字），
    % 并返回相应的4x4齐次旋转矩阵。

    % 初始化为4x4单位矩阵
    T = eye(4);

    % 根据轴向设置旋转矩阵
    switch axis
        case 'x'
            % 绕x轴旋转
            Rx = [1 0 0 0; 
                  0 cos(theta) -sin(theta) 0; 
                  0 sin(theta) cos(theta) 0;
                  0 0 0 1];
            T = Rx;
        case 'y'
            % 绕y轴旋转
            Ry = [cos(theta) 0 sin(theta) 0; 
                  0 1 0 0; 
                  -sin(theta) 0 cos(theta) 0;
                  0 0 0 1];
            T = Ry;
        case 'z'
            % 绕z轴旋转
            Rz = [cos(theta) -sin(theta) 0 0; 
                  sin(theta) cos(theta) 0 0; 
                  0 0 1 0;
                  0 0 0 1];
            T = Rz;
        otherwise
            error('Invalid rotation axis. Choose ''x'', ''y'', or ''z''.');
    end
end
