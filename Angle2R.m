function R = Angle2R(axis, theta)
% axis为旋转轴，取值为'x'、'y'或'z'
% theta为旋转角度，单位为弧度
% 返回一个3*3的旋转矩阵R

% example
% R = Angle2R('x', pi/4);

switch axis
    case 'x'
        R = [1 0 0; 
             0 cos(theta) -sin(theta); 
             0 sin(theta) cos(theta)];
    case 'y'
        R = [cos(theta) 0 sin(theta); 
             0 1 0; 
             -sin(theta) 0 cos(theta)];
    case 'z'
        R = [cos(theta) -sin(theta) 0; 
             sin(theta) cos(theta)  0; 
             0 0 1];
    otherwise
        error('invalid input');
end
end


