% function T = modified_DH_transform(theta, d, a, alpha)
%     % 计算改进的DH参数的位姿变换矩阵
%     % 输入：
%     %   theta - 沿z轴的关节旋转角度（弧度）
%     %   d     - 沿z轴的连杆长度
%     %   a     - 沿x轴的连杆偏移
%     %   alpha - 绕x轴的连杆扭转角（弧度）
% 
%     % 注意此处对cos进行了特化，所以的alpha 为pi的系数
%     
%     % 计算变换矩阵
%     T = [cospi(theta), -sinpi(theta), 0, a;
%          sinpi(theta) * cospi(alpha), cospi(theta) * cospi(alpha), -sinpi(alpha), -sinpi(alpha) * d;
%          sinpi(theta) * sinpi(alpha), cospi(theta) * sinpi(alpha), cospi(alpha), cospi(alpha) * d;
%          0, 0, 0, 1];
% end


function T = modified_DH_transform(theta, d, a, alpha)
    % MODIFIED_DH_TRANSFORM 计算改进的DH参数的位姿变换矩阵
    % 输入：
    %   theta - 沿z轴的关节旋转角度（可以为符号或数值，单位为弧度，可能含有pi）
    %   d     - 沿z轴的连杆长度（通常为数值）
    %   a     - 沿x轴的连杆偏移（通常为数值）
    %   alpha - 绕x轴的连杆扭转角（通常为数值，以pi的倍数表示）

    % 将alpha转换为以pi为单位的形式，以便使用cospi和sinpi
    alpha_pi = alpha / pi;
    
    % 判断theta是否为符号变量
    if isa(theta, 'sym')
        % 使用符号化的cos和sin进行计算
        T = [cos(theta), -sin(theta), 0, a;
             sin(theta) * cospi(alpha_pi), cos(theta) * cospi(alpha_pi), -sinpi(alpha_pi), -sinpi(alpha_pi) * d;
             sin(theta) * sinpi(alpha_pi), cos(theta) * sinpi(alpha_pi), cospi(alpha_pi), cospi(alpha_pi) * d;
             0, 0, 0, 1];
    else
        % 对于数值计算，使用cospi和sinpi
        theta_pi = theta / pi;
        T = [cospi(theta_pi), -sinpi(theta_pi), 0, a;
             sinpi(theta_pi) * cospi(alpha_pi), cospi(theta_pi) * cospi(alpha_pi), -sinpi(alpha_pi), -sinpi(alpha_pi) * d;
             sinpi(theta_pi) * sinpi(alpha_pi), cospi(theta_pi) * sinpi(alpha_pi), cospi(alpha_pi), cospi(alpha_pi) * d;
             0, 0, 0, 1];
    end
end

