%wrist IK cal
function output = wrist_IK_cal(ee_point)
% ee_point 是末端P点在局部坐标系下的表示(x,y,z)，单位mm
% output 是支链直线电机的进给量，单位mm
% 零点 （209.2，0，0）

% 直线电机初始长度，单位mm
l20= 153.78;
l30= 153.78;

d1 = 39.5;
d2 = 2.5;
d3 = 157.7;
d4 = 12;
d5 = -2.5;

x = ee_point(1);
y = ee_point(2);
z = ee_point(3);
theta_12 = asin(-z/d1);

% a=sin(theta_11) b=cos(theta_11)

b = (x-d3+((d2+d5)*y)/(d4+d1*cos(theta_12)))/((d4+d1*cos(theta_12))+(d2+d5)/(d4+d1*cos(theta_12))*(d2+d5));
a = (y-(d2+d5)*b)/(d4+d1*cos(theta_12));

theta_11_s = asin(a);
theta_11_c = acos(b);
theta_11_t =atan2(a,b);

Pp_G = [d1,d2,0,1]'; % P在G坐标系下的表示
T_11 = [1,0,0,d3;
    0,1,0,0;
    0,0,1,0;
    0,0,0,1];
T_12 = [1,0,0,d4;
    0,1,0,d5;
    0,0,1,0;
    0,0,0,1];
T_BG = T_11*homogeneousRotation('z',theta_11_t)*T_12*homogeneousRotation('y',theta_12);
Pp_B = T_BG*Pp_G;
% Pp_B = T_BG*Pp_G; % Pp_B是P点在B坐标系下的坐标，这里可以求出theta_11和theta_12

% P_B是知道了末端在局部坐标系下的位置。但是这个地方还无法直接求出反解
% 同理知道P_B,可以知道G点的位置，知道G点位置，可以表示出C和D点位置
% 知道P点位置，可以直接求出beta（绕y转动的角度）

Pg_G=[0,0,0,1]'; % G点在G坐标系下的表示
Pg_B=T_BG*Pg_G; % G点在B坐标系下的表示
Vcg_G=[12,19.1,-28,0]'; % vector_CG在G下的表示
Vcg_B = T_BG*Vcg_G;

Pc_B = Pg_B-Vcg_B;
Pa_B = [5.5,0,32,1]';
V_l2 = Pc_B-Pa_B;
V_l2 = V_l2(1:3);
length_V_l2 = norm(V_l2);
% 到这里表示出来了支链2的长度，通过求模可以求出最后的推杆变化量
% 类似地，可以求支链3的长度

Vdg_G = [12,19.1,28,0]';% vector_DG在G下的表示
Vdg_B = T_BG*Vdg_G;
Pd_B = Pg_B-Vdg_B;
Pe_B = [5.5,0,-32,1]';
V_l3 = Pd_B-Pe_B;
V_l3 = V_l3(1:3);
length_V_l3 = norm(V_l3);

% 手腕球关节摆角约束,这里计算约束的时候，考虑的是不带局部自由度时候，这个有点对不上，数值上还差一点
V_scd_axis_B = Pd_B-Pc_B;
V_scd_axis_B = V_scd_axis_B(1:3);
V_sed_axis_B = Pd_B-Pe_B;
V_sed_axis_B = V_sed_axis_B(1:3);
dot_prod1 = dot(V_sed_axis_B,V_scd_axis_B);
norm_cd = norm(V_scd_axis_B);
norm_ed = norm(V_sed_axis_B);
angle_1 = acos(dot_prod1/(norm_ed*norm_cd));
angle_D = (pi/2-angle_1)/pi*180;

V_sdc_axis_B = Pc_B-Pd_B;
V_sdc_axis_B = V_sdc_axis_B(1:3);
V_sac_axis_B = Pc_B-Pa_B;
V_sac_axis_B = V_sac_axis_B(1:3);
dot_prod2 = dot(V_sdc_axis_B,V_sac_axis_B);
norm_dc = norm(V_sdc_axis_B);
norm_ac = norm(V_sac_axis_B);
angle_2 = acos(dot_prod2/(norm_dc*norm_ac));
angle_C = (pi/2-angle_2)/pi*180;


% 输出为复数或超出范围则返回空
if imag(length_V_l2) > 1e-3 || imag(length_V_l3) > 1e-3 || length_V_l2 < 132.5 || length_V_l2 > 173.5 || length_V_l3 < 132.5 || length_V_l3 > 173.5 ...
|| theta_11_t >pi/2 || theta_11_t < -pi/2 ...
    %      || angle_C < -35 || angle_C > 35 || angle_D < -35 || angle_D > 35
    output = [];
else
    delta_l2 = length_V_l2-l20;
    delta_l3 = length_V_l3-l30;
    output = [delta_l2,delta_l3];
end
end