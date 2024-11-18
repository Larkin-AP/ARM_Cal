% 定义符号变量
clear all
clc
syms theta_11 theta_12 x y z d1 d2 d3 d4 d5 real


% d1 = 39.5;
% d2 = 2.5;
% d3 = 157.7;
% d4 = 12;
% d5 = 2.5;

a=sin(theta_11);
b=cos(theta_11);
theta_12 = asin(-z/d1);


eq1 = 12*b-2.5*a+9*b*cos(theta_12)+156 == x;
eq2 = 2.5*b+12*a+9*cos(theta_12)*a == y;

(12+9*cos(theta_12))*a == y-2.5*b;
a =(y-2.5*b)/(12+9*cos(theta_12));
(12+9*cos(theta_12))*b -2.5*a+156 == x;
(12+9*cos(theta_12))*b -2.5*(y-2.5*b)/(12+9*cos(theta_12))+156 == x;
(12+9*cos(theta_12)+6.25/(12+9*cos(theta_12)))*b==x-156+(2.5*y)/(12+9*cos(theta_12));


b = (x-d3+((d2+d5)*y)/(d4+d1*cos(theta_12)))/((d4+d1*cos(theta_12))+(d2+d5)/(d4+d1*cos(theta_12))*(d2+d5));
% b = (x-d3+((d2+d5)*y)/(12+9*cos(theta_12)))/(12+9*cos(theta_12)+6.25/(d4+d1*cos(theta_12)));
a = (y-(d2+d5)*b)/(d4+d1*cos(theta_12));
theta_11_s = asin(a);
theta_11_c = acos(b);
theta_11_t =atan2(a/b);










