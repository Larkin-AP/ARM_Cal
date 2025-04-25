% temp_cal
addpath('E:\Research\Research\0000_matlab_math_lib\code');

syms x y z real
d = [x,y,z]';
temp1 = d*d'

temp2 = eye(3)+vecToLieAlgebra(d)*vecToLieAlgebra(d)