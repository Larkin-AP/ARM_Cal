clear
clc
% 添加my_lib
addpath('E:\Research\Research\0000_matlab_math_lib\code');

%此处的计算参考曹行的论文进行计算，目前的版本先进行初步验证，当作标准SRS，以第六关节的轴线作为第二个S的点

% 第一步，关于臂型角的定义，这个地方的臂型角的定义我认为曹行的论文计算过于复杂了，参考博文中的方式

%----符号定义------%
%臂型角定义为phi
%定义SRS的相关向量
%SE=vec_e
%EW=vec_f (forearm)
%SW=vec_w

%下面这三个变量是常数
%length_SE = l_e
%length_EW = l_f
%length_SW = l_w

% 一般来说机械臂反解认为是已知臂型角和末端位姿
% 在曹行的论文中，同样也给出了臂型角的计算方式，但是这里认为臂型角是已知的，值得说明的是，经过我个人的理解，认为曹行的计算是没有问题的，因此可以直接参考答案
%D-H参数参考FK中的注释，值得注意的是，此处计算


%----------------特别注意--------------%
%%%%%%%%%%%%%%%%%%%%%%
%                    %
%   以第六关节为S点  %
%   这里的计算，传入的法兰处的位姿（坐标系8）
%   坐标7和坐标系6重合，这个重合的7认为是7v
%   实际的7认为是7a
%%%%%%%%%%%%%%%%%%%%%

%这里涉及参考平面的设计，但是目前还没搞明白这点，所以先按照论文中照猫画虎，后面再检查这个论文


%----------------特别注意--------------%

%{
改进DH方法建系

关节        a_i         alpha_i     d_i         theta_i
1           0           0           d_bs        theta_1+pi                  
2           0           pi/2        0           theta_2-pi/2
3           0           pi/2        d_se        theta_3+pi/2
4           a_se       -pi/2        0           theta_4
5           0           pi/2        d_ew        theta_5
6           0           pi/2        0           theta_6+pi/2
7           a_wf        pi/2        d_wf        theta_7

d_bs=158.3
d_se=359.6
d_ew=256.9
d_wf=5
a_wf=0
a_se=9

%}

%常量
d_bs=158.3;
d_se=359.6;
d_ew=256.9;
d_wf=0;
a_wf=12;
a_se=9;
a_ee=39.5;

% theta1 = 0;
% theta2 = 0;
% theta3 = 0;
% theta4 = 0;
% theta5 = 0;
% theta6 = 0;
% theta7 = 0;


% 常量符号
% syms d_bs d_se d_ew a_wf a_se d_wf a_ee real


% 反解的已知量：T_07, phi，这里T_07随便给了一个
% T_07 = [[1,0,0;
%         0,0,1;
%         0,-1,0],[d_se+d_ew;0;d_bs];
%     [0,0,0],1];

%传入变量是T_08,是法兰位姿，姿态与坐标系7相同
T_08 = [[1,0,0;
        0,0,1;
        0,-1,0],[d_se+d_ew+a_wf+a_ee;a_se;d_bs];
    [0,0,0],1];

% 手腕关节在base的位置
p7=T_08*[-a_wf,0,0,1]';



%T_07与T_08朝向一致，缺少一个法兰偏置
T_07 = T_08;
T_07(1:4,4) = p7;

%臂型角
phi=0;

%----从这里开始计算---%

% 根据dh参数和目标位姿有以下已知,这里我认为原论文的notation有问题，没有说清楚，他的notation不好，这里我进行修正，下面这些向量都在base坐标系（0坐标系）下进行表示：
p_02 = [0,0,d_bs]'; % base -- shoulder
p_24 = [d_se,0,0]'; % shoudler -- elbow 
p_46 = [d_ew,0,0]'; % elbow -- wrist 
p_78 = [a_wf,0,0]'; % wrist -- flange % 这个地方和原文有点差别，我的6和7认为是重合的
%这几个向量和他定义的差别还蛮大，这些向量都是在0坐标系下表示的

%根据传入的数据进行变脸计算，现在这些7都是7v
R_07 = T_07(1:3,1:3);
p_07 = T_07(1:3,4);

% 肩腕关节向量表示为，这个地方p_07是已知的，p_02是在0坐标系下的固定向量不会变化，p_67是初始位置下在0坐标系下的表示，因此要加上坐标变换，要转移到当前的0坐标系下
% 这个地方出现了问题，就是说因为肘部带偏置，所以这个theta4不好直接这样计算
% 那现在要重新计算theta4
% 这里参考曹行论文的计算方法，传入的末端姿态是法兰处的末端姿态
% T_07的点，是法兰处的点在Base的坐标，现在是要获得腕关节在Base的坐标
% 知道法兰在Base下的坐标，知道法兰在腕关节的坐标，要求腕关节的坐标
% 其实法兰只是在坐标系7下的一个点，但是在反解过程中，我们还是认为有一个坐标系8
% 坐标系8和坐标系7的朝向一致，沿着7的x的正方向偏置了a_wf的距离
% 在求解过程中，认为坐标系7和6的原点重合，这个7认为是7v，只有这样处理，才会变成标准的带偏置的SRS机械臂
% 反解过程中，给定的是坐标系8的位姿（也就是法兰的位姿），需要得到手腕的位姿
% 也就是沿着坐标系8的x的负方向，平移a_wf距离，应该就是腕关节的位姿
% 对于正运动学来讲，腕关节的位置计算[pw;1] = A1*A2*A3*A4*[0,-d_ew,0,1];

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 
A1=modified_DH_transform(theta1+pi,d_bs,0,0);
A2=modified_DH_transform(theta2-pi/2,0,0,pi/2);
A3=modified_DH_transform(theta3+pi/2,d_se,0,pi/2);
A4=modified_DH_transform(theta4,0,a_se,-pi/2);
A5=modified_DH_transform(theta5,d_ew,0,pi/2);
A6=modified_DH_transform(theta6+pi/2,0,0,pi/2);
A7=modified_DH_transform(theta7,0,a_wf,pi/2);
A8=modified_DH_transform(0,0,a_ee,0);

R567 = A5*A6*A7
R567 = simplify(R567(1:3,1:3));
R567


w_pos = A1*A2*A3*A4*[0,-d_ew,0,1]'; % 到这里通过数值验证都没问题
w_pos = simplify(w_pos(1:3));


x = w_pos(1);
y = w_pos(2);
z = w_pos(3);



%{
现在这么多问题主要都是肘部带偏置而引出的，theta4一旦计算得出，后面的计算都会顺利很多
肘部偏置实际上就是a_se的偏差
思路一，如何考虑在肘部带偏置的情况下，计算出theta4，参考曹行的论文，他那种方法具有特殊性，很多情况下，无法消去theta1，2，3的影响
最起码我从我的运动学正解中没有获得一个显示的只包含theta4的方程（80%的概率没有，我猜测）
那么要进一步看一下，能不能从几何中获得求解方法
%}

%下面的分析，我目前感觉都是解之一，并不是所有的解，这个后面再继续验证吧，问题应该都是出在theta4

%求出肩腕关节（带偏置）的向量，这个7是7v
p_27 = p_07-p_02;
vec_w = p_27;
%这个地方有点区分，主要是肩肘关节的向量要加上偏置
d_se_offset = sqrt(d_se^2+a_se^2);
beta = acos((d_se_offset^2+d_ew^2-norm(vec_w)^2)/(2*d_ew*d_se_offset));
% 上面这个要分情况进行讨论，根据推导，beta可能在上也可能在下，也就是会有两种情况
% 目前还没有找到办法能直接区分出两种情况，为了验证方便，现在先以肘关节零度开展计算
% 肘关节为零度的时候，也就是取在下面的情况
beta1 = atan2(a_se,d_se);
theta4_down = beta+beta1-pi; % 到这里以零位来看没问题

% beta是实际的臂内角,上面的这个beta算法，应该适用的是肩腕关节向量位于w1之上，也就是臂内角大于180°的时候
% w1的定义是，当肩腕关节向量w与SE向量共线。

% 这里要讨论一个问题，也就是其实是不知道w向量的，这个w向量是定不下来的
% 所以只能知道边长，和借助余弦定理，这里有一点是，肘关节是单向弯曲的
% 也就是，在只考虑位置的情况下，theta5，6，7是不发挥作用的，theta1，2，3，4决定末端位置
% theta


%当臂内角大于180°时

%求初始位置时候的beta0,向量都是在base下的初始位置表示的
% vec_SE = [d_se,a_se,0]';
% norm_x = [1,0,0]';
% beta1 = acos(dot(vec_SE,norm_x)/(norm(vec_SE)*norm(norm_x)));
% beta0 = pi+beta1;
% theta4 = beta0-beta;

theta4 = theta4_down
%这里theta4 求解完毕



%前三个位姿矩阵的正运动学矩阵，用于求解反解
% T_03_phi = A1*A2*A3;


% 定义参考向量vec_v_hat ，参考向量vec_v_hat用于确定参考平面
vec_v_hat = [0,1,0]'; %随便定的，后面再改
vec_w_hat = vec_w/norm(vec_w);

%重点是要表示出初始平面内关节4轴线的位置，按照曹行的论文
z4 = cross(vec_w_hat,vec_v_hat); %这个与参考平面有关，参考平面由参考向量v确定
%这个z4完全由前三个转动决定,z4已经是单位向量了

z4_hat = z4/norm(z4);

% 计算alpha，alpha是由目标位置和构型确定，alpha =angle_E'SW
% 当给定末端位姿时，根据前面的计算p_26可以计算出来，根据三角形余弦定理
% 这个地方需要进行修正，这个地方的角度需要有一个判断机制
% 这个地方我们的手臂，只可以向内屈臂，所以可以比较
% 腕关节可以只向上，所以加上判断，要通过什么来判断呢
% 可以通过向量叉乘进行判断，接下来试一下
% 这里我目前认为还有点小问题，就是关于向量的参考系的问题，此处取得是最简单的情况，也就是暂时不考虑手臂旋转的情况



% vec_SE = [d_se,0,-a_se]';
% vec_SW = p_26;
% %参考法向量 n
% vec_n = [0,1,0]';
% ESW_sign = cross(vec_SE,vec_SW);
% sign1 = dot(vec_n,ESW_sign);
% if sign1 >=0
%     angle_ESW = acos((vec_SW'*vec_SE)/(norm(vec_SE)*norm(vec_SW)))
% else
%     angle_ESW = -acos((vec_SW'*vec_SE)/(norm(vec_SE)*norm(vec_SW)))
% end
% 
% angle_EvSE = atan2(a_se,d_se);
% angle_EvSW = angle_ESW+angle_EvSE;
% alpha = angle_EvSW;

% 上面写的可能还是不一定对
% 根据之前的推理，现在考虑elbow down的情况
% 现在推理，这里的alpha应该和elbow 的情况是绑定的
% 在elbow down的情况下，alpha =angle_EvSW，EvSW=ESEv-ESW
% 重点就是ESW具有方向性
% 在elbow down的情况下
vec_SW = p_27;
len_SW = norm(vec_SW);
len_EW = d_ew;
angle_ESW = acos((len_SW^2+d_se_offset^2-len_EW^2)/(2*len_SW*d_se_offset));
angle_EvSE = atan2(a_se,d_se);
angle_EvSW = angle_EvSE-angle_ESW;
alpha = angle_EvSW;

% 经过理论确认，都是把w绕着z4 顺时针转动alpha，再绕z4逆时针转动pi/2
% z4垂直纸面向外
%绕着z4顺时针旋转alpha
R41 = eye(3)+sin(-alpha)*vecToLieAlgebra(z4_hat)+(1-cos(-alpha))*vecToLieAlgebra(z4_hat)^2;
%绕着z4逆时针旋转pi/2
R42 = eye(3)+sin(pi/2)*vecToLieAlgebra(z4_hat)+(1-cos(pi/2))*vecToLieAlgebra(z4_hat)^2;
%x4
x4 = R42*R41*vec_w;
x4_hat = x4/norm(x4);
y4_hat = cross(z4_hat,x4_hat);

% 根据曹行的论文来看，应该可以采取类似的方式。z4的转轴垂直于参考平面，这个地方应该没问题，无非是正负的问题
% vec_w绕着z4逆时针旋转alpha，再顺时针旋转pi/2就可以得到x4
% 这个地方的alpha是肘关节偏置引起的角度alpha =angle_E'SW,

R_04_v=[x4_hat,y4_hat,z4_hat];  

R_03_v= [x4_hat,z4_hat,-y4_hat];  

%%

%{ 
现在重点理解这个z4的姿态向量的问题
首先可以确定的是，R_03_v确实是表示的坐标系4的姿态（相对于坐标系0），当臂型角为0的时候
%}

%R_0phi 实际上就是表示参考平面绕着vec_w向量进行旋转
p_27_hat = p_27/norm(p_27);
R_0phi = eye(3)+sin(phi)*vecToLieAlgebra(p_27_hat)+(1-cos(phi))*vecToLieAlgebra(p_27_hat)^2;

% R_03_v是臂型角为0的时候的坐标系4的姿态矩阵
% R_03是臂型角为phi的时候的坐标系4的姿态矩阵
R_03 = R_0phi*R_03_v; %到这里都没问题

% 根据运动学正解反解出来的角度
theta2 = asin(-R_03(3,3))
theta3 = atan2(R_03(3,1),R_03(3,2))
theta1 = atan2(R_03(2,3),R_03(1,3))

A1=modified_DH_transform(theta1+pi,d_bs,0,0);
A2=modified_DH_transform(theta2-pi/2,0,0,pi/2);
A3=modified_DH_transform(theta3+pi/2,d_se,0,pi/2);
A4=modified_DH_transform(theta4,0,a_se,-pi/2);
R1234 =A1*A2*A3*A4;
R1234 = R1234(1:3,1:3);
R567 = R1234\R_07;
theta6 = asin(-R567(2,3))
theta7v = atan(R567(2,2)/R567(2,1))
theta5 = atan2(R567(3,3),R567(1,3))

% 到这里，在elbow down的情况下的theta都求解好了
% 这里有几点要注意
% 1. 这里是传入的法兰位姿
% 2. 这里的theta7是theta7v，还需要进一步从theta7v变成theta7
% 7v和7a的坐标系方向一样，比如认为7a是在7v的x正方向偏移了d_offset7
d_offset7 = 5; % d_offset7随便给的
temp_t = sin(theta7v)*a_wf;
temp_d = cos(theta7v)*a_wf;
theta7a = atan(temp_t/(temp_d-d_offset7));

% 到这里之后，等效串联机械臂的运动学全部解出
% 进一步要接触机构中的实际电机角度

%肘关节直线电机，符号与草稿纸对应
d_bx = 214.6;
d_by = 15;
l_b = sqrt(d_bx^2+d_by^2);
l10 = 191.2;%推杆的初始长度
l_offset = sqrt(15^2+23.4^2);
% alpha2 = atan(23.4/15)+pi/2; % 连杆的固定偏转角度
theta_c0 = acos((l_b^2+l_offset^2-l10^2)/(2*l_b*l_offset)); % 参考theta_c
% alpha2_0 =theta_c0+alpha2; %初始位置时候的参考角度
theta_c = theta4+theta_c0 ; % 这个是化简过，alpha消掉了
l1 = sqrt(l_b^2+l_offset^2-cos(theta_c)*2*l_b*l_offset);
delta_l1 = l1-l10; % 这个地方delta_l1是肘关节推杆的变化的长度

%接下来是如何根据theta6个theta7 解算肘关节的角度









% 通过正运动学得到的结果
% R_03_2 = T_03_phi(1:3,1:3);
% R_03_2是通过正运动学求解得到的，主要是为了求反解的解析式
% R_1 是把0转到1，受theta1的影响
% R_3是把0转到3，受theta123的影响
% 可以确定的是z4 肯定不受theta4的影响
% z4 应该和theta3是固定的相对姿态是固定的
% 但是我认为z4 应该会相对R123 有个相对位姿
% 那这里就试试吧

%通过之前的正运动学来看，由此解出的反解为：
%这里正运动学可能也要验证，因为正运动学没有验证，这个和建模有关


syms d_bs d_se d_ew d_wf a_wf a_se theta1 theta2 theta3 theta4 theta5 theta6 theta7 


R123 = A1*A2*A3;
R123 = simplify(R123(1:3,1:3));
R123

R567 = A5*A6*A7
R567 = simplify(R567(1:3,1:3));
R567

% theta2 = asin(-R33);
% theta3 = atan2(R31,R32);
% theta1 = atan2(R23,R13);







% 
% 
% %进一步求后三个角度，也是通过正运动学
% A1=modified_DH_transform(theta1+pi,d_bs,0,0);
% A2=modified_DH_transform(theta2-pi/2,0,0,pi/2);
% A3=modified_DH_transform(theta3+pi/2,d_se,0,pi/2);
% A4=modified_DH_transform(theta4,0,a_se,-pi/2);
% A5=modified_DH_transform(theta5,d_ew,0,pi/2);
% A6=modified_DH_transform(theta6+pi/2,0,0,pi/2);
% A7=modified_DH_transform(theta7,d_wf,a_wf,pi/2);
% R567 = A5*A6*A7;
% R567 = simplify(R567(1:3,1:3))
% 
% 
% R1234 =A1*A2*A3*A4;
% R1234 = simplify(R1234(1:3,1:3))
% R567 = R1234\R_07;
% theta6 = acos(R567(2,3))
% theta7 = atan(-R567(2,2)/R567(2,1))
% theta5 = atan(R567(3,3)/R567(1,3))








% %关节4的姿态矩阵可以写作
% R_04 = R_0phi*R_04_v;
% %这个约束条件等效于
% R_03 = R_0phi*R_03_v;
% %因为theta4相同，对于虚拟机械臂和实际的机械臂，所以有R_34_v= R_34



%在原文的公式肩腕关节向量 p_26=p_07-p_02-(R_07*p_67)
%这里要注意下在论文这种notation的标记p_26是向量，从2到6，所以(R_07*p_67)是将末端执行器的相对位置转换到与肩膀的相对位置上，以得到正确的肩-腕向量。
%所以原文的公式，实际上就是都转到base坐标系下vec_w.
%那在这里他的dh参数和我的dh参数会有一些出入，所以需要自己重新计算。
%{
论文中的向量
p_02  d_bs  base - shoulder
p_24  d_se  shoulder - elbow
p_46  d_ew  elbow -  wrist
p_67  d_wf  wrist - flange

关键是最后一个p_67,他有一个flange的变量，在我目前的建模中是没有考虑的
实际上，在整个机械臂的建模中，最大的区别就是肘关节的偏置和第七关节，原论文的建模考虑的是经典DH参数，一定要注意区分。
所以他的关节标注符号和我是有差别的，对于他来说他确实也会有一个末端偏置

那么对于我来说，好像也可以参考，首先我的肩关节是很标注的S,我的腕关节，可以按照之前的看法，虚拟成一个球关节，之后再进行坐标变化，也就是关节6处也是我的腕关节
同时也设置一个法兰向量，对于反解来说，知道法兰向量的位姿，法兰向量的位姿描述的是工具盘相当于base的姿态
A1描述的是坐标系1相对于Base的变化（受关节1的影响），那么对于T_07=A1*A2*A3*A4*A5*A6*A7;
那么T_07是坐标系7相对于Base的姿态，但是反解是考虑工件坐标系，工件坐标系的姿态与R_07相同，增加了一个偏置。
那么也就是说，假设现在a_wf是工件坐标系的偏置
对于运动学反解知道是
T07 = [R_07, p_07;
    0,1]
p_07是法兰相对与base的位置向量，要先获得肩腕关节向量

%}










%那做的这里，初步的反解就做完了
%接下来还有两个问题
% 问题1，这里还存在一个问题是如何把theta7正确的求出来
% 问题2，把这些问题角度转化为直线关节模组的角度

% 问题1，把正确的theta7求出来
%{
目前的theta7是在关节6处的虚拟角度，我们给出的是法兰的位姿情况
也就是说，通过这个虚拟转动，使得末端具有我们想要的位姿。
进一步的描述来说，在wrist关节处，让虚拟theta7（theta7_v）旋转一个角度，让theta7（实际的转轴）到那里具有同样的效果
我们知道，theta7和theta7_v的相对位姿关系（相同的姿态，存在一个位置偏置）
从简单的几何来看
目前推测theta7=theta7_v+gamma,gamma由末端构成的三角形求解而来
%}

% 至此，该机构等效的串联机械臂的反解就完全解出了，接下来要进行上述内容的验证和简化










