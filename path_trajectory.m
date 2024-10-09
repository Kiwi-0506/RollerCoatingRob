function [p,q] = path_trajectory(ptCloud,p0,l,q0,robot,step,eps)
%path_trajectory计算下一步需要到达的位置对应的角度关节值

%% 输入参数:
% l - 可活动的杆长
% beta - 要求的夹角
% step - 规划点步长
% p0 - 目前的末端位置
% q - 目前的关节角度值
% robot - robot模型

%% Step 1: 计算出z0-step处的法向量n
% 要求包括6个分量，前3个分量为法向量起始点位置，后三个分量表示法向量方向
z0=p0(2)-step;
n = calculate_normal_vector(x0,y0,z0,ptCloud,eps);

%% Step 2: 反算出执行器应该到达的位置(x1, y1, z1)
% 将法方向绕Y轴旋转beta角，并沿该方向平移距离l
Rotation=[cos(beta),0,sin(beta);
          0,1,0;
          -sin(beta),0,cos(beta)];
d=Rotation*n(3:5);
d=l*d;
p=n(0:2)+d(0:2);

%% Step 3: 利用逆运动学ikine计算角度关节值
T=robot.fkine(q0);
q=robot.ikine(p,T);
end