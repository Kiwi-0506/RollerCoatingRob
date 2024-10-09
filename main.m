clear;
close all;
%% STD-DH参数

%连杆偏移
d1 = 398;
d2 = -0.299;
d3 = 0;
d4 = 556.925;
d5 = 0;
d6 = 165;
%连杆长度
a1 = 168.3;
a2 = 650.979;
a3 = 156.240;
a4 = 0;
a5 = 0;
a6 = 0;
%连杆扭角
alpha1 = pi/2;
alpha2 = 0;
alpha3 = pi/2;
alpha4 = -pi/2;
alpha5 = pi/2;
alpha6 = 0;
%建立机械臂连杆
%       theta  d        a        alpha    
L1=Link([0     d1       a1       alpha1]);
L2=Link([0     d2       a2       alpha2]);L2.offset = pi/2;
L3=Link([0     d3       a3       alpha3]);
L4=Link([0     d4       a4       alpha4]);
L5=Link([0     d5       a5       alpha5]);
L6=Link([0     d6       a6       alpha6]);
%限制机器人的关节空间——关节转动最大最小角
L1.qlim = [(-165/180)*pi,(165/180)*pi];
L2.qlim = [(-95/180)*pi, (70/180)*pi];
L3.qlim = [(-85/180)*pi, (95/180)*pi];
L4.qlim = [(-180/180)*pi,(180/180)*pi];
L5.qlim = [(-115/180)*pi,(115/180)*pi];
L6.qlim = [(-360/180)*pi,(360/180)*pi];
%连接连杆，机器人取名为myrobot
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','myrobot');

%% 读取点云与目前位姿
ptCloud=pcread('filename');
l=350;      step=100;
p0=[];      q0=[];      eps=10;

%% 轨迹规划
[p,q] = path_trajectory(ptCloud,p0,l,q0,myrobot,step,eps);
