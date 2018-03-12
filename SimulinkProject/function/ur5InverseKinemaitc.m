function [ configSoln,solnInfo ] = ur5InverseKinemaitc( Tform )
%ur5 机器人的逆运动学方程
%   created by kp 2018/2/8

%基本思想：先让目标姿态沿着原姿态的坐标架方向做一次Z方向和X方向的逆运动，得到一个位置，即第4关节与第5关节轴线相交的位置A。
%   利用位置A的空间坐标 [xA,yA,zA] 求出前三个关节变量。

%使用D-H法中的定义，d为沿着Z轴的移动距离，a为沿着X轴的移动距离
%单位：m
wrist3Link_d=0.0823;
wrist2Link_d=0.09465;
wrist1Link_d=0.10915;

shoulder_pan_link_d=0.089159;
%shoulder_pan_link_a=0.13585%guess
shoulder_lift_link_a=0.39225;
%elbow_link_d=???%
elbow_link_a=0.4250;

%firstStep: position and pose should travel back to the crosspoint of the
%axes of the fourth joint and the fifth joint

zAxis_endReverse=Tform(1:3,3)*-1;
xAxis_endReverse=Tform(1:3,3)*-1;
T_travelBack=transl([-wrist2Link_d,0,-wrist3Link_d]);
T_crosspoint=Tform * T_travelBack;

%secondStep: 

end

