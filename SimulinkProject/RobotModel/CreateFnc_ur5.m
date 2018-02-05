function [ ur5 ] = CreateFnc_ur5( )
% Create model of UR5 manipulator
%
%      mdl_ur5
%
% Script creates the workspace variable str6_05 which describes the 
% kinematic and dynamic characterstics of a STR6_05 manipulator 
% modified DH conventions.
%
% Also defines the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qstretch   arm is stretched out in the X direction
%
%
% See also SerialLink, mdl_puma560, mdl_stanford, mdl_twolink.

% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

%------------definition of the global varient----%
% global qz qr qstretch;

%------------------------------------------------%
deg = pi/180;



% 单位m      theta    d        a         alpha     sigma   'mod' means modified  D&H
L_1 = Link([  0    0.089159   0         pi/2       0]);
L_2 = Link([  0    0         -0.4250    0          0]);
L_3 = Link([  0    0         -0.39225   0          0]);
L_4 = Link([  0    0.10915    0         pi/2       0]); 
L_5 = Link([  0    0.09465    0        -pi/2       0]); 
L_6 = Link([  0    0.0823     0         0          0]);

L_2.offset=-pi/2; %剩余的偏差用偏置量来平衡
L_4.offset=-pi/2;

%link limit 暂时没有针对ur5 作修改
L_1.qlim=[-170*deg,170*deg];
L_2.qlim=[-100*deg,135*deg];
L_3.qlim=[-120*deg,155*deg];
L_4.qlim=[-150*deg,150*deg];
L_5.qlim=[-120*deg,120*deg];

% link mass 单位kg
L_1.m = 3.7000; %Link1 不等于基座
L_2.m = 8.3930; 
L_3.m = 2.2750;
L_4.m = 1.2190;
L_5.m = 1.2190;
L_6.m = 0.1879;

% link COG wrt link coordinate frame 单位m
%              rx      ry      rz
L_1.r = [0.0        -0.02561    0.00193  ];
L_2.r = [0.2125      0.0        0.11336  ];
L_3.r = [0.11993     0.0        0.0265   ];
L_4.r = [0.0        -0.0018     0.01634  ];
L_5.r = [0.0        -0.0018     0.01634  ];
L_6.r = [0.0         0.0       -0.001159 ];

% link inertia matrix 杆件惯量矩阵 （对称阵，只有6个独立变量）单位kg.m^2  暂时不予以修改
%               Ixx     Iyy      Izz    Ixy     Iyz     Ixz
% L(1).I = [149235.06563   122855.98517   88940.95787   65.78546    4221.08184     -7042.04735];
% L(2).I = [381483.81049   77623.15382    354522.48090  1432.95781  -13580.61240   34.24686  ];
% L(3).I = [48300.98308    21681.69783    50694.71952   4707.52143  -1574.17921    -126.70180 ];
% L(4).I = [34305.58253    30830.17616    15930.40518   38.61385    -1339.18046    -9.65035 ];
% L(5).I = [1649.30175     5625.39070     5927.83168    -95.13313   -32.78135      -30.18877];
% L(6).I = [18.90213       20.86480       356.9729      0.04717     -0.07094       0.11325 ];

L_1.I = [149235.06563   122855.98517   88940.95787   65.78546    4221.08184     -7042.04735]*1.0e-06;
L_2.I = [381483.81049   77623.15382    354522.48090  1432.95781  -13580.61240   34.24686  ]*1.0e-06;
L_3.I = [48300.98308    21681.69783    50694.71952   4707.52143  -1574.17921    -126.70180 ]*1.0e-06;
L_4.I = [34305.58253    30830.17616    15930.40518   38.61385    -1339.18046    -9.65035 ]*1.0e-06;
L_5.I = [1649.30175     5625.39070     5927.83168    -95.13313   -32.78135      -30.18877]*1.0e-06;
L_6.I = [18.90213       20.86480       356.9729      0.04717     -0.07094       0.11325 ]*1.0e-06;

% motor inertia 电机惯量
L_1.Jm =  291e-6;
L_2.Jm =  409e-6;
L_3.Jm =  299e-6;
L_4.Jm =  35e-6;
L_5.Jm =  35e-6;
L_6.Jm =  35e-6;

% Gear ratio  齿轮传动比
L_1.G =  80;
L_2.G =  120;
L_3.G =  100;
L_4.G =  80;
L_5.G =  80;
L_6.G =  80;
% % 
% % viscous friction (motor referenced)
% % unknown
% % 
% % Coulomb friction (motor referenced)
% % unknown

%
% some useful poses
%
% qz = [0 0 0 0 0 0]; % zero angles, L shaped pose
% qr = [0 0 -pi/2 0 0 0]; % ready pose, arm up
% qstretch = [0 pi/2 -pi/2 0 0 0]; % horizontal along x-axis

ur5 = SerialLink([L_1 L_2 L_3 L_4 L_5 L_6], 'name', 'UR5', 'manufacturer', 'UniversalRobot', 'comment', 'theory');

%the world frame to the coordinate of base_link is rpy=[0 0 -pi] xyz=[0 0 0]
ur5.base=[ -1    0   0   0
            0   -1   0   0
            0    0   1   0
            0    0   0   1   ];

end

