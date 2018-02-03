function [x,y,z] = line_function1(varargin)
%   the linefuction should be normalized
%   the parameter u is a variable rangging from 0 to 1
 %  an arbitary line form will be 
 %  x=x0+ u*delta_x
 %  y=y0+ u*delta_y
 %  z=z0+ u*delta_z

%     x=0.5*u-0.25;
%     y=0.6+0*u;
%     z=0.4+0*u;
if nargin==1
    u=varargin{1}; 
%       x=0.5*u-0.25; %该轨迹在GithubFiles/DynamicsAlgorithm/model 路径中的模型
%     y=0.6+0.2*u;
%     z=0.4+0.1*u;

    x=0.2*u-0.1;  %该轨迹是为了给GithubFiles/R-GUI/model 路径中的模型用的
    y=0.3+0.1*u;
    z=0.2+0.1*u;
    return;
elseif nargin==2 && strcmp(varargin{2},'pose')
    u = varargin{1};
end
  
if u==0   % 此时x y z相当于alpha beta gamma
    x = 0; % deg
    y= 90; 
    z= 90;
elseif u ==1 
    x= 0; %deg
    y= -90;
    z= 0;
    
else
    error('the pose at such u didn''t define');
end

end

