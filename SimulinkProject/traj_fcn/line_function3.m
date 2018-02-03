function [x,y,z] = line_function3(u)
%   the linefuction should be normalized 
%   the parameter u is a variable rangging from 0 to 1
%   a space circle
u=u*2*pi;

n_vector=[0,-0.707106781186547,0.707106781186547]; %圆所在平面的法向量
r=0.2;% 圆的半径 单位m
origin=[0,0.5,0.4]; %圆的圆心位置，单位m


A=n_vector(1,1);
B=n_vector(1,2);
C=n_vector(1,3);

x0=origin(1,1);
y0=origin(1,2);
z0=origin(1,3);

x=x0+ r * B/(sqrt(A^2+B^2))*sin(u)  + r*(A*C)/sqrt(A^2+B^2)*sqrt(A^2+B^2+C^2) *cos(u);
y=y0- r * A/(sqrt(A^2+B^2))*sin(u)  + r*(B*C)/sqrt(A^2+B^2)*sqrt(A^2+B^2+C^2) *cos(u);
z=z0-  r*sqrt(A^2+B^2)/sqrt(A^2+B^2+C^2) *cos(u);



end

