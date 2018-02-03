function [ x,y,z ] = line_function5( u )
%   the linefuction should be normalized 
%   the parameter u is a variable rangging from 0 to 1
%   a heart figure
t=2*pi*u;
scale_factor=0.025;
x=0.4;
y=16.*(sin(t)).^3;
y=y.*scale_factor;
z=13.*cos(t)-5.*cos(2.*t)-2.*cos(3.*t)-cos(4.*t);
z=z.*scale_factor+0.2;

end
