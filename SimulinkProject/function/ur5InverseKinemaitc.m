function [ configSoln,solnInfo ] = ur5InverseKinemaitc( Tform )
%ur5 �����˵����˶�ѧ����
%   created by kp 2018/2/8

%����˼�룺����Ŀ����̬����ԭ��̬������ܷ�����һ��Z�����X��������˶����õ�һ��λ�ã�����4�ؽ����5�ؽ������ཻ��λ��A��
%   ����λ��A�Ŀռ����� [xA,yA,zA] ���ǰ�����ؽڱ�����

%ʹ��D-H���еĶ��壬dΪ����Z����ƶ����룬aΪ����X����ƶ�����
%��λ��m
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

