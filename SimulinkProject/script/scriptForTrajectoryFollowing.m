% script using robotics.InverseKinematics to calculate configuration fo ur5
% created by kp 2018/2/7

trajfun=@line_function1;
%%---------------trajectory planning-----------%
[u,t]=ParabolicBlend(trajfun,0.5,2,50);
timeFromStart=cumsum(t);
%---------------------------------------planning end-----%

%%---------------ik problem and message forming--------%
pointsNum=size(u,1);
weights=ones(6,1);
firstConfig=ur5.homeConfiguration;

[x,y,z]=trajfun(u(1,1));
[a_s,b_s,c_s]=trajfun(u(1,1),'pose');
[a_e,b_e,c_e]=trajfun(u(pointsNum,1),'pose');
R_s=rpy2r([a_s,b_s,c_s]);
R_e=rpy2r([a_e,b_e,c_e]);
quaternion_s=Quaternion(R_s);
quaternion_e=Quaternion(R_e); 

tform=transl([x,y,z]);
tform(1:3,1:3)=quaternion_s.R;
[configSoln,solnInfo] = ikSolver1('wrist_3_link',tform,weights,firstConfig);
if solnInfo.ExitFlag~=1
    warning('can not find a proper solution');
end

msgPoint=rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgPoint.TimeFromStart.Sec=floor(timeFromStart(1));
msgPoint.TimeFromStart,Nsec=uint32(timeFromStart(1)-floor(timeFromStart(1)) );
msgPoint.Positions=zeros(6,1);
msgPoint.Positions(1:6,:)=configSoln(:).JointPosition;
msgPoint.Velocities=zeros(6,1);
msgPoint.Accelerations=zeros(6,1);

msg_armControllerGoal.Trajectory.Points(1)=msgPoint;

for i=2:pointsNum-1
    vel=zeros(6,1);
    accel=zeros(6,1);
    guessConfig=configSoln;
    [x,y,z]=trajfun(u(i,1));
    tform=transl([x,y,z]);
    quaternion_m=interp(quaternion_s,quaternion_e,u(i,1));
    tform(1:3,1:3)=quaternion_m.R;
    [configSoln,solnInfo]=ikSolver2('wrist_3_link',tform,weights,guessConfig);
    for k=1:6
    p_s=guessConfig(k).JointPosition;
    p_e=configSoln(k).JointPosition;
    vel(k,1)=(p_s-p_e)/t(i);
    accel(k,1)=vel(k,1)/t(i);
    end
    
    timeSec=floor( timeFromStart(i));
    msgPoint.TimeFromStart.Sec=timeSec;
    msgPoint.TimeFromStart.Nsec=uint32((timeFromStart(i)-timeSec )*10^6);
    for k=1:6
    msgPoint.Positions(k,1)=configSoln(k).JointPosition;
    msgPoint.Velocities(k,1)=vel(k);
    msgPoint.Accelerations(k,1)=accel(k);
    end
    
    msg_armControllerGoal.Trajectory.Points(i,1)=msgPoint;
    
end

%---------------------------------------------- end-----%