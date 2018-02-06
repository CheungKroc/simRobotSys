function [ msg_armControllerGoal ] = trajReceiverCallback( src,msg,userData )
%A callback function used to transit information between msg receive from
%/trajectory_topic and msg wrapping into msg_armControllerGoal 
%   created by kp 2018/2/6

%PS: there should be a update version whose return parameter is the result
%rather than msg_armControllerGoal
global ActionClient_armController

msg_armControllerGoal = userData;

seqCurrent = msg_armControllerGoal.Trajectory.Header.Seq;
points = msg.Points;% for testing
% disp(msg.Points(3).TimeFromStart.Sec)
% disp(msg.Points(3).TimeFromStart.Nsec)

msg_armControllerGoal.Trajectory.Points=msg.Points;
msg_armControllerGoal.Trajectory.Header.Seq =seqCurrent + 1;

msg_armControllerGoal.Trajectory.Header.Stamp= rostime('now');
sendGoal(ActionClient_armController,msg_armControllerGoal)
end

