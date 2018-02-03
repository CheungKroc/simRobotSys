%simpleScriptForLoadingData
% created by kp 2018/2/2
%load the data named Data_armCntroFollowJointTrajGoal0.mat which has contained 
%the data fork from the ROS-based simulation robot

load('D:\Program Files\MATLAB\MATLAB user files\SimulinkProj\SimulinkProject\data\Data_armCntrolFollowJointTrajGoal0.mat');
msg_armControllerGoal.Trajectory.Points=trajectory_points
