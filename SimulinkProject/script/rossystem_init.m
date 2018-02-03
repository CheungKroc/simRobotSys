%rossystem_init
%A script-like function used for setting up the environment of MATLAB-ROS
%collaborating system. 
%   created by kp on 2018/2/2

%% initializing the MATLAB as a sub-node of ROS
%  In order to use rosinit(),you need to ensure MATLAB-slaver and ROS-host
%  should be in the same network. Secondly, you need to know the IP address
%  of ROS-host or its name,while the port of the ROS should also be
%  known,in which case its IP address is 192.168.199.178,the port is
%  11311,while its name is kp-Q170-4S.
rosinit('http://192.168.199.178:11311','NodeHost','192.168.199.174','NodeName','/MATLAB_node')

%-----------------------------------------------------------------------%

%% building some useful rosclient for the provided rosservice
%  when you want to use the service,just call(client,msg,'Timeout',a-specific-time)

% 以下语句暂时无法使用 
% client_ListControllerTypes=rossvcclient('/controller_manager/list_controller_types')
% msg_ListControllerTypes=rosmessage(client_ListControllerTypes)

% client_ListController=rossvcclient('/controller_manager/list_controllers')
% msg_ListControllers=rosmessage(client_ListController)
% controllerList=call(client_ListController,msg_ListControllers,'Timeout',10)
%-----------------------------------------------------------------------%

%% constructing the actionclient for action named /arm_controller/follow_joint_trajectory
% It will return two object:a SimpleActionClient and a Goalmsg used for
% conmunicating with actionserver
[ActionClient_armController,msg_armControllerGoal]=rosactionclient('/arm_controller/follow_joint_trajectory')
waitForServer(ActionClient_armController)

%fill in the msg_armControllerGoal
msg_armControllerGoal.Trajectory.JointNames = {'elbow_joint';'shoulder_lift_joint';'shoulder_pan_joint';'wrist_1_joint';'wrist_2_joint';'wrist_3_joint'};
msg_armControllerGoal.Trajectory.Header.FrameId='/world';


%-----------------------------------------------------------------------%