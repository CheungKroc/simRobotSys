%The script to build ur5 robot as a RigidBodyTrees object
% created by kp 2018/2/7
     %  a           alpha      d         theta 
dh_ur5=[0           0          0.089159      0
        0          -pi/2       0             0  %0.135850 is measured from Gazebo  0.39225 0.4250
        0.39225     0          0             0
        0.4250      0          0             0  
        0          -pi/2       0.10915       0  %0.09465
        0           pi/2       0.09465       0];%0.0823
dh_endEffector=[1 0 0 0
                0 1 0 0
                0 0 1 0.0823
                0 0 0 1];
% T1=[];
% T2=[];
% T3=[];
% T4=[];
% T5=[];
% T6=[];
    
ur5= robotics.RigidBodyTree; %here will create a base link which is  equivalent to the base_link in the ur5Robot

%the shoulder_link of the ur5Robot assignment and added to robot
shoulder_link=robotics.RigidBody('shoulder_link');
shoulder_pan_joint=robotics.Joint('shouler_pan_joint','revolute');
setFixedTransform(shoulder_pan_joint,dh_ur5(1,:),'dh'); %use to set the JointToParentTransform property
% setFixedTransform(shoulder_pan_joint,T1); %use to set the JointToParentTransform property
shoulder_link.Joint=shoulder_pan_joint;
addBody(ur5,shoulder_link,'base');

%the upper_arm_link
upper_arm_link=robotics.RigidBody('upper_arm_link');
shoulder_lift_joint=robotics.Joint('shoulder_lift_joint','revolute');
setFixedTransform(shoulder_lift_joint,dh_ur5(2,:),'dh');
upper_arm_link.Joint=shoulder_lift_joint;
addBody (ur5,upper_arm_link,'shoulder_link');

%the fore_arm_link
fore_arm_link=robotics.RigidBody('fore_arm_link');
elbow_joint=robotics.Joint('elbow_joint','revolute');
setFixedTransform(elbow_joint,dh_ur5(3,:),'dh');
fore_arm_link.Joint=elbow_joint;
addBody(ur5,fore_arm_link,'upper_arm_link');

%the wrist_1_link
wrist_1_link=robotics.RigidBody('wrist_1_link');
wrist_1_joint=robotics.Joint('wrist_1_joint','revolute');
setFixedTransform(wrist_1_joint,dh_ur5(4,:),'dh');
wrist_1_link.Joint=wrist_1_joint;
addBody(ur5,wrist_1_link,'fore_arm_link');

%the wrist_2_link
wrist_2_link=robotics.RigidBody('wrist_2_link');
wrist_2_joint=robotics.Joint('wrist_2_joint','revolute');
setFixedTransform(wrist_2_joint,dh_ur5(5,:),'dh');
wrist_2_link.Joint=wrist_2_joint;
addBody(ur5,wrist_2_link,'wrist_1_link');

%the wrist_3_link
wrist_3_link=robotics.RigidBody('wrist_3_link');
wrist_3_joint=robotics.Joint('wrist_3_joint','revolute');
setFixedTransform(wrist_3_joint,dh_ur5(6,:),'dh');
wrist_3_link.Joint=wrist_3_joint;
addBody(ur5,wrist_3_link,'wrist_2_link');

%display which can be annotated
showdetails(ur5);
show(ur5);

%after building the robot, two InverseKinematic solver also need to be
%setup,for which, they would be used in trajectory following problem
ikSolver1=robotics.InverseKinematics('RigidBodyTree',ur5,'SolverAlgorithm','BFGSGradientProjection');
ikSolver2=robotics.InverseKinematics('RigidBodyTree',ur5,'SolverAlgorithm','LevenbergMarquardt');
