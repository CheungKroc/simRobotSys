%trajectory_subscriberInit
%A script used to create a subscriber which is subscribed to a topic named
%/trajectory_topic

trajectory_receiver=rossubscriber('/trajectory_topic','trajectory_msgs/JointTrajectory',{@trajReceiverCallback,msg_armControllerGoal});