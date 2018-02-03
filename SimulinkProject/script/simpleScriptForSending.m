%simpleScriptForSending
% created by kp 2018/2/2
%use rostime to initial the stamp in the Header and use sendGoalAndWait()
%function to send the goal

msg_armControllerGoal.Trajectory.Header.Stamp= rostime('now');
sendGoal(ActionClient_armController,msg_armControllerGoal)