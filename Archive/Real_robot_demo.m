rosshutdown;
rosinit('192.168.0.1'); % If unsure, please ask a tutor
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

pose_1 = [-0.4 0 0 0 0 0 -pi/2];
%%
pause(2); % Pause to give time for a message to appear

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');

goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.10);

bufferSeconds = 3; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 10; % This is how many seconds the movement will take

%%
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0.10);

nextJointState_123456 = pose_1;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

sendGoal(client,goal);

pause(5);