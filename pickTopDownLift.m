% HW #8 Jaren & Josh Mechatronics
% Preliminary setup, making sure ROS is shutdown and then initialized, and
% that the robot is in an optimal configuration
rosshutdown
rosinit("192.168.19.128")
goHome('qr')
resetWorld

% Function to make sure grippers are open
OpenGrippers_HW08

%Creating an action client whose job is to follow a trajectory path for
%joint angles.
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                          'control_msgs/FollowJointTrajectory',...
                          'DataFormat', 'struct') 

%Creating a message related to the action client and naming it our goal so
%that we know it is the message that will carry the final answer we want.
trajGoal = rosmessage(trajAct)

%Getting rid of the feedback function
trajAct.FeedbackFcn = []; 

%Next, we make a subscriber in the joint_states topic becasue we are
%calculating the path to follow using joint angles.
jointSub = rossubscriber("/joint_states")

%Now we are assigning the subscribers latest messages to a variable which
%we will populate later.
jointStateMsg = jointSub.LatestMessage

%Next, we load in the robot in Matlab so that we can use it to run forward 
% and inverse kinematics guesses.
UR5e = loadrobot('universalUR5e', DataFormat="row")

%This is for adjusting the naming for the kinematics because the naming is
%done differntly in Gazebo and Matlab, but the angles need to be in the
%same order to make the correct adjustments.
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

%Now we make an inverse kinematicss object for the specific robot model
ik = inverseKinematics("RigidBodyTree",UR5e);
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1];

% Recieve the current robot confiuration and assign it to the latest
% message in the jointsubscriber we made earlier. 
jointStateMsg = receive(jointSub,3)

%Make an initial guess for the IK
initialIKGuess = homeConfiguration(UR5e)

%Now, for the message where we are putting in the inputs of the robots
%confiugration and assigning it to the newest message we need to reorder
%the names for the configurations because the order is different between
%Matlab and Gazebo and so if thinkgs are not reorder it will not call
%things correctly. 
jointStateMsg.Name
% update configuration in initial guess
initialIKGuess(1) = jointStateMsg.Position(4);  % Shoulder Pan
initialIKGuess(2) = jointStateMsg.Position(3);  % Shoulder Tilt
initialIKGuess(3) = jointStateMsg.Position(1);  % Elbow
initialIKGuess(4) = jointStateMsg.Position(5);  % W1
initialIKGuess(5) = jointStateMsg.Position(6);  % W2
initialIKGuess(6) = jointStateMsg.Position(7);  % W3

%Here we are going to define the final pose we want the arm to reach for X
%Y and Z.

gripperX = -.015;
gripperY = 0.8;
gripperZ = 0.45;
gripperTranslation = [gripperX gripperY gripperZ];
gripperRotation    = [-pi/2 -pi 0]; %  [Z Y Z] radians

tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess)

%Need to re reorder the stuff to send to Gazebo
UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)]

%Finally use packing function to fix filling in the names and position
trajGoal = packTrajGoal(UR5econfig,trajGoal)

%Lastly send it!
sendGoal(trajAct,trajGoal)

pause(5)

gripperX = -.04;
gripperY = 0.8;
gripperZ = 0.13;
gripperTranslation = [gripperX gripperY gripperZ];
gripperRotation    = [-pi/2 -pi 0]; %  [Z Y Z] radians

tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess)

%Need to re reorder the stuff to send to Gazebo
UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)]

%Finally use packing function to fix filling in the names and position
trajGoal = packTrajGoal(UR5econfig,trajGoal)

%Lastly send it!
sendGoal(trajAct,trajGoal)

% Reference function code to close grippers
pause(4)

CloseGrippers_HW08

pause(4)

gripperX = -.04;
gripperY = 0.8;
gripperZ = 0.45;
gripperTranslation = [gripperX gripperY gripperZ];
gripperRotation    = [-pi/2 -pi 0]; %  [Z Y Z] radians

tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess)

%Need to re reorder the stuff to send to Gazebo
UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)]

%Finally use packing function to fix filling in the names and position
trajGoal = packTrajGoal(UR5econfig,trajGoal)

%Lastly send it!
sendGoal(trajAct,trajGoal)














%Function For PackGoal To Reference
function gripGoal=packGripGoal(pos,gripGoal)
    jointWaypointTimes = 0.1;                                                 % Duration of trajectory
    jointWaypoints = [pos]';                                                % Set position of way points
    numJoints = size(jointWaypoints,1);                                     % Only 1 joint for gripper

    % Joint Names --> gripGoal.Trajectory
    gripGoal.Trajectory.JointNames = {'robotiq_85_left_knuckle_joint'};

    % Goal Tolerance: set type, name, and pos/vel/acc tolerance
    gripGoal.GoalTolerance = rosmessage('control_msgs/JointTolerance');
    gripGoal.GoalTolerance.Name = gripGoal.Trajectory.JointNames{1};
    gripGoal.GoalTolerance.Position = 0;
    gripGoal.GoalTolerance.Velocity = 0.1;
    gripGoal.GoalTolerance.Acceleration = 0.1;
    
    % Create waypoint as a ROS trajectory point type. Time for position will be set to TimeFromStart and then just position via JointWaypoints. t 
    trajPts = rosmessage('trajectory_msgs/JointTrajectoryPoint');

    trajPts.TimeFromStart = rosduration(jointWaypointTimes);
    trajPts.Positions = jointWaypoints;

    % Zero out everything else
    trajPts.Velocities      = zeros(size(jointWaypoints));
    trajPts.Accelerations   = zeros(size(jointWaypoints));
    trajPts.Effort          = zeros(size(jointWaypoints));
    
    % Copy trajPts --> gripGoal.Trajectory.Points
    gripGoal.Trajectory.Points = trajPts;
end

function trajGoal = packTrajGoal(config,trajGoal)
    jointWaypointTimes = 1;
    jointWaypoints = config';
    numJoints = size(jointWaypoints,1);
    
    trajGoal.Trajectory.JointNames = {'elbow_joint','shoulder_lift_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
    
    %for idx = 1:numJoints

    % trajGoal.GoalTolerance = rosmessage('control_msgs/JointTolerance','DataFormat', 'struct');
    % 
    % trajGoal.GoalTolerance.Name = trajGoal.Trajectory.JointNames{1};
    % trajGoal.GoalTolerance.Position = 0;
    % trajGoal.GoalTolerance.Velocity = 0;
    % trajGoal.GoalTolerance.Acceleration = 0;

    %end
    
    trajPts = rosmessage('trajectory_msgs/JointTrajectoryPoint','DataFormat','struct');
    trajPts.TimeFromStart = rosduration(jointWaypointTimes,'DataFormat','struct');
    trajPts.Positions = jointWaypoints;
    trajPts.Velocities = zeros(size(jointWaypoints));
    trajPts.Accelerations = zeros(size(jointWaypoints));
    trajPts.Effort = zeros(size(jointWaypoints));
    
    trajGoal.Trajectory.Points = trajPts;
end









