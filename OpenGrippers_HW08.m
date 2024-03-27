

grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory',...
                              'control_msgs/FollowJointTrajectory')
gripGoal    = rosmessage(grip_client);
gripPos     = 0.0;
gripGoal    = packGripGoal(gripPos,gripGoal)
sendGoal(grip_client,gripGoal)


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