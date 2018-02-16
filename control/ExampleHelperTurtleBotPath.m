function ExampleHelperTurtleBotPath(handles)
    % Get goal location 
    goal = [3, 3];
    
    % Use PSO to generate path as a set of waypoints
    waypoints = [1 0; 3 0; 3 2];
    
    % Execute path following
    executePath(waypoints, goal, handles)
    
end





function res = executePath(waypoints, goal, handles)
    
    halt = false;

    gazebo = ExampleHelperGazeboCommunicator();
    kobuki = ExampleHelperGazeboSpawnedModel('mobile_base',gazebo)
    [velPub,velMsg] = rospublisher('/mobile_base/commands/velocity');

    s = size (waypoints)
    numWaypoints = s(1)
    minDistToGoal = 0.5;
    
    pp = robotics.PurePursuit;
    pp.Waypoints = waypoints
    pp.DesiredLinearVelocity = 0.5;
    
    pose = readPose(handles.odomSub.LatestMessage);
    [position, orientation, velocity] = getState(kobuki);
    pose = [position(1) position(2) orientation(1)]
    distanceToGoal = norm([pose(1) pose(2)] - goal);
    
    while (distanceToGoal > minDistToGoal && halt == false)
        
        pose = readPose(handles.odomSub.LatestMessage);
        [position, orientation, velocity] = getState(kobuki);
        pose = [position(1) position(2) orientation(1)]
        theta = deg2rad(pose(3));
        [v, w, lookaheadPoint] = step(pp, [pose(1), pose(2), theta]);
        lookaheadPoint
        [v w];

        distanceToGoal = norm([pose(1) pose(2)] - goal);

        %% CONTROL
        % Package ROS message and send to the robot

        if (distanceToGoal > minDistToGoal)
            velMsg.Linear.X = v;
            velMsg.Angular.Z = w;
            send(velPub,velMsg)
        end
    end

end



function pose = readPose(odomMsg)
%readPose Extract the robot odometry reading as [x y theta] vector

% Extract the x, y, and theta coordinates
poseMsg = odomMsg.Pose.Pose;
xpos = poseMsg.Position.X;
ypos = poseMsg.Position.Y;
quat = poseMsg.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
theta = angles(1);
pose = [xpos, ypos, theta];

end