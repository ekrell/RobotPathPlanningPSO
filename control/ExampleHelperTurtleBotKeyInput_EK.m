classdef ExampleHelperTurtleBotKeyInput_EK < handle
    %ExampleHelperTurtleBotKeyInput - Class for obtaining keyboard input
    %   OBJ = ExampleHelperTurtleBotKeyInput() creates a figure with instructions for Turtlebot
    %   keyboard control. The object keeps track of the figure and axes handles
    %   and converts keyboard input into ASCII values.
    %
    %   ExampleHelperTurtleBotKeyInput methods:
    %       getKeystroke            - Returns ASCII value for keystroke
    %       closeFigure             - Closes the figure and cleans up
    %
    %   ExampleHelperTurtleBotKeyInput properties:
    %       Figure                  - Stores the figure handle
    %       Axes                    - Stores the axes handle
    %
    %   See also exampleHelperTurtleBotKeyboardControl
    
    %   Copyright 2014-2015 The MathWorks, Inc.
    
    properties
        Figure = [];            % Stores the figure handle
        Axes = [];              % Stores the axes handle
    end
    
    methods
        function obj = ExampleHelperTurtleBotKeyInput_EK(handles, plotobj)
            %ExampleHelperTurtleBotKeyInput - Constructor for KeyInput class
            
            callstr = 'set(gcbf,''Userdata'',double(get(gcbf,''Currentcharacter''))) ; uiresume ' ;
            
            obj.Figure = figure(...
                'Name','Press a key', ...
                'KeyPressFcn',callstr, ...
                'Position',[500 500  500 300],...
                'UserData','Timeout');
            obj.Axes = axes('Color','k','Visible','Off','XLim',[0,100],'YLim',[0,100]);
            text(50,80,'i = Forward','HorizontalAlignment','center','EdgeColor','k');
            text(50,40,'k = Backward','HorizontalAlignment','center','EdgeColor','k');
            text(25,60,'j = Left','HorizontalAlignment','center','EdgeColor','k');
            text(75,60,'l = Right','HorizontalAlignment','center','EdgeColor','k');
            text(50,20,'q = Quit','HorizontalAlignment','center','EdgeColor','r');
            %text(25,80,'g = Goto Target','HorizontalAlignment','center','EdgeColor','r');
            %text(75,80,'h = Halt Mission','HorizontalAlignment','center','EdgeColor','r');
            text(50,0,'Keep this figure in scope to give commands','HorizontalAlignment','center');
            
            followButton = uicontrol(obj.Figure,'Style','pushbutton',...
                'String','Start Follow','Position',[10 200 100 50],...
                'UserData', handles,...
                'Callback', @pushbuttonCallback_startFollow);
            exploreButton = uicontrol(obj.Figure,'Style','pushbutton', 'String','Start Explore','Position',[10 150 100 50], 'Callback', @pushbuttonCallback_startExplore);
            haltButton = uicontrol(obj.Figure,'Style','pushbutton', 'String','Halt Robot','Position',[10 100 100 50], 'Callback', @pushbuttonCallback_haltRobot);
            
            function pushbuttonCallback_startFollow(hObject, eventdata)
                global halt;
                pushbuttonCallback_haltRobot(0, 0);
                pause (1);
                halt = false;
                handles = hObject.UserData;
                %count = 0;
                %while (halt == false)
                %   count = count + 1
                %   pause (0.5)
                %end
                
                
                gazebo = ExampleHelperGazeboCommunicator();
                kobuki = ExampleHelperGazeboSpawnedModel('mobile_base',gazebo);
                targetSphere = ExampleHelperGazeboSpawnedModel('unit_sphere_1', gazebo);
                
                
                % Get blue object location
                goal = getState(targetSphere);
                goal = [goal(1) goal(2)]
                goalGrid = world2grid (plotobj.map, goal + 10);
                
                % Get current location
                pose = readPose(handles.odomSub.LatestMessage);
                %plotPose(plotobj, pose);
                
                [position, orientation, velocity] = getState(kobuki);
                pose = [position(1) position(2) orientation(1)]
                poseXY = [pose(1), pose(2)];
                distanceToGoal = norm([pose(1) pose(2)] - goal);
                poseGrid = world2grid(plotobj.map, poseXY + 10);
                
              % TODO: Use PSO to generate path as a set of waypoints
                % Write current occupancy grid
                disp ('Saving occupancy grid..');
                writeOccupancyGrid(plotobj);
                disp ('Saved occupancy grid');
                
                % Send occupancy grid
                occFile = 'OccupancyMap.txt';
                HOSTNAME = '10.4.146.67';
                USERNAME = 'krell';
                PASSWORD = '--------------------------------------------';
                ssh2_conn = scp_simple_put(HOSTNAME, USERNAME, PASSWORD, occFile);
                
                %% PSO arguments
                
                worldWidth = plotobj.map.GridSize(1);
                worldHeight = plotobj.map.GridSize(2);
                startX = poseGrid(1);
                startY = poseGrid(2);
                targetX = goalGrid(1);
                targetY = goalGrid(2);
                numWaypoints = 5;
                
                PSO_commandString = join(['bash execPSO.sh',...
                                     occFile,...
                                     string(worldWidth),...
                                     string(worldHeight),...
                                     string(startX),...
                                     string(startY),...
                                     string(targetX),...
                                     string(targetY),...
                                     string(numWaypoints)]);
                PSO_commandString = char (PSO_commandString)
                
                command_output = ssh2_simple_command(HOSTNAME,USERNAME,PASSWORD,PSO_commandString);
                waypoints_grid = vec2mat (str2double (strsplit (char (string(command_output(1,1))), ",")), 2)
                waypoints = grid2world(plotobj.map, waypoints_grid)
                waypoints = waypoints - 10
                waypoints
                halt = false;
                
                [velPub,velMsg] = rospublisher('/mobile_base/commands/velocity');

                s = size (waypoints);
                numWaypoints = s(1);
                minDistToGoal = 0.5;

                pp = robotics.PurePursuit;
                pp.Waypoints = waypoints;
                pp.LookaheadDistance = 0.3;
                pp.DesiredLinearVelocity = 0.5



                global flag;
                while (distanceToGoal > minDistToGoal && halt == false)

                    if (flag == 0)
                        break;
                    end
                    pose = readPose(handles.odomSub.LatestMessage);
                    %plotPose(plotobj, pose);
                    [position, orientation, velocity] = getState(kobuki);
                    pose = [position(1) position(2) orientation(1)];
                    theta = deg2rad(pose(3));
                    [v, w, lookaheadPoint] = step(pp, [pose(1), pose(2), theta]);
                    lookaheadPoint;
                    [v w];

                    distanceToGoal = norm([pose(1) pose(2)] - goal);

                    %% CONTROL
                    % Package ROS message and send to the robot

                    if (distanceToGoal > minDistToGoal)
                        velMsg.Linear.X = v;
                        velMsg.Angular.Z = w;
                        send(velPub,velMsg);
                    end

                    % pause the loop to allow other callbacks to fire
                    pause(0.05);
                end
            end
                
            
            function pushbuttonCallback_startExplore(hObject, eventdata)
               global halt;
               pushbuttonCallback_haltRobot(0,0);
               pause(1);
               halt = false;
               count = 0;
               while (halt == false)
                  count = count + 2
                  pause (0.5);
               end
            end
            
            function pushbuttonCallback_haltRobot(hObject, eventdata)
                global halt;
                halt = true;
            end
            
        end
        
        function keyout = getKeystroke(obj)
            %GETKEYSTROKE - Returns ASCII value for keystroke
            
            try
                uiwait(obj.Figure);
                keyout = get(obj.Figure,'Userdata') ;
            catch
                keyout = 'q';
            end
        end
        
        function closeFigure(obj)
            %CLOSEFIGURE - Closes the figure and cleans up
            
            try
                figure(obj.Figure);
                close(obj.Figure);
            catch
            end
        end
        
        
        function startPath(hObject, eventdata)
        5 + 5
        while get(hObject, 'Value')
            disp(rand)
            pause (0.5)
        end
    %     % Get goal location 
    %     goal = [3, 3];
    %     
    %     % Use PSO to generate path as a set of waypoints
    %     waypoints = [1 0; 3 0; 3 2];
    %     
    %     halt = false;
    %     gazebo = ExampleHelperGazeboCommunicator();
    %     kobuki = ExampleHelperGazeboSpawnedModel('mobile_base',gazebo)
    %     [velPub,velMsg] = rospublisher('/mobile_base/commands/velocity');
    % 
    %     s = size (waypoints)
    %     numWaypoints = s(1)
    %     minDistToGoal = 0.5;
    %     
    %     pp = robotics.PurePursuit;
    %     pp.Waypoints = waypoints
    %     pp.DesiredLinearVelocity = 0.5;
    %     
    %     pose = readPose(handles.odomSub.LatestMessage);
    %     [position, orientation, velocity] = getState(kobuki);
    %     pose = [position(1) position(2) orientation(1)]
    %     distanceToGoal = norm([pose(1) pose(2)] - goal);
    %     
    %     global flag;
    %     while (distanceToGoal > minDistToGoal && halt == false)
    % 
    %         if (flag == 0)
    %             break;
    %         end
    %         pose = readPose(handles.odomSub.LatestMessage);
    %         [position, orientation, velocity] = getState(kobuki);
    %         pose = [position(1) position(2) orientation(1)]
    %         theta = deg2rad(pose(3));
    %         [v, w, lookaheadPoint] = step(pp, [pose(1), pose(2), theta]);
    %         lookaheadPoint
    %         [v w];
    % 
    %         distanceToGoal = norm([pose(1) pose(2)] - goal);
    % 
    %         %% CONTROL
    %         % Package ROS message and send to the robot
    % 
    %         if (distanceToGoal > minDistToGoal)
    %             velMsg.Linear.X = v;
    %             velMsg.Angular.Z = w;
    %             send(velPub,velMsg)
    %         end
    %         
    %         % pause the loop to allow other callbacks to fire
    %         pause(0.01);
    %         
    %     end
    end

     function stop_Callback(hObject, eventdata, handles)
        global flag;
        flag = false;
     end    

    end
    
    
end

function plotPose(obj,pose)
    %PLOTPOSE - Plots the current Turtlebot pose

    if ~ishandle(obj.FigureHandle)
        % Check that figure hasn't been closed
        error('Figure close request: Exiting');
    end

    % Delete current pose objects from plot
    delete(obj.PoseHandle);
    delete(obj.ArrowHandle);

    % Plot current pose and direction
    obj.PoseHandle = plot(obj.AxesHandle, pose(1),pose(2),'o','MarkerSize',5);
    obj.ArrowHandle = plot(obj.AxesHandle,[pose(1), pose(1) + 0.5*cos(pose(3))], ...
        [pose(2), pose(2) + 0.5*sin(pose(3))], ...
        '*','MarkerSize',2,'Color','r','LineStyle','-');

    % Keep a history of previous robot positions (fixed-size FIFO)
    poseHistoryHandle = plot(obj.AxesHandle,pose(1),pose(2),'*','Color','c','MarkerSize',2);
    addNewPoseHandle(obj, poseHistoryHandle);
end

function laserData = readLaserData(laserMsg)
%readPose Extract the laser data in Cartesian coordinates
laserData = readCartesian(laserMsg) * [0 1; -1 0];
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

