classdef ExampleHelperTurtleBotVisualizer_EK < handle
    %ExampleHelperTurtleBotVisualizer - Class for plotting TurtleBot and local environment
    %   OBJ = ExampleHelperTurtleBotVisualizer(RANGES) creates a plot of the TurtleBot's world and
    %   according to the input axis ranges. The object keeps track of goal
    %   points, pose history, obstacle location data, and handles to graphical
    %   objects.
    %
    %   ExampleHelperTurtleBotVisualizer methods:
    %      plotGoal                 - Plots the goal point on the graph
    %      plotPose                 - Plots the current Turtlebot pose
    %      plotData                 - Plots object location data and robot pose
    %
    %   ExampleHelperTurtleBotVisualizer properties:
    %      FigureHandle             - Handle to the object figure
    %      AxesHandle               - Handle to the object axes
    %      Goal                     - Matrix of goal points
    %      PoseHandle               - Handle to the current pose object
    %      ArrowHandle              - Handle to current direction object
    %      PoseHistory              - Complete pose history matrix
    %      DataHistory              - Data of all occupied points
    %
    %   See also exampleHelperTurtleBotKeyboardControl, exampleHelperTurtleBotObstacleTimer
    
    %   Copyright 2014-2015 The MathWorks, Inc.
    
    properties
        FigureHandle = [];      % Handle to the object figure
        AxesHandle = [];        % Handle to the object axes
        Goal = [];              % Matrix of goal points
        PoseHandle = [];        % Handle to the current pose object
        ArrowHandle = [];       % Handle to current direction object
        PoseHistory = [];       % History of all currently plotted robot poses
        DataHistory = [];       % History of all currently plotted laser scans
        
        map = [];               % Occupancy map
        FigureHandleOcc = [];
        AxesHandleOcc = [];
        MapHandleOcc = [];
        
    end
    
    properties (Constant, Access = private)
        %MaxPoseHistorySize The maximum number of poses that should be stored in PoseHistory
        %   PoseHistory is a FIFO and if it reaches MaxPoseHistorySize, the
        %   oldest element will be deleted.
        MaxPoseHistorySize = 10000   
        
        %MaxDataHistorySize The maximum number of laser readings that should be stored in DataHistory
        %   DataHistory is a FIFO and if it reaches MaxDataHistorySize, the
        %   oldest element will be deleted.
        MaxDataHistorySize = 150000   
    end
    
    methods (Access = public)
        function obj = ExampleHelperTurtleBotVisualizer_EK(ranges)
            %ExampleHelperTurtleBotVisualizer - Constructor sets all the figure properties
            obj.FigureHandle = figure('Name','Robot Position','CloseRequestFcn',@obj.closeFigure);
            obj.AxesHandle = axes('Parent',obj.FigureHandle,'XGrid','on','YGrid','on','XLimMode','manual','YLimMode','manual');
            axis(obj.AxesHandle,ranges);
            hold(obj.AxesHandle,'on');
            
            try
                var = load('OccupancyMat.mat');
                obj.map = var.var;
            catch
                obj.map = robotics.BinaryOccupancyGrid(20, 20, 10);
            end
            obj.FigureHandleOcc = figure('Name', 'Map');
            obj.AxesHandleOcc = axes('Parent', obj.FigureHandleOcc);
            obj.MapHandleOcc = show(obj.map, 'Parent', obj.AxesHandleOcc)
            title(obj.AxesHandleOcc, 'OccupancyGrid: Update 0');    
            hold(obj.AxesHandleOcc, 'on');
            
            
            
        end
        
        function plotGoal(obj,goal)
            %PLOTGOAL - Plots the goal point on the graph
            
            if ~ishandle(obj.FigureHandle)
                % Check that figure hasn't been closed
                error('Figure close request: Exiting');
            end
            
            plot(obj.AxesHandle,goal(1),goal(2),'d','Color','m','MarkerSize',6);
            obj.Goal = [obj.Goal;goal];
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
        
        function plotWaypoints(obj, waypoints)
             if ~ishandle(obj.FigureHandleOcc)
                % Check that figure hasn't been closed
                error('Figure close request: Exiting');
             end
             waypoints(:,1)
             waypoints(:,2)
             plot(obj.AxesHandleOcc, waypoints(:,2), waypoints(:,1), 'x', 'MarkerSize', 15);
            
        end
        
        function plotData(obj,pose,data)
            %PLOTDATA - Plots object location data and robot pose
            
            if ~ishandle(obj.FigureHandle)
                % Check that figure hasn't been closed
                error('Figure close request: Exiting');
            end
            
            % Plot pose
            plotPose(obj,pose);
            
            th = pose(3)-pi/2;
            % Compute the world-frame location of laser points
            dataWorld = data*[cos(th) sin(th);-sin(th) cos(th)] ...
                + repmat(pose(1:2),[numel(data(:,1)),1]);
            
            % Plot the transformed laser data on the world map
            % Also keep a history of previous laser data handles (fixed-size FIFO)
            laserDataHandle = plot(obj.AxesHandle,dataWorld(:,1), dataWorld(:,2), '*', 'MarkerSize',1,'Color','k');
            addNewLaserDataHandle(obj, laserDataHandle);
            
            % Update and show occupancy grid
            xy = dataWorld(:,1:2) + 10;
            size (xy);
            setOccupancy(obj.map, xy, 1);
            obj.MapHandleOcc.CData = occupancyMatrix(obj.map);
        end
        
        function writeOccupancyGrid(obj)
            mapInflated = copy (obj.map);
            inflate(mapInflated, 1, 'grid');
            O = zeros (mapInflated.GridSize(1), mapInflated.GridSize(2));
            ij = [ 1:mapInflated.GridSize(1) ; 1:mapInflated.GridSize(2) ]';
            %%%%getOccupancy(obj.map, ij, 'grid')
            
            % Print all values of occupancy grid
            for x = 1:mapInflated.GridSize(1)
                for y = 1:mapInflated.GridSize(2)
                    O(x,y) = getOccupancy(mapInflated, [x y], 'grid');
                end
            end
            O = round(O);
            dlmwrite('OccupancyMap.txt', O, '')
            
            var = obj.map;
            % Also write the actual variable object
            save ('OccupancyMat.mat', 'var');
            
        end
    end
    
    
    
    methods (Access = protected)
        function addNewPoseHandle(obj, poseHandle)
            %addNewPoseHandle Store a new pose graphics handle
            
            obj.PoseHistory = [obj.PoseHistory; poseHandle];
            if length(obj.PoseHistory) > obj.MaxPoseHistorySize
                % If we reached the maximum size of the array, delete the
                % oldest graphics handle.
                delete(obj.PoseHistory(1));
                obj.PoseHistory(1) = [];
            end
        end
        
        function addNewLaserDataHandle(obj, laserDataHandle)
            %addNewLaserDataHandle Store a new laser data graphics handle
            
            obj.DataHistory = [obj.DataHistory; laserDataHandle];
            if length(obj.DataHistory) > obj.MaxDataHistorySize
                % If we reached the maximum size of the array, delete the
                % oldest graphics handle.
                delete(obj.DataHistory(1));
                obj.DataHistory(1) = [];
            end
        end
        
        function closeFigure(obj,~,~)
            %CLOSEFIGURE - Callback function that deletes the figure
            % handle
            
            delete(obj.FigureHandle);
        end
    end
    
end

