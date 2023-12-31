classdef swordFighting_app_test < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure              matlab.ui.Figure
        StopButton_2          matlab.ui.control.Button
        StopButton            matlab.ui.control.Button
        Player1WinsLabel      matlab.ui.control.Label
        FightButton           matlab.ui.control.Button
        Player1TextAreaLabel  matlab.ui.control.Label
        Player2TextAreaLabel  matlab.ui.control.Label
        Player2_pos3          matlab.ui.control.Button
        Player2_pos2          matlab.ui.control.Button
        Player2_pos1          matlab.ui.control.Button
        Player1_pos3          matlab.ui.control.Button
        Player1_pos2          matlab.ui.control.Button
        Player1_pos1          matlab.ui.control.Button
        UIAxes                matlab.ui.control.UIAxes
    end

    properties (Access = public)
        % Properties
        robots = cell(1,2);
        swordStartLoc = cell(1,2);
        swordfile = cell(1,2);
        baseRobot = cell(1,2);
        
        steps = 50;

        % setup each link
        links = [];

        % Setup robot pose
        pose = cell(1,2);
        pose1 = cell(1,2);
        pose2 = cell(1,2);

        % Setup Swords
        swords = cell(1,2);
        sword_vertices = cell(1,2);
        
        
        %% ****************************** Prepare to fight pose need to be hard code but it need to be adjust later on *********************************************
        Preparepose = cell(1,2);
        
        % Object in the scence that are 3D models
        ObjectInTheScene=[];
    end
    
    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = swordFighting_app

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
    
    methods (Access = private)
        
        function self = main()
            % Initialization and Setup
            clf;
            clc;
            hold on;
            axis equal;
            camlight;

            self.swordStartLoc{1} = [-0.4 -0.5 0.5];
            self.swordStartLoc{2} = [-0.4 1.55 0.5];
            self.swordfile{1} = "model3D/Lightsaber2.ply";
            self.swordfile{2} = "model3D/Lightsaber2.ply";
            self.baseRobot{1} = transl(0,0,0.5);
            self.baseRobot{2} = transl(0,1,0.5);

            self.pose1{1} = transl([0 1 1]) * troty(0, 'deg');
            self.pose1{2} = transl([0 0 1]) * trotz(0, 'deg');

            self.Preparepose{1} = [-0.4 0 0 0 0 0 -pi/2];
            self.Preparepose{2} = [-pi/2 pi/2 -0 pi/3 0 0.5];

            [self.swords{1}, self.sword_vertices{1}, self.swords{2}, self.sword_vertices{2}] = self.setupSwords(self.swordfile{1}, self.swordfile{2}, self.swordStartLoc{1}, self.swordStartLoc{2});

            % Setup Robots
            self.robots{1} = self.SetupRobot(self.baseRobot{1}, false);
            self.robots{2} = self.SetupRobot(self.baseRobot{2}, true);

            % Setup Ellipsoid
            self.robots = self.SetupEllipsoid(self.robots);

            self.links{1} = self.UpdateEachLink(self.robots{1});
            self.links{2} = self.UpdateEachLink(self.robots{2});
            % Setup Environment
            self.ObjectInTheScene = self.setupEnvironment();


            self.pose = cell(1,2);
            self.pose{1} = self.robots{1}.model.fkine(self.robots{1}.homeQ).T;
            self.pose{2} = self.robots{2}.model.fkine(self.robots{2}.homeQ).T;

            %% Interleave the operations of robot1 and robot2
            self.pickupSwordsStep(self.robots, self.swordStartLoc, self.swords, self.sword_vertices, self.steps);
            %%
            self.PreparePoses(self.robots, self.Preparepose, self.swords, self.sword_vertices, self.steps);
            %%
            collision = self.MoveRobotToLocation(self.robots, self.pose1, self.swords, self.sword_vertices, self.steps);
            if collision
                str = input('Type "y" to move each robot back to original\n Or "n" to cancle program: ', 's');
                switch str
                    case 'y'
                        % Code to move each robot back to its original position
                        disp('Moving robots back to original position...');
                        home = {self.robots{1}.model.fkine(self.robots{1}.homeQ), self.robots{2}.model.fkine(self.robots{2}.homeQ)};
                        self.MoveRobotToLocation(self.robots, home, self.swords, self.sword_vertices, self.steps, false);
                    case 'n'
                        disp('Cancelling program...');
                        return; % This will exit the script or function
                    otherwise
                        disp('Invalid input. Please type "y" or "n".');
                end
            end

            collisionDetected = self.CheckCollision(self.robots{1}, self.robots{2});

            if collisionDetected
                disp('Collision detected!');
            else
                disp('No collision detected.');
            end
        end
        function execution(self)
            % pickup the sword

            % prepare the to fight 

            % run in the loop 
                % start fighting the robot
                % check health of each robot 
                % show status health message
                
                % if health is == 0 then exit the loop and show vitory message

            % return the sword to original position
            % return to original position of the robot
        end

        %% Setup Robot parts and robot base
        function robot = SetupRobot(self, base, newRobot)
            if newRobot
                robot = FanucM20(base); 
            else
                robot = LinearUR5(base);
            end
            robot.model.delay = 0.01;
            q = zeros(1, length(robot.model.links));
            if newRobot
                scale = 0.3;
                % robot.model.plot(q, 'scale', scale);
                robot.model.animate(q);
            else
                robot.model.animate(q);
            end
        end

        %% Setup Environment of the scene 
        function ObjectInTheScene = setupEnvironment(self)
            hold on;
            axis equal;
            Object{1} = PlaceObject('table_v1.ply', [-0.4,0,0]); % Assuming PlaceObject is a function or another script
            Object{2} = PlaceObject('table_v1.ply', [-0.4,1,0]);

            ObjectInTheScene = cell(size(Object));
            % Assign Vertices for object 
            for i=1:size(Object)
                ObjectInTheScene{i} = get(Object{i}, 'Vertices');
            end
            view(3);
            axis([-2 2 -1 2 0 2.5]);
            camlight;
        end

        %% Setup Swords location base on hardcode location
        function [sword1, sword1_vertices, sword2, sword2_vertices] = setupSwords(self, swordfile1, swordfile2, sword1StartLoc, sword2StartLoc)
            sword1 = PlaceObject(swordfile1);
            sword2 = PlaceObject(swordfile2);
            sword1_vertices = get(sword1, 'Vertices');
            sword2_vertices = get(sword2, 'Vertices');
            self.RotateObject(sword1, (transl(sword1StartLoc) * trotx(90, 'deg')));
            self.RotateObject(sword2, (transl(sword2StartLoc) * trotx(90, 'deg')));
        end

        %% This function is initial function for the robot to pickup swords and return the pose to prepare pose
        function collision = pickupSwordsStep(self, robots, locations, swords, swordVertices, steps)
            collision = self.MoveRobotToLocation(robots, locations, [], swordVertices, steps);
            for i = 1:length(robots)
                robot = robots{i};
                sword = swords{i};
                vertices = swordVertices{i};
                self.MoveSwords(robot, sword, vertices);
                pause(0.001);
            end
        end

        function collision = PreparePoses(self, robots, locations, swords, swordVertices, steps)
            collision = self.MoveRobotToLocation(robots, locations, swords, swordVertices, steps);
            for i = 1:length(robots)
                robot = robots{i};
                sword = swords{i};
                vertices = swordVertices{i};
                self.MoveSwords(robot, sword, vertices);
                pause(0.001);
            end
        end

        %% Move the robot base on end-effector from 'location', update swords location and checking collision between robot parts itself.
        function collision = MoveRobotToLocation(self, robots, locations, swords, swordVertices, steps, PassCollision)
            
            if nargin < 7			
                PassCollision = true;				
            end

            qMatrix = cell(1, length(robots));
            
            for i = 1:length(robots)
                robot = robots{i};
                location = locations{i};
                qMatrix{i} = self.CalculateQmatrix(robot, location, steps);
            end
            
            for s = 1:steps
                for i = 1:length(robots)
                    robot = robots{i};
                    sword = [];
                    if ~isempty(swords)
                        sword = swords{i};
                    end
                    vertices = swordVertices{i};
                    
                    try 
                        robot.model.animate(qMatrix{i}(s,:)); 
                    catch 
                        % robot.model.animate(qMatrix{i}(s,:) - 0.1); 
                        disp('Unable to move in this matrix : ');
                        display(qMatrix{i}(s,:));
                    end
                    self.MoveSwords(robot, sword, vertices);
                    pause(0.001);
                end

                collision = self.CheckCollision(robots{1}, robots{2});
                if collision && PassCollision
                    % if the robot part is collition stop the robot immediately
                    disp('STOP All systems, the robot parts have collide.')
                    return;
                end
            end
        end

        function qMatrix = CalculateQmatrix(self, robot, location, steps)
            % Ensure output is initialized
            qMatrix = [];
            if length(robot.model.links) ~= length(location)
                if length(location) == 3
                    MoveToObject = transl(location) * troty(-90,'deg');
                else
                    MoveToObject = location;
                end
                
                initialGuess = robot.model.getpos;
                try 
                    newQ1 = robot.model.ikine(MoveToObject, 'q0', initialGuess, 'forceSoln');
                catch 
                    newQ1 = robot.model.ikcon(MoveToObject, initialGuess);
                end
                
                if isempty(newQ1)
                    newQ1 = robot.model.ikcon(MoveToObject, 'q0', initialGuess);
                end 
            else
                initialGuess = robot.model.getpos;
                newQ1 = location;
            end
            try 
                qMatrix = jtraj(initialGuess,newQ1,steps);
            end
        end

        
        function MoveSwords(self, robot, sword, swordVertices)
            if isempty(sword) 
                return;  
            end
            robotPose = robot.model.fkine(robot.model.getpos).T * trotz(90, 'deg');
            transformedVertices = [swordVertices, ones(size(swordVertices,1),1)] * robotPose';
            set(sword,'Vertices',transformedVertices(:,1:3));
        end

        function RotateObject(self, object, transformMatrix)
            vertices = get(object, 'Vertices');
            transformedVertices = (transformMatrix * [vertices, ones(size(vertices, 1), 1)]')';
            set(object, 'Vertices', transformedVertices(:, 1:3));
        end


        %% UpdateEachLink
        function tr = UpdateEachLink(self, robot)
            q = zeros(1,robot.model.n); 
            tr = zeros(4,4,robot.model.n+1);
            tr(:,:,1) = robot.model.base;
            L = robot.model.links;
            for i = 1 : robot.model.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
        end

        %% Setup ellipsoid
        function robots = SetupEllipsoid(self, robots)

            for i = 1:length(robots)
                robot = robots{i};
                
                numLinks = robot.model.n+1;
                for k = 2:numLinks  % Start from the second link
                    % Compute the distance between two consecutive transformation matrices
                    distanceFromXYZ = sqrt(robot.model.links(k-1).a^2 + robot.model.links(k-1).d^2);
                    
                    % Handle the case where distanceFromXYZ is 0
                    if distanceFromXYZ == 0
                        distanceFromXYZ = 0.1; % Set a minimum value or adjust as needed
                    end
                    
                    % Use the computed distance to determine the radii of the ellipsoid
                    radii = [distanceFromXYZ/1.2, 0.1, 0.1];  % Adjust as needed
                    center = [-distanceFromXYZ/2 0 0];
                    [X,Y,Z] = ellipsoid(center(1), center(2), center(3), radii(1), radii(2), radii(3));
                    robot.model.points{k} = [X(:),Y(:),Z(:)];
                    warning off
                    robot.model.faces{k} = delaunay(robot.model.points{k});
                    warning on;
                end
                robots{i} = robot;
            end
        end

        function points = UpdateEllipsoid(self, robot, tr)
            links = robot.model.links;
            numLinks = robot.model.n+1;
            points = cell(1,numLinks);
        
            for i = 2:numLinks  % Start from the second link
                a = robot.model.links(i-1).a;
                d = robot.model.links(i-1).d;
                
                % Compute the distance for ellipsoid size
                distanceFromXYZ = sqrt(a^2 + d^2);
                
                % Handle the case where distanceFromXYZ is 0
                if distanceFromXYZ == 0
                    distanceFromXYZ = 0.1; % Set a minimum value or adjust as needed
                end
                if a ~= 0
                    radii = [-distanceFromXYZ/2, 0.2, 0.2];
                    ellipsoidCenter = [-distanceFromXYZ/2, 0, 0];
                else
                    radii = [0.2, -distanceFromXYZ/2, 0.2];
                    ellipsoidCenter = [0, -distanceFromXYZ/2, 0];
                end        
                % Extract the rotation matrix and translation vector for the current link
                rotation = tr(1:3, 1:3, i);
                translation = tr(1:3, 4, i);
        
                % Create the ellipsoid in its local frame
                [X,Y,Z] = ellipsoid(ellipsoidCenter(1), ellipsoidCenter(2), ellipsoidCenter(3), radii(1), radii(2), radii(3));
                
                % Rotate the ellipsoid points
                rotatedPoints = rotation * [X(:)'; Y(:)'; Z(:)'];
                
                % Translate the rotated points
                translatedPoints = bsxfun(@plus, rotatedPoints, translation);
                
                % Store the points
                points{i} = translatedPoints';
            end
        end

        %% GetLinkPoses
        % q - robot joint angles
        % robot -  seriallink robot model
        % transforms - list of transforms
        function [ transforms ] = GetLinkPoses(self, q, robot)
            links = robot.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = robot.base;
            for i = 1:length(links)
                L = links(1,i);
                if L.jointtype == 'P' %Prismatic
                    transforms(:,:,i + 1) = transforms(:,:,i) * trotz(L.theta) * transl(0,0, q(i)) * transl(L.a,0,0) * trotx(L.alpha);
                else
                    % Use the theta value from the robot model
                    theta = L.theta;
                    if isnumeric(theta)
                        joint_angle = theta + q(i);
                    else
                        joint_angle = q(i); % If theta is symbolic (e.g., 'q1'), use the joint angle directly
                    end
                    transforms(:,:,i + 1) = transforms(:,:, i) * trotz(joint_angle + L.offset) * transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                end
                
            end
        end

        function collision = CheckCollision(self, robot1, robot2)
            collision = false;
            poses1 = self.GetLinkPoses(robot1.model.getpos, robot1.model);
            poses2 = self.GetLinkPoses(robot2.model.getpos, robot2.model);
            
            point1 = self.UpdateEllipsoid(robot1, poses1);
            point2 = self.UpdateEllipsoid(robot2, poses2);
        
            % check collision between two robot
            for i = 2: size(poses1,3)
                for k = 2: size(point2,2)
                    distanceFromXYZ = robot1.model.links(i-1).a';
                    radii = [distanceFromXYZ/1.5, 0.3, 0.3];
        
                    cubePointsAndOnes = [inv(poses1(:,:,i)) * [point2{k},ones(size(point2{k},1),1)]']';
                    updatedCubePoints = cubePointsAndOnes(:,1:3);
                    base = robot1.model.base.T;
                    algebraicDist = self.GetAlgebraicDist(updatedCubePoints, base(1:3,4)', radii);
                    pointsInside = find(algebraicDist < 1);
                    if length(pointsInside) > 5
                        disp(['Base on robot1 There are ', num2str(size(pointsInside,1)),' tr inside the ',num2str(i), ' points inside the ',num2str(k), ' th ellipsoid']);
                        collision = true;
                        return;
                    end
                end
            end
        end

        %% function 
        function algebraicDist = GetAlgebraicDist(self, points, centerPoint, radii)
            try
                algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                            + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                            + ((points(:,3)-centerPoint(3))/radii(3)).^2;
            catch
                return
            end
        end        
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: Player2_pos2
        function Player2_pos2Pushed(app, event)
            
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 684 516];
            app.UIFigure.Name = 'MATLAB App';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'Title')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.XTick = [0 0.2 0.4 0.6 0.8 3];
            app.UIAxes.XTickLabel = {'0'; '0.2'; '0.4'; '0.6'; '0.8'; '3'};
            app.UIAxes.YTick = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 3];
            app.UIAxes.YTickLabel = {'0'; '0.1'; '0.2'; '0.3'; '0.4'; '0.5'; '0.6'; '0.7'; '0.8'; '3'};
            app.UIAxes.ZTick = [0 0.5 3];
            app.UIAxes.HandleVisibility = 'callback';
            app.UIAxes.Position = [154 161 377 324];

            % Create Player1_pos1
            app.Player1_pos1 = uibutton(app.UIFigure, 'push');
            app.Player1_pos1.Position = [53 370 68 37];
            app.Player1_pos1.Text = 'Position 1';

            % Create Player1_pos2
            app.Player1_pos2 = uibutton(app.UIFigure, 'push');
            app.Player1_pos2.Position = [53 306 68 37];
            app.Player1_pos2.Text = 'Position 2';

            % Create Player1_pos3
            app.Player1_pos3 = uibutton(app.UIFigure, 'push');
            app.Player1_pos3.Position = [53 246 68 37];
            app.Player1_pos3.Text = 'Position 3';

            % Create Player2_pos1
            app.Player2_pos1 = uibutton(app.UIFigure, 'push');
            app.Player2_pos1.Position = [566 370 68 37];
            app.Player2_pos1.Text = 'Position 1';

            % Create Player2_pos2
            app.Player2_pos2 = uibutton(app.UIFigure, 'push');
            app.Player2_pos2.ButtonPushedFcn = createCallbackFcn(app, @Player2_pos2Pushed, true);
            app.Player2_pos2.Position = [566 306 68 37];
            app.Player2_pos2.Text = 'Position 2';

            % Create Player2_pos3
            app.Player2_pos3 = uibutton(app.UIFigure, 'push');
            app.Player2_pos3.Position = [566 246 68 37];
            app.Player2_pos3.Text = 'Position 3';

            % Create Player2TextAreaLabel
            app.Player2TextAreaLabel = uilabel(app.UIFigure);
            app.Player2TextAreaLabel.HorizontalAlignment = 'center';
            app.Player2TextAreaLabel.Position = [575 441 49 22];
            app.Player2TextAreaLabel.Text = 'Player 2';

            % Create Player1TextAreaLabel
            app.Player1TextAreaLabel = uilabel(app.UIFigure);
            app.Player1TextAreaLabel.HorizontalAlignment = 'center';
            app.Player1TextAreaLabel.Position = [62 441 49 22];
            app.Player1TextAreaLabel.Text = 'Player 1';

            % Create FightButton
            app.FightButton = uibutton(app.UIFigure, 'push');
            app.FightButton.Position = [312 127 94 24];
            app.FightButton.Text = 'Fight!';

            % Create Player1WinsLabel
            app.Player1WinsLabel = uilabel(app.UIFigure);
            app.Player1WinsLabel.HorizontalAlignment = 'center';
            app.Player1WinsLabel.Position = [302 89 113 22];
            app.Player1WinsLabel.Text = 'Player 1 Wins!';

            % Create StopButton
            app.StopButton = uibutton(app.UIFigure, 'push');
            app.StopButton.BackgroundColor = [1 0 0];
            app.StopButton.FontColor = [1 1 1];
            app.StopButton.Position = [30 75 113 53];
            app.StopButton.Text = 'Stop';

            % Create StopButton_2
            app.StopButton_2 = uibutton(app.UIFigure, 'push');
            app.StopButton_2.BackgroundColor = [1 0 0];
            app.StopButton_2.FontColor = [1 1 1];
            app.StopButton_2.Position = [543 75 113 53];
            app.StopButton_2.Text = 'Stop';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    
end