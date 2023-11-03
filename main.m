classdef main
    %MAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Properties
        robots = cell(1,2);
        grip_ = cell(1,2);
        grip_1;
        grip_2;
        
        swordfile = {"model3D/Lightsaber2.ply", "model3D/Lightsaber2.ply"};
        baseRobot = {transl(0,0,0.5), transl(0,1,0.5)};
        
        swordStartLoc = {[-0.4 -0.5 0.5], [-0.4 1.55 0.5]};
        
        steps = 20;

        % setup each link
        links = [];

        % Setup robot pose
        PickupPose = {[-0.4590 0 0.8295 -1.9506 -0.1776 0 -pi/1.5], [1.9220 2.1520 2.4021 0.0187 0.0187 -3.4]}
        pose = cell(1,2);
        pose1 = {transl([0 1 1]) * troty(0, 'deg'), transl([0 0 1]) * trotz(0, 'deg')};
        pose2 = cell(1,2);

        % Setup Swords
        swords = cell(1,2);
        sword_vertices = cell(1,2);
        
        
        %% ****************************** Prepare to fight pose need to be hard code but it need to be adjust later on *********************************************
        Preparepose = {[-0.4 0 0 0 0 0 -pi/2], [-pi/2 pi/2 1.3 0 0 0.26]};
        Testose = {[-0.4 0 0 0 0 0 -0.5], deg2rad([-130 61 60 0 6 90])};

        DemoPose = {{[-0.4 0 0 0 0 0 -0.5], deg2rad([-130 61 60 0 6 90])}  
                    {[-0.4 -0.5 0 pi/3 1 0 0], [-pi/4 pi/2.5 pi/2.2 pi/2 0 0]}
                    {[-0.4559 -0.5775 -1.2918 -0.1777 2.6849 -1.5708 2.1482], [-1.5708 2.1906 2.2471 0.7330 0.8378 1.5708]}
                    {[0 0.2797 -0.3640 0 -1.1375 1.5761 -1.8507], [-2.3562 2.2048 2.1457 0.7330 0.8378 2.3562]}

                    }

        % Object in the scence that are 3D models
        ObjectInTheScene=[];

        gripper = zeros(2,3);
        Grip1;
        Grip2;
        GripDegree = 45;

        advance = false;            % set advance to true if want to turn on GUI
        TurnOnGripper = false;
    end
    
    methods
        function self = main()
            % Initialization and Setup
            clf;
            clc;
            hold on;
            axis([-3 3 -3 3 0 3]);
            camlight;
            
            %% Setup Swords
            [self.swords{1}, self.sword_vertices{1}, self.swords{2}, self.sword_vertices{2}] = self.setupSwords(self.swordfile{1}, self.swordfile{2}, self.swordStartLoc{1}, self.swordStartLoc{2});
            %% Setup Robots
            self.robots{1} = self.SetupRobot(self.baseRobot{1}, false);
            self.robots{2} = self.SetupRobot(self.baseRobot{2}, true);
            %% Setup Gripper
            if self.TurnOnGripper
                self.gripper(1,:) = self.CreateGrip(self.gripper(1,:),self.robots{1});
                self.gripper(2,:) = self.CreateGrip(self.gripper(2,:),self.robots{2});
            end
            %% Setup Ellipsoid
            self.robots = self.SetupEllipsoid(self.robots);            
            %% Setup Environment
            self.ObjectInTheScene = self.SetupEnvironment();
            [objectVertices] = self.UpdateObjectsVertices(self.ObjectInTheScene);
            %% plot bounding box
            % self.plotBoundingBox(objectVertices{1});
            % self.plotBoundingBox(objectVertices{2});
            %% Set home pose 
            self.pose = cell(1,2);
            self.pose{1} = self.robots{1}.model.fkine(self.robots{1}.homeQ).T;
            self.pose{2} = self.robots{2}.model.fkine(self.robots{2}.homeQ).T;
            
            % self.DisplayAllPossiblePositionAndWorkspace('logs file/FanucM20.txt');
            % self.DisplayAllPossiblePositionAndWorkspace('logs file/PositionUR5CanDo.txt');
            % 
            %% Execution 
            self.Execution();
            
        end

        function Execution(self)
            % pickup the sword
            
            [self.gripper,collision] = self.pickupSwordsStep(self.robots, self.PickupPose, self.swords, self.sword_vertices, self.steps);
            % Prepare the to fight 
            [self.gripper,collision] = self.MoveRobotToLocation(self.robots, self.Preparepose, self.swords, self.sword_vertices, self.steps, false);
            % Start random pose to fighting
            for i = 1:length(self.DemoPose)
                Pose = self.DemoPose{i};
                [self.gripper,collision] = self.MoveRobotToLocation(self.robots, Pose, self.swords, self.sword_vertices, self.steps);
                [self.gripper,collision] = self.MoveRobotToLocation(self.robots, self.Preparepose, self.swords, self.sword_vertices, self.steps, false);
            end
           
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
            robot.model.animate(q);
        end

        %% Setup Environment of the scene 
        function Object = SetupEnvironment(self)
            hold on;
            self.PlaceFloor(20, 'Images/floor.jpg');
            self.PlaceWall(0,6,4,15,'Images/KendoClub1.jpg', true);
            self.PlaceWall(6,0,4,15,'Images/KendoClub1.jpg', false);
            
            Object{1} = PlaceObject('model3D/table_v1.ply', [-0.4,0,0]); % Assuming PlaceObject is a function or another script
            Object{2} = PlaceObject('model3D/table_v1.ply', [-0.4,1,0]);
            PlaceObject('model3D/C3PO.ply', [4,-2,0]);
            PlaceObject('model3D/C3PO.ply', [4,-1,0]);
            PlaceObject('model3D/C3PO.ply', [4,0,0]);
            PlaceObject('model3D/C3PO.ply', [4,1,0]);
            PlaceObject('model3D/C3PO.ply', [4,2,0]);
            PlaceObject('model3D/Star_wars_JAWA.ply', [-2,5,0]);
            PlaceObject('model3D/Star_wars_JAWA.ply', [-1,5,0]);
            PlaceObject('model3D/Star_wars_JAWA.ply', [0,5,0]);
            PlaceObject('model3D/Star_wars_JAWA.ply', [1,5,0]);
            PlaceObject('model3D/Star_wars_JAWA.ply', [2,5,0]);
            PlaceObject('model3D/TABLER~1.PLY', [0,4.5,0]);
            PlaceObject('model3D/fireExtinguisher.ply', [1,4.5,0]);
            PlaceObject('model3D/emergencyStopButton.ply', [-0.2,4.5,0.25]);
            PlaceObject('model3D/Traffic Cone.ply', [-2,3.5,0]);
            PlaceObject('model3D/Traffic Cone.ply', [2,3.5,0]);
            PlaceObject('model3D/Traffic Cone.ply', [-2,-2,0]);
            PlaceObject('model3D/Traffic Cone.ply', [2,-2,0]);

            view(3);

            % Parameters for the box
            box_center = [0, 1, 0]; % The center of the box
            box_width = 4; % Width of the box
            box_height = 6; % Height of the box
            box_depth = 4; % Depth of the box
            
            % Create the box
            box_planes = self.createBox(box_center, box_width, box_height, box_depth);
            
            % Plot the planes of the box
            colors = ['r', 'g', 'b', 'y']; % Different color for each plane
            for i = 1:4
                surf(box_planes(i).x_grid, box_planes(i).y_grid, box_planes(i).z_grid, ...
                     'FaceColor', colors(i), 'FaceAlpha', 0.5);
                hold on;
            end



            axis([-3.5 5 -3.5 6 -0.1 4]);
            % axis([-3 3 -3 3 0.1 4]);
            camlight;
        end
        function PlaceFloor(self, imageSize, img)
            img = imread(img);
            img = im2double(img);
            % Define the size and position of the image
            imageSize = imageSize;  % Size of the image (5x5 units)
            [rows, cols] = size(img);
            scaleFactorX = imageSize / cols;
            scaleFactorY = imageSize / rows;
            
            % Create a scaled meshgrid
            [X, Y] = meshgrid(linspace(-imageSize/2, imageSize/2, cols), linspace(-imageSize/2, imageSize/2, rows));
            
            % Set Z to zero for a flat surface
            Z = zeros(size(X));
            
            % Plot the image
            surf(X, Y, Z, img, 'FaceColor', 'texturemap', 'EdgeColor', 'none');
        end
        function PlaceWall(self, x,y,height,wallWidth,img, rotate)
            % Read the image
            img = imread(img);
            img = imrotate(img, 180);
            img = im2double(img);  % Convert to double for texture mapping
            % Define the wall dimensions and position
            wallWidth = wallWidth;  % Width of the wall
            wallHeight = height; % Height of the wall
            wallPositionX = x; % X position of the wall
            wallPositionY = y; % Y position of the wall
            if rotate
                % Create a meshgrid for the wall
                [X, Z] = meshgrid(linspace(wallPositionX - wallWidth/2, wallPositionX + wallWidth/2, 100), linspace(0, wallHeight, 100));
                Y = wallPositionY * ones(size(X)); % Y is constant as the wall is vertical
            else
                % Create a meshgrid for the wall
                [Y, Z] = meshgrid(linspace(wallPositionY - wallWidth/2, wallPositionY + wallWidth/2, 100), linspace(0, wallHeight, 100));
                X = wallPositionX * ones(size(Y)); % Y is constant as the wall is vertical
            end
            % Plot the wall
            surf(X, Y, Z, img, 'FaceColor', 'texturemap', 'EdgeColor', 'none');  % Change 'blue' to any color you like
        end
        
        %% Plot bounding box
        function plotBoundingBox(self,vertices)
            minV = min(vertices);
            maxV = max(vertices);
            for i = 1:3
                range{i} = linspace(minV(i), maxV(i), 2);
            end
            [X, Y, Z] = meshgrid(range{:});
            X = X(:);
            Y = Y(:);
            Z = Z(:);
            plot3(X, Y, Z, 'ro'); % Plot corners of the bounding box
            hold on;
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
      
        %% Update Swords location 
        function [sword_vertices] = UpdateObjectsVertices(self, objects)
            sword_vertices = cell(1,length(objects));
            for i=1:length(objects)
                sword_vertices{i} = get(objects{i}, 'Vertices');
            end
        end
       
        %% Check collision of swords
        function isColliding = checkObjectsCollision(self, vertices1, vertices2)
            % Calculate the bounding box for each set of vertices
            min1 = min(vertices1);
            max1 = max(vertices1);
            min2 = min(vertices2);
            max2 = max(vertices2);
        
            % Check for overlap in each dimension
            overlapX = (min1(1) < max2(1)) && (max1(1) > min2(1));
            overlapY = (min1(2) < max2(2)) && (max1(2) > min2(2));
            overlapZ = (min1(3) < max2(3)) && (max1(3) > min2(3));
        
            % If there is overlap in all three dimensions, the objects are colliding
            isColliding = overlapX && overlapY && overlapZ;
        end
       
        %% This function is initial function for the robot to pickup swords and return the pose to prepare pose
        function [gripper,collision] = pickupSwordsStep(self, robots, locations, swords, swordVertices, steps)
            [gripper,collision] = self.MoveRobotToLocation(robots, locations, [], swordVertices, steps, true);
            if self.TurnOnGripper
                gripper(1,:) = self.GridGripping(gripper(1,:), robots{1}, true);
                gripper(2,:) = self.GridGripping(gripper(2,:), robots{2}, true);
            end
            for i = 1:length(robots)
                robot = robots{i};
                sword = swords{i};
                vertices = swordVertices{i};
                self.MoveSwords(robot, sword, vertices);
                pause(0.001);
            end
            % self.gripper = self.DeleteGrip(self.gripper);
            % gripper = self.gripper;
        end

        function [self,collision] = PreparePoses(self, robots, locations, swords, swordVertices, steps)
            [self.gripper,collision] = MoveRobotToLocation(robots, locations, swords, swordVertices, steps, false);
        end

        %% Move the robot base on end-effector from 'location', update swords location and checking collision between robot parts itself.
        function [gripper,collision] = MoveRobotToLocation(self, robots, locations, swords, swordVertices, steps, PassCollision)
            % GripOpen = false;
            collision = false;
            gripper = [];
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
                    try 
                        robot.model.animate(qMatrix{i}(s,:)); 
                    catch 
                        % robot.model.animate(qMatrix{i}(s,:) - 0.1); 
                        disp('Unable to move in this matrix : ');
                        display(qMatrix{i}(s,:));
                    end
                    vertices = swordVertices{i};
                    self.MoveSwords(robot, sword, vertices);
                    pause(0.001);
                end
                if self.TurnOnGripper
                    self.gripper(1,:) = self.SetupGrip(self.gripper(1,:),self.robots{1});
                    self.gripper(2,:) = self.SetupGrip(self.gripper(2,:),self.robots{2});
                    gripper = self.gripper;
                end
                %% this may need to be in the CheckCollision ////////////////////////////////////////////////////////
                if PassCollision
                    [sword_vertices] = self.UpdateObjectsVertices(self.swords);
                    isColliding = self.checkObjectsCollision(sword_vertices{1}, sword_vertices{2});
                    if isColliding
                        collision = true;
                        disp('Swords collide')
                        return;
                    end
    
                    [collision,swordCollide] = self.CheckCollision(robots{1}, robots{2}, sword_vertices);
                    if collision
                        % if the robot part is collition stop the robot immediately
                        disp('STOP All systems, the robot parts have collide.')
                        return;
                    elseif swordCollide
                        disp('Swords collide with robot')
                        return;
                    end
                end
                %% ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
                    newQ1 = [];
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
            % transformedVertices = [swordVertices, ones(size(swordVertices,1),1)] * robotPose';
            % set(sword,'Vertices',transformedVertices(:,1:3));
            
            % Create a homogeneous transformation matrix for rotation
            rotationMatrix = robotPose;            
            % Get the object's vertices
            vertices = get(sword, 'Vertices');
            % Apply the rotation transformation to the object's vertices
            transformedVertices = (rotationMatrix * [swordVertices, ones(size(swordVertices, 1), 1)]')';
            % Update the object's vertices with the rotated vertices
            set(sword, 'Vertices', transformedVertices(:, 1:3));
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


                    a = robot.model.links(k-1).a;
                    d = robot.model.links(k-1).d;
                    
                    % Compute the distance for ellipsoid size
                    distanceFromXYZ = sqrt(a^2 + d^2);
                    
                    % Handle the case where distanceFromXYZ is 0
                    if distanceFromXYZ == 0
                        distanceFromXYZ = 0.1; % Set a minimum value or adjust as needed
                    end
                    if a ~= 0
                        radii = [-distanceFromXYZ/2, 0.2, 0.2];
                        center = [-distanceFromXYZ/2, 0, 0];
                    else
                        radii = [0.2, -distanceFromXYZ/2, 0.2];
                        center = [0, -distanceFromXYZ/2, 0];
                    end

                    [X,Y,Z] = ellipsoid(center(1), center(2), center(3), radii(1), radii(2), radii(3));
                    robot.model.points{k} = [X(:),Y(:),Z(:)];
                    warning off
                    robot.model.faces{k} = delaunay(robot.model.points{k});
                    warning on;
                end
                robots{i} = robot;
            end
            % workspace_limits = [-2 3 -1 2 -0 3]; % Example limits, adjust as needed
            % robots{1}.model.plot3d(robots{1}.model.getpos, 'workspace', workspace_limits);
            % robots{2}.model.plot3d(robots{2}.model.getpos, 'workspace', workspace_limits);
            % axis(workspace_limits);
            % view(3)

        end

        function [points, radiis, ellipsoidCenters] = UpdateEllipsoid(self, robot, tr)
            links = robot.model.links;
            numLinks = robot.model.n + 1;
            points = cell(1, numLinks);
            radiis = cell(1, numLinks);
            ellipsoidCenters = cell(1, numLinks);
            sizeRadii = 0.05;
            h = cell(1, numLinks); % Cell array to store plot handles
        
            for i = 2:numLinks  % Start from the second link
                a = robot.model.links(i - 1).a;
                d = robot.model.links(i - 1).d;
        
                % Compute the distance for ellipsoid size
                distanceFromXYZ = sqrt(a^2 + d^2);
        
                % Handle the case where distanceFromXYZ is 0
                if distanceFromXYZ == 0
                    distanceFromXYZ = 0.1; % Set a minimum value or adjust as needed
                end
        
                if a ~= 0
                    radii = [-distanceFromXYZ / 2, sizeRadii, sizeRadii];
                    ellipsoidCenter = [-distanceFromXYZ / 2, 0, 0];
                else
                    radii = [sizeRadii, -distanceFromXYZ / 2, sizeRadii];
                    ellipsoidCenter = [0, -distanceFromXYZ / 2, 0];
                end        
                radiis{i} = radii;
                ellipsoidCenters{i} = ellipsoidCenter;
        
                % Extract the rotation matrix and translation vector for the current link
                rotation = tr(1:3, 1:3, i);
                translation = tr(1:3, 4, i);
        
                % Create the ellipsoid in its local frame
                [X, Y, Z] = ellipsoid(ellipsoidCenter(1), ellipsoidCenter(2), ellipsoidCenter(3), radii(1), radii(2), radii(3));
        
                % Rotate and translate the ellipsoid points
                rotatedPoints = rotation * [X(:)'; Y(:)'; Z(:)'];
                translatedPoints = bsxfun(@plus, rotatedPoints, translation);
        
                % Store the points
                points{i} = translatedPoints';
        
                % Visualize the ellipsoid
                % h{i} = surf(reshape(translatedPoints(1, :), size(X)), ...
                %             reshape(translatedPoints(2, :), size(Y)), ...
                %             reshape(translatedPoints(3, :), size(Z)));
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

        function [collision,swordCollide] = CheckCollision(self, robot1, robot2, sword_vertices)
            collision = false;
            swordCollide = false;
            poses1 = self.GetLinkPoses(robot1.model.getpos, robot1.model);
            poses2 = self.GetLinkPoses(robot2.model.getpos, robot2.model);
            
            [point1, radiis, ellipsoidCenters] = self.UpdateEllipsoid(robot1, poses1);
            [point2, radiis, ellipsoidCenters] = self.UpdateEllipsoid(robot2, poses2);
            
            sizeRadii=0.05;
            % check collision between two robot
            for i = 2: size(poses1,3)
                for k = 2: size(point2,2)
                    distanceFromXYZ = robot1.model.links(i-1).a';
                    radii = [distanceFromXYZ/1.5, sizeRadii, sizeRadii];
        
                    cubePointsAndOnes = [inv(poses1(:,:,i)) * [point2{k},ones(size(point2{k},1),1)]']';
                    updatedCubePoints = cubePointsAndOnes(:,1:3);
                    % base = robot1.model.base.T;
                    base = poses1(:,:,2);
                    algebraicDist = self.GetAlgebraicDist(updatedCubePoints, base(1:3,4)', radii);
                    pointsInside = find(algebraicDist < 1);
                    if length(pointsInside) > 5
                        disp(['Base on robot1 There are ', num2str(size(pointsInside,1)),' tr inside the ',num2str(i), ' points inside the ',num2str(k), ' th ellipsoid']);
                        collision = true;
                        return;
                    end

                    swordCollide = self.checkObjectsCollision(sword_vertices{1}, point2{k});
                    if swordCollide
                        disp('Sword 1 attacked robot 2')
                        return
                    end
                end
            end
            for i = 2: size(poses2,3)
                for k = 2: size(point1,2)
                    swordCollide = self.checkObjectsCollision(sword_vertices{2}, point1{k});
                    if swordCollide
                        disp('Sword 2 attacked robot 1')
                        return
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

        function CalculateAllPossiblePositionRobot1(self,robot)
            stepRads = deg2rad(30);
            qlim = robot.model.qlim;
            % Don't need to worry about joint 7
            pointCloudeSize = prod(floor((qlim(1:6,2)-qlim(1:6,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            tic

            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                % Don't need to worry about joint 7, just assume it=0
                                for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    q7 = 0;
                                    q = [q1,q2,q3,q4,q5,q6,q7];
                                    tr = robot.model.fkine(q);     
                                    translationVector = tr.t;
                                    pointCloud(counter,:) = translationVector;
                                    counter = counter + 1; 
                                    if mod(counter/pointCloudeSize * 100,1) == 0
                                        display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
            % 2.6 Create a 3D model showing where the end effector can be over all these samples.  
            hold on;
            plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            %% Store all possible position of the robot
            data = pointCloud;
            filename = 'PositionUR5CanDo.txt';  % Specify the file name
            % Call the function to write data to the file
            writeToTextFile(filename, data);

            %% function writeToTextFile
            function writeToTextFile(filename, data)
                % Check if the file exists
                if ~exist(filename, 'file')
                    % Create a new file if it doesn't exist
                    try
                        fileID = fopen(filename, 'w');
                        fclose(fileID);
                    catch
                        error('Failed to create the file.');
                    end
                end
            
                % Open the file for writing
                fileID = fopen(filename, 'w');
            
                % Write the data to the file
                fprintf(fileID, '%d %d %d\n', data');
            
                % Close the file
                fclose(fileID);
            end
            
        end

        function CalculateAllPossiblePositionRobot2(self,robot)
            stepRads = deg2rad(30);
            qlim = robot.model.qlim;
            % Don't need to worry about joint 7
            pointCloudeSize = prod(floor((qlim(1:6,2)-qlim(1:6,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            tic

            for q1 = qlim(1,1):stepRads:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                % Don't need to worry about joint 7, just assume it=0
                                q6 = 0;
                                q = [q1,q2,q3,q4,q5,q6];
                                tr = robot.model.fkine(q);     
                                translationVector = tr.t;
                                pointCloud(counter,:) = translationVector;
                                counter = counter + 1; 
                                if mod(counter/pointCloudeSize * 100,1) == 0
                                    display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                                end
                                
                            end
                        end
                    end
                end
            end
            
            % 2.6 Create a 3D model showing where the end effector can be over all these samples.  
            hold on;
            plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            %% Store all possible position of the robot
            data = pointCloud;
            filename = 'FanucM20.txt';  % Specify the file name
            % Call the function to write data to the file
            writeToTextFile(filename, data);

            %% function writeToTextFile
            function writeToTextFile(filename, data)
                % Check if the file exists
                if ~exist(filename, 'file')
                    % Create a new file if it doesn't exist
                    try
                        fileID = fopen(filename, 'w');
                        fclose(fileID);
                    catch
                        error('Failed to create the file.');
                    end
                end
            
                % Open the file for writing
                fileID = fopen(filename, 'w');
            
                % Write the data to the file
                fprintf(fileID, '%d %d %d\n', data');
            
                % Close the file
                fclose(fileID);
            end
            
        end

        function checkValueInFile(filename, targetValue)
            % Read data from the file
            try
                fileData = importdata(filename);  % Import data from the file
                data = fileData;  % Extract data from the imported data structure
            catch
                error('Failed to read data from the file.');
            end
        
            % Assuming you have defined targetValue and data as mentioned
            found = false;  % Initialize a flag to check if the value is found
            
            % Define the number of decimal places to round to
            decimalPlaces = 3;
            
            % Round the targetValue to the specified number of decimal places
            roundedTargetValue = round(targetValue, decimalPlaces);
            
            % Check if targetValue is present in data (rounded to the same decimal places)
            for i = 1:size(data, 1)
                roundedData = round(data(i, :), decimalPlaces);
                if isequal(roundedData, roundedTargetValue)
                    found = true;
                    break;  % Exit the loop if the value is found
                end
            end
            
            % Print a message based on whether the value is found
            if found
                fprintf('Value [%f %f %f] is present in the file.\n', targetValue);
            else
                fprintf('Value [%f %f %f] is not found in the file.\n', targetValue);
            end
        end

        function DisplayAllPossiblePositionAndWorkspace(self,filename)
            % Read data from the file
            try
                fileData = importdata(filename);  % Import data from the file
                data = fileData;  % Extract data from the imported data structure
            catch
                error('Failed to read data from the file.');
            end
        
            % Display collected data points
            % figure;
            check = plot3(data(:,1),data(:,2),data(:,3),'r.');
            display('Press enter to delete this plot spaces')
            pause();
            try delete(check); end
        end

        %% Gripper //////////////////////////////////////////
        %% Move end effector grippers
        function moveEndEffector(self, base_pos, grip_1, grip_2, end_effector_open)
            grip_1.base = base_pos*trotx(pi/2)*transl(0.035,0,0);
            grip_2.base = base_pos*trotx(pi/2)*transl(-0.035,0,0);
            % grip_1.base = grip_1_plot;
            % grip_2.base = grip_2_plot;

            if end_effector_open == true
                q = [145*pi/180];
            else
                q = [pi/2];
            end
         
            grip_1.animate(pi-q);                  % Plot the robot
            grip_2.animate(q);
        end
         
        %% Open end effector grippers
        function openEndEffector(self, grip_1, grip_2)
            for i = (pi/2):(pi/70):(145*pi/180)
                grip_1.animate(pi-i);                  % Plot the robot
                grip_2.animate(i);
            end
        end
         
        %% Close end effector grippers
        function closeEndEffector(self, grip_1, grip_2)
            for i = (145*pi/180):(-pi/70):(pi/2)
                grip_1.animate(pi-i);                  % Plot the robot
                grip_2.animate(i);
            end
        end
        
        function [grip_1, grip_2] = makeEndEffector(self, name)
            % Define DH parameters for gripper 1
            L1 = Link('d',0, 'a', 0.04, 'alpha', 0, 'qlim',[0 pi/2]);
            % % Define DH parameters for gripper 2
            L2 = Link('d',0, 'a', 0.04, 'alpha', 0, 'qlim',[pi/2 pi]);
            
            grip_1 = SerialLink([L1], 'name', 'TwoGripperEndEffector');
            grip_2 = SerialLink([L2], 'name', 'TwoGripperEnd');
            
            q = [pi/2];
         
            grip_1.plot(q);                  % Plot the robot
            grip_2.plot(q);
        end

        % Gripper //////////////////////////////////////////
        function gripper = CreateGrip(self, gripper,robot)
            pose = robot.model.fkine(robot.model.getpos).T;
            gripper(1) = self.DeleteObject(gripper(1));
            gripper(2) = self.DeleteObject(gripper(2));
            gripper(1) = PlaceObject('model3D/Newhand.ply');
            gripper(2) = PlaceObject('model3D/Newhand2.ply');
            gripper(3) = 45;
            % need to set z of end-effector to have offset in z about 0.0859
            self.RotateObjectgGripper(gripper(1), [0,0,0], pose * trotz(180, 'deg') * trotx(-gripper(3), 'deg'));
            self.RotateObjectgGripper(gripper(2), [0,0,0], pose * trotz(180, 'deg') * trotx(gripper(3), 'deg'));
        end
        function gripper = SetupGrip(self, gripper,robot)
            pose = robot.model.fkine(robot.model.getpos).T;
            gripper(1) = self.DeleteObject(gripper(1));
            gripper(2) = self.DeleteObject(gripper(2));
            gripper(1) = PlaceObject('model3D/Newhand.ply');
            gripper(2) = PlaceObject('model3D/Newhand2.ply');
            % need to set z of end-effector to have offset in z about 0.0859
            self.RotateObjectgGripper(gripper(1), [0,0,0], pose * trotz(180, 'deg') * trotx(-gripper(3), 'deg'));
            self.RotateObjectgGripper(gripper(2), [0,0,0], pose * trotz(180, 'deg') * trotx(gripper(3), 'deg'));
        end
        function gripper = DeleteGrip(self, gripper)
            gripper(1) = self.DeleteObject(gripper(1));
            gripper(2) = self.DeleteObject(gripper(2));
        end
        function obj = DeleteObject(self, obj)
            try
                delete(obj);
            end
        end

        function RotateObjectgGripper (self, gripper,location, degree)
            % Define the rotation angle in degrees (e.g., 180 degrees)
            rotationAngleDegrees = degree;
            % Create a homogeneous transformation matrix for rotation
            rotationMatrix = transl(location) * rotationAngleDegrees;            
            % Get the object's vertices
            vertices = get(gripper, 'Vertices');
            % Apply the rotation transformation to the object's vertices
            transformedVertices = (rotationMatrix * [vertices, ones(size(vertices, 1), 1)]')';
            % Update the object's vertices with the rotated vertices
            set(gripper, 'Vertices', transformedVertices(:, 1:3));
        end
       
        function gripper = GridGripping(self, gripper, robot, setGrip)   % true if grip, false ungrip
            for i = 1:30
                if setGrip
                    gripper(3) = gripper(3) - 1;
                else
                    gripper(3) = gripper(3) + 1;
                end
                pose = robot.model.fkine(robot.model.getpos).T;
                % Degree = Degree - 1;
                gripper(1) = self.DeleteObject(gripper(1));
                gripper(2) = self.DeleteObject(gripper(2));
                gripper(1) = PlaceObject('model3D/Newhand.ply');
                gripper(2) = PlaceObject('model3D/Newhand2.ply');
                self.RotateObjectgGripper(gripper(1), [0,0,0], pose * trotz(180, 'deg') * trotx(-gripper(3), 'deg'))
                self.RotateObjectgGripper(gripper(2), [0,0,0], pose * trotz(180, 'deg') * trotx(gripper(3), 'deg'))
                pause(0.001);
            end 
        end

        function [x_grid, y_grid, z_grid] = createPlane(self,point, width, height)
            % point: A 1x3 vector [x, y, z] representing a point on the plane
            % width: The width of the plane along the X-axis
            % height: The height of the plane along the Y-axis
            
            % Create a grid of points for the plane
            [y_grid, z_grid] = meshgrid(linspace(-height/2, height/2, 10), linspace(-width/2, width/2, 10));
            x_grid = point(1) * ones(size(y_grid)); % x is constant for a vertical plane
            
            % Adjust y and z to be centered around the provided point
            y_grid = y_grid + point(2);
            z_grid = z_grid + point(3);
        end
        function intersect_point = lineIntersection(self,obj_start, obj_direction, plane_x)
            % obj_start: A 1x3 vector [x, y, z] representing the starting point of the line
            % obj_direction: A 1x3 vector [dx, dy, dz] representing the direction of the line
            % plane_x: The x-coordinate of the vertical plane
            
            % Calculate the parameter t at which the line intersects the plane
            t_intersect = (plane_x - obj_start(1)) / obj_direction(1);
            
            % Calculate the intersection point
            intersect_point = obj_start + t_intersect * obj_direction;
        end
        % function [planes] = createBox(self,center, width, height, depth)
        %     % center: A 1x3 vector [x, y, z] representing the center of the box
        %     % width: The width of the box along the X-axis
        %     % height: The height of the box along the Y-axis
        %     % depth: The depth of the box along the Z-axis
        %     hold on;
        %     % Initialize the planes structure
        %     planes = struct('x_grid', [], 'y_grid', [], 'z_grid', []);
        % 
        %     % Half-dimensions
        %     halfWidth = width / 2;
        %     halfHeight = height / 2;
        %     halfDepth = depth / 2;
        % 
        %     % Define the grid for each plane
        %     [y_grid, z_grid] = meshgrid(linspace(-halfHeight, halfHeight, 10), linspace(-halfDepth, halfDepth, 10));
        % 
        %     % Front plane (x is constant)
        %     planes(1).x_grid = (center(1) + halfWidth) * ones(size(y_grid));
        %     planes(1).y_grid = y_grid + center(2);
        %     planes(1).z_grid = z_grid + center(3);
        % 
        %     % surf(planes(1).x_grid, planes(1).y_grid, planes(1).z_grid, ...
        %     %      'FaceColor', 'r', 'FaceAlpha', 0.5);
        % 
        %     % Back plane (x is constant)
        %     planes(2).x_grid = (center(1) - halfWidth) * ones(size(y_grid));
        %     planes(2).y_grid = y_grid + center(2);
        %     planes(2).z_grid = z_grid + center(3);
        % 
        %     % surf(planes(2).x_grid, planes(2).y_grid, planes(2).z_grid, ...
        %     %      'FaceColor', 'r', 'FaceAlpha', 0.5);
        % 
        %     % Left plane (y is constant)
        %     planes(3).y_grid = (center(2) - halfHeight) * ones(size(z_grid));
        %     planes(3).x_grid = z_grid + center(1); % z_grid reused for x since y is constant
        %     planes(3).z_grid = y_grid + center(3); % y_grid reused for z
        % 
        %     % surf(planes(3).x_grid, planes(3).y_grid, planes(3).z_grid, ...
        %     %      'FaceColor', 'r', 'FaceAlpha', 0.5);
        % 
        %     % Right plane (y is constant)
        %     planes(4).y_grid = (center(2) + halfHeight) * ones(size(z_grid));
        %     planes(4).x_grid = z_grid + center(1); % z_grid reused for x since y is constant
        %     planes(4).z_grid = y_grid + center(3); % y_grid reused for z
        % 
        %     % surf(planes(4).x_grid, planes(4).y_grid, planes(4).z_grid, ...
        %     %      'FaceColor', 'r', 'FaceAlpha', 0.5);
        % 
        % end
        function [planes] = createBox(self, center, width, height, depth)
            % center: A 1x3 vector [x, y, z] representing the center of the box
            % width: The width of the box along the X-axis
            % height: The height of the box along the Y-axis
            % depth: The depth of the box along the Z-axis
            
            % Initialize the planes array
            planes = struct('x_grid', {}, 'y_grid', {}, 'z_grid', {});
            
            % Half-dimensions
            halfWidth = width / 2;
            halfHeight = height / 2;
            halfDepth = depth / 2;
            
            % Define the grid for each plane
            [y_grid, z_grid] = meshgrid(linspace(-halfHeight, halfHeight, 10), linspace(-halfDepth, halfDepth, 10));
            [x_grid, z_grid_x] = meshgrid(linspace(-halfWidth, halfWidth, 10), linspace(-halfDepth, halfDepth, 10));
            
            % Front plane (x is constant)
            planes(end+1) = struct('x_grid', (center(1) + halfWidth) * ones(size(y_grid)), ...
                                    'y_grid', y_grid + center(2), ...
                                    'z_grid', z_grid + center(3));
            
            % Back plane (x is constant)
            planes(end+1) = struct('x_grid', (center(1) - halfWidth) * ones(size(y_grid)), ...
                                    'y_grid', y_grid + center(2), ...
                                    'z_grid', z_grid + center(3));
            
            % Left plane (y is constant)
            planes(end+1) = struct('x_grid', x_grid + center(1), ...
                                    'y_grid', (center(2) - halfHeight) * ones(size(x_grid)), ...
                                    'z_grid', z_grid_x + center(3));
            
            % Right plane (y is constant)
            planes(end+1) = struct('x_grid', x_grid + center(1), ...
                                    'y_grid', (center(2) + halfHeight) * ones(size(x_grid)), ...
                                    'z_grid', z_grid_x + center(3));
            
            % Top plane (z is constant)
            planes(end+1) = struct('x_grid', x_grid + center(1), ...
                                    'y_grid', y_grid + center(2), ...
                                    'z_grid', (center(3) + halfDepth) * ones(size(x_grid)));
            
            % Bottom plane (z is constant)
            planes(end+1) = struct('x_grid', x_grid + center(1), ...
                                    'y_grid', y_grid + center(2), ...
                                    'z_grid', (center(3) - halfDepth) * ones(size(x_grid)));
        end

        function isInside = checkInsideBox(self,point, box_center, width, height, depth)
            % Check if a point is inside the given box
            halfWidth = width / 2;
            halfHeight = height / 2;
            halfDepth = depth / 2;
            
            isInside = point(1) >= (box_center(1) - halfWidth) && ...
                       point(1) <= (box_center(1) + halfWidth) && ...
                       point(2) >= (box_center(2) - halfHeight) && ...
                       point(2) <= (box_center(2) + halfHeight) && ...
                       point(3) >= (box_center(3) - halfDepth) && ...
                       point(3) <= (box_center(3) + halfDepth);
        end



    end
end



