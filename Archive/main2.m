classdef main2
    %MAIN Summary of this class goes here
    %   Detailed explanation goes here

    properties
        % Properties
        robots = cell(1,2);

        swordfile = {"model3D/Lightsaber2.ply", "model3D/Lightsaber2.ply"};
        baseRobot = {transl(0,0,0.5), transl(0,1,0.5)};

        swordStartLoc = {[-0.4 -0.5 0.5], [-0.4 1.55 0.5]};

        steps = 50;

        % setup each link
        links = [];

        % Setup robot pose
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
        % Preparepose{1} = [-0.4 0 0 0 0 0 -pi/2];
        % Preparepose{2} = [-pi/2 pi/2 -0 pi/3 0 0.5];

        % Object in the scence that are 3D models
        ObjectInTheScene=[];

        advance = false;            % set advance to true if want to turn on GUI
    end

    methods
        function self = main2()
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

            %% Execution 
            self.Execution();

        end

        function Execution(self)
            % pickup the sword
            self.pickupSwordsStep(self.robots, self.swordStartLoc, self.swords, self.sword_vertices, self.steps);
            % Prepare the to fight 
            self.PreparePoses(self.robots, self.Preparepose, self.swords, self.sword_vertices, self.steps);
            % Start random pose to fighting
            for i = 1:length(self.DemoPose)
                Pose = self.DemoPose{i};
                self.MoveRobotToLocation(self.robots, Pose, self.swords, self.sword_vertices, self.steps);
                self.PreparePoses(self.robots, self.Preparepose, self.swords, self.sword_vertices, self.steps);
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
            if newRobot
                scale = 0.3;
                % robot.model.plot(q, 'scale', scale);
                robot.model.animate(q);
            else
                robot.model.animate(q);
            end
        end

        %% Setup Environment of the scene 
        function Object = SetupEnvironment(self)
            hold on;

            Object{1} = PlaceObject('model3D/table_v1.ply', [-0.4,0,0]); % Assuming PlaceObject is a function or another script
            Object{2} = PlaceObject('model3D/table_v1.ply', [-0.4,1,0]);

            % ObjectInTheScene = cell(size(Object));
            % % Assign Vertices for object 
            % for i=1:length(Object)
            %     ObjectInTheScene{i} = get(Object{i}, 'Vertices');
            % end
            view(3);
            axis([-2 2 -2 3 0 4]);
            camlight;
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
            % sword2_vertices = get(sword2, 'Vertices');
            % sword_vertices = {sword1_vertices, sword2_vertices};
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
            self.MoveRobotToLocation(robots, locations, swords, swordVertices, steps, false);
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

                %% this may need to be in the CheckCollision ////////////////////////////////////////////////////////
                if PassCollision
                    [self.sword_vertices] = self.UpdateObjectsVertices(self.swords);
                    isColliding = self.checkObjectsCollision(self.sword_vertices{1}, self.sword_vertices{2});
                    if isColliding
                        collision = true;
                        disp('Swords collide')
                        return;
                    end

                    [collision,swordCollide] = self.CheckCollision(robots{1}, robots{2}, self.sword_vertices);
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

            % % Wait for 0.1 seconds after plotting all ellipsoids
            % pause(0.1);
            % 
            % % Delete all the surface plots
            % for i = 2:numLinks
            %     if isgraphics(h{i})
            %         delete(h{i});
            %     end
            % end
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
                    % distanceFromXYZ = robot1.model.links(i-1).a';
                    % radii = [distanceFromXYZ/1.5, 0.2, 0.2];
                    % 
                    % cubePointsAndOnes = [inv(poses1(:,:,i)) * [point2{k},ones(size(point2{k},1),1)]']';
                    % updatedCubePoints = cubePointsAndOnes(:,1:3);
                    % % base = robot1.model.base.T;
                    % base = poses1(:,:,2);
                    % algebraicDist = self.GetAlgebraicDist(updatedCubePoints, base(1:3,4)', radii);
                    % pointsInside = find(algebraicDist < 1);
                    % if length(pointsInside) > 5
                    %     disp(['Base on robot1 There are ', num2str(size(pointsInside,1)),' tr inside the ',num2str(i), ' points inside the ',num2str(k), ' th ellipsoid']);
                    %     collision = true;
                    %     return;
                    % end

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

        function CalculateAllPossiblePosition(robot)
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

        function DisplayAllPossiblePositionAndWorkspace(filename)
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
            display('Press enter to continue calculation of workspace')
            pause();
            % xlabel('X');
            % ylabel('Y');
            % zlabel('Z');
            % title('Collected Data Points');

            % Calculate workspace radius and volume
            min_x = min(data(:,1))
            max_x = max(data(:,1))
            min_y = min(data(:,2))
            max_y = max(data(:,2))
            min_z = min(data(:,3))
            max_z = max(data(:,3))

            % Calculate workspace radius
            workspace_radius = max(sqrt(max_x^2 + max_y^2 + max_z^2), sqrt(min_x^2 + min_y^2 + min_z^2));

            % Calculate workspace volume (using Monte Carlo sampling)
            num_samples = length(data);
            sample_points = [rand(num_samples, 1)*(max_x-min_x)+min_x, ...
                             rand(num_samples, 1)*(max_y-min_y)+min_y, ...
                             rand(num_samples, 1)*(max_z-min_z)+min_z];
            points_inside_workspace = sum(sample_points(:,1).^2 + sample_points(:,2).^2 + sample_points(:,3).^2 <= workspace_radius^2);
            workspace_volume = points_inside_workspace / num_samples * (max_x-min_x) * (max_y-min_y) * (max_z-min_z);

            % Calculate and display maximum reach
            max_reach = max(sqrt(max_x^2 + max_y^2 + max_z^2), sqrt(min_x^2 + min_y^2 + min_z^2));

            % Display minimum and maximum values for X, Y, and Z coordinates
            fprintf('Minimum X: %.4f\n m', min_x);
            fprintf('Maximum X: %.4f\n m', max_x);
            fprintf('Minimum Y: %.4f\n m', min_y);
            fprintf('Maximum Y: %.4f\n m', max_y);
            fprintf('Minimum Z: %.4f\n m', min_z);
            fprintf('Maximum Z: %.4f\n m', max_z);

            % Display workspace radius and volume
            fprintf('Workspace Radius: %.4f m\n ', workspace_radius);
            fprintf('Workspace Volume: %.4f m^3\n', workspace_volume);
            fprintf('Maximum Reach: %.4f\n m', max_reach);



            display('Press enter to delete this plot spaces')
            pause();
            try delete(check); end
        end





        % function self = SetupEnvironment(self)
        %     hold on;
        %     PlaceObject('table_v1.ply', [-0.4,0,0]);
        %     PlaceObject('table_v1.ply', [-0.4,1,0]);
        %     view(3);

        %     axis([-2 2 -2 2 0 3]);
        % end

        % function self = setupSwords(self)
        %     self.sword1 = self.DeleteObject(self.sword1);
        %     self.sword2 = self.DeleteObject(self.sword2);

        %     self.sword1 = PlaceObject(self.swordfile1);
        %     self.sword2 = PlaceObject(self.swordfile2);

        %     self.sword1_vertices = get(self.sword1, 'Vertices');
        %     self.sword2_vertices  = get(self.sword2, 'Vertices');
        %     % pose1 = self.robot1.model.fkine(self.robot1.model.getpos).T;
        %     self.RotateObject(self.sword1, (transl(self.sword1StartLoc) * trotx(90, 'deg') ));
        %     self.RotateObject(self.sword2, (transl(self.sword2StartLoc) * trotx(90, 'deg') ));

        %     % self.sword1_vertices = get(self.sword1, 'Vertices');
        %     % self.sword2_vertices  = get(self.sword2, 'Vertices');
        %     % [minXYZ, maxXYZ] = getObjectMinMaxVertices(self, self.sword1);
        % end

        % function self = pickupSwords(self, robot, location, object, objectVertic)
        %     if length(location) == 3
        %         location = transl(location) * troty(90, 'deg') * trotx(90, 'deg');
        %     else
        %         % location = location;
        %     end
        %     % location = transl(location) * troty(90, 'deg') * trotx(90, 'deg');
        %     self.MoveRobotToLocation(robot, location, object, objectVertic);
        % end

        % function [self, sword]= MoveSwords(self, robot, sword, sword_vertices)
        %     if (isempty(sword)) 
        %         return;  
        %     end
        %     robotPose = robot.model.fkine(robot.model.getpos).T * trotz(90, 'deg');
        %     transformedVertices = [sword_vertices, ones(size(sword_vertices,1),1)] * robotPose';
        %     set(sword,'Vertices',transformedVertices(:,1:3));
        % end


        % function self = MoveRobotToLocation(self, robot, location, object, objectVertic)
        %     if length(location) == 3
        %         MoveToObject = transl(location) * troty(-90,'deg');% * trotz(90,'deg');
        %     else
        %         MoveToObject = location;
        %     end
        %     initialGuess = robot.model.getpos;
        %     try 
        %         newQ1 = robot.model.ikine(MoveToObject, 'q0', initialGuess, 'forceSoln');
        %     catch 
        %         newQ1 = robot.model.ikcon(MoveToObject, initialGuess);
        %     end
        %     if isempty(newQ1)
        %         newQ1 = robot.model.ikcon(MoveToObject, 'q0', initialGuess);
        %     end 
        %     qMatrix = jtraj(initialGuess,newQ1,self.steps);
        %     for i = 1:length(qMatrix)
        %         try 
        %             robot.model.animate(qMatrix(i,:)); 
        %         catch 
        %             disp('Unable to move in this matrix : ')
        %             display(qMatrix(i,:))
        %         end
        %         [self, object] = self.MoveSwords(robot, object, objectVertic)
        %         pause(0.001);

        %     end
        % end

        % function self = DeleteObject(self, object)
        %     try 
        %         delete(object); 
        %     end
        % end
        % function RotateObject(self, object, transformMatrix)
        %     % Get the object's vertices
        %     vertices = get(object, 'Vertices');
        %     % Apply the rotation transformation to the object's vertices
        %     transformedVertices = (transformMatrix * [vertices, ones(size(vertices, 1), 1)]')';
        %     % Update the object's vertices with the rotated vertices
        %     set(object, 'Vertices', transformedVertices(:, 1:3));
        % end

        % function MoveObject(object, rotationMatrix, rotationCenter)
        %     % Get the object's vertices
        %     vertices = get(object, 'Vertices');

        %     % Translate vertices to origin based on rotation center
        %     vertices = vertices - repmat(rotationCenter, size(vertices, 1), 1);

        %     % Rotate the vertices
        %     rotatedVertices = (rotationMatrix * vertices')';

        %     % Translate back to original location
        %     rotatedVertices = rotatedVertices + repmat(rotationCenter, size(vertices, 1), 1);

        %     % Update the object's vertices
        %     set(object, 'Vertices', rotatedVertices);
        % end


        % function [minXYZ, maxXYZ] = getObjectMinMaxVertices(self, object)
        %     % Extracting the vertices of the object
        %     vertices = get(object, 'Vertices');

        %     % Finding the min and max points along each axis
        %     minX = min(vertices(:, 1));
        %     minY = min(vertices(:, 2));
        %     minZ = min(vertices(:, 3));

        %     maxX = max(vertices(:, 1));
        %     maxY = max(vertices(:, 2));
        %     maxZ = max(vertices(:, 3));

        %     minXYZ = [minX, minY, minZ];
        %     maxXYZ = [maxX, maxY, maxZ];

        %     % Displaying the results
        %     % fprintf('Min XYZ: [%f, %f, %f]\n', minXYZ);
        %     % fprintf('Max XYZ: [%f, %f, %f]\n', maxXYZ);
        % end


    end
end




% TODO 
% 
% add gripper     // lauren
% add sword       // lauren
% add collision   // Pk
% add avoidance collision // pk 
% 
% hard code for ways point of the robot aims //Pk
% 
% 
% 
% configuration for gui  // lauren
% 
% code pseudo
% 
%     Start environment 
%     show the life of the robot 
%     pickup sword 
%     start flighting until no more life left

% in demonstration 