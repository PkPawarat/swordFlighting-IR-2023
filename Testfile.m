% Initialization and Setup
clf;
clc;
hold on;
axis equal;
camlight;

% Properties
swordStartLoc = cell(1,2);
swordfile = cell(1,2);
baseRobot = cell(1,2);

swordStartLoc{1} = [-0.4 -0.5 0.5];
swordStartLoc{2} = [-0.4 1.55 0.5];
swordfile{1} = "model3D/Lightsaber2.ply";
swordfile{2} = "model3D/Lightsaber2.ply";
baseRobot{1} = transl(0,0,0.5);
baseRobot{2} = transl(0,1,0.5);
steps = 50;

% setup each link
links = [];

% Setup robot pose
pose1{1} = transl([0 1 1]) * troty(0, 'deg');
pose1{2} = transl([0 0 1]) * trotz(0, 'deg');

% Setup Swords
swords = cell(1,2);
sword_vertices = cell(1,2);
[swords{1}, sword_vertices{1}, swords{2}, sword_vertices{2}] = setupSwords(swordfile{1}, swordfile{2}, swordStartLoc{1}, swordStartLoc{2});
%% ****************************** Prepare to fight pose need to be hard code but it need to be adjust later on *********************************************
Preparepose = cell(1,2);
Preparepose{1} = [-0.4 0 0 0 0 0 -pi/2];
Preparepose{2} = [-pi/2 pi/2 -0 pi/3 0 0.5];






% Setup Robots
robot = cell(1,2);
robot{1} = SetupRobot(baseRobot{1}, false);
robot{2} = SetupRobot(baseRobot{2}, true);

% Setup Ellipsoid
robot = SetupEllipsoid(robot);

links{1} = UpdateEachLink(robot{1});
links{2} = UpdateEachLink(robot{2});
% Setup Environment
ObjectInTheScene = setupEnvironment();


pose = cell(1,2);
pose{1} = robot{1}.model.fkine(robot{1}.homeQ).T;
pose{2} = robot{2}.model.fkine(robot{2}.homeQ).T;



%% Interleave the operations of robot1 and robot2
    pickupSwordsStep(robot, swordStartLoc, swords, sword_vertices, steps);
   %%
    PreparePoses(robot, Preparepose, swords, sword_vertices, steps);
    %%
    collision = MoveRobotToLocation(robot, pose1, swords, sword_vertices, steps);
    if collision
        str = input('Type "y" to move each robot back to original\n Or "n" to cancle program: ', 's');
        switch str
            case 'y'
                % Code to move each robot back to its original position
                disp('Moving robots back to original position...');
                home = {robot{1}.model.fkine(robot{1}.homeQ), robot{2}.model.fkine(robot{2}.homeQ)};
                MoveRobotToLocation(robot, home, swords, sword_vertices, steps, false);
            case 'n'
                disp('Cancelling program...');
                return; % This will exit the script or function
            otherwise
                disp('Invalid input. Please type "y" or "n".');
        end
    end
%% check collistion 
robot{1}.model.animate([-0.4 0 0 0 0 0 -pi/2])
robot{2}.model.animate([-pi/2 pi/2 -0 pi/3 0 0.5 ])         
poses1 = GetLinkPoses(robot{1}.model.getpos, robot{1}.model);
poses2 = GetLinkPoses(robot{2}.model.getpos, robot{2}.model);
% collisionDetected = CheckCollision2(robot{1}, robot{2}, poses1, poses2);
% robot{1}.model.animate(robot{1}.model.ikcon(pose1{1}))
% robot{2}.model.animate(robot{2}.model.ikcon(pose1{2}))

%%
collisionDetected = CheckCollision(robot{1}, robot{2});

if collisionDetected
    disp('Collision detected!');
else
    disp('No collision detected.');
end

%%
% workspace_limits = [-2 2 -1 2 0 2]; % Example limits, adjust as needed
% robot{1}.model.plot3d(robot{1}.model.getpos, 'workspace', workspace_limits);
% robot{2}.model.plot3d(robot{2}.model.getpos, 'workspace', workspace_limits);

%% Setup Robot parts and robot base
function robot = SetupRobot(base, newRobot)
    if newRobot
        robot = FanucM20(base); 
        a=0;
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
function ObjectInTheScene = setupEnvironment()
    hold on;
    axis equal
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
function [sword1, sword1_vertices, sword2, sword2_vertices] = setupSwords(swordfile1, swordfile2, sword1StartLoc, sword2StartLoc)
    sword1 = PlaceObject(swordfile1);
    sword2 = PlaceObject(swordfile2);
    sword1_vertices = get(sword1, 'Vertices');
    sword2_vertices = get(sword2, 'Vertices');
    RotateObject(sword1, (transl(sword1StartLoc) * trotx(90, 'deg')));
    RotateObject(sword2, (transl(sword2StartLoc) * trotx(90, 'deg')));
end

%% This function is initial function for the robot to pickup swords and return the pose to prepare pose
function collision = pickupSwordsStep(robots, locations, swords, swordVertices, steps)
    
    collision = MoveRobotToLocation(robots, locations, [], swordVertices, steps);

    for i = 1:length(robots)
        robot = robots{i};
        sword = swords{i};
        vertices = swordVertices{i};
        MoveSwords(robot, sword, vertices);
        pause(0.001);
    end
end

function collision = PreparePoses(robots, locations, swords, swordVertices, steps)
    
    collision = MoveRobotToLocation(robots, locations, swords, swordVertices, steps);

    for i = 1:length(robots)
        robot = robots{i};
        sword = swords{i};
        vertices = swordVertices{i};
        MoveSwords(robot, sword, vertices);
        pause(0.001);
    end
end

%% Move the robot base on end-effector from 'location', update swords location and checking collision between robot parts itself.
function collision = MoveRobotToLocation(robots, locations, swords, swordVertices, steps, PassCollision)
    
    if nargin < 6			
	    PassCollision = true;				
    end

    qMatrix = cell(1, length(robots));
    
    for i = 1:length(robots)
        robot = robots{i};
        location = locations{i};
        qMatrix{i} = CalculateQmatrix(robot, location, steps);
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
            MoveSwords(robot, sword, vertices);
            pause(0.001);
        end
        collision = CheckCollision(robots{1}, robots{2});
        if collision && PassCollision
            % if the robot part is collition stop the robot immediately
            disp('STOP All systems, the robot parts have collide.')
            return;
        end
    end
end

function qMatrix = CalculateQmatrix(robot, location, steps)
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

function MoveSwords(robot, sword, swordVertices)
    if isempty(sword) 
        return;  
    end
    robotPose = robot.model.fkine(robot.model.getpos).T * trotz(90, 'deg');
    transformedVertices = [swordVertices, ones(size(swordVertices,1),1)] * robotPose';
    set(sword,'Vertices',transformedVertices(:,1:3));
end

function RotateObject(object, transformMatrix)
    vertices = get(object, 'Vertices');
    transformedVertices = (transformMatrix * [vertices, ones(size(vertices, 1), 1)]')';
    set(object, 'Vertices', transformedVertices(:, 1:3));
end

%% UpdateEachLink
function tr = UpdateEachLink(robot)
    q = zeros(1,robot.model.n); 
    tr = zeros(4,4,robot.model.n+1);
    tr(:,:,1) = robot.model.base;
    L = robot.model.links;
    for i = 1 : robot.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end
end

%% Setup ellipsoid
function robots = SetupEllipsoid(robots)

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
    % workspace_limits = [-2 3 -1 2 -0 3]; % Example limits, adjust as needed
    % robots{1}.model.plot3d(robots{1}.model.getpos, 'workspace', workspace_limits);
    % robots{2}.model.plot3d(robots{2}.model.getpos, 'workspace', workspace_limits);
    % axis(workspace_limits);
    % view(3)
    axis equal
end


function points = UpdateEllipsoid(robot, tr)
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
        
        % Visualize the ellipsoid
        % surf(reshape(translatedPoints(1,:), size(X)), ...
        %      reshape(translatedPoints(2,:), size(Y)), ...
        %      reshape(translatedPoints(3,:), size(Z)));
    end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)
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

function collision = CheckCollision(robot1, robot2)
    collision = false;
    poses1 = GetLinkPoses(robot1.model.getpos, robot1.model);
    poses2 = GetLinkPoses(robot2.model.getpos, robot2.model);
    
    point1 = UpdateEllipsoid(robot1, poses1);
    point2 = UpdateEllipsoid(robot2, poses2);

    % check collision between two robot
    for i = 2: size(poses1,3)
        for k = 2: size(point2,2)
            distanceFromXYZ = robot1.model.links(i-1).a';
            radii = [distanceFromXYZ/1.5, 0.3, 0.3];

            cubePointsAndOnes = [inv(poses1(:,:,i)) * [point2{k},ones(size(point2{k},1),1)]']';
            updatedCubePoints = cubePointsAndOnes(:,1:3);
            base = robot1.model.base.T;
            algebraicDist = GetAlgebraicDist(updatedCubePoints, base(1:3,4)', radii);
            pointsInside = find(algebraicDist < 1);
            if length(pointsInside) > 0
                disp(['Base on robot1 There are ', num2str(size(pointsInside,1)),' tr inside the ',num2str(i), ' points inside the ',num2str(k), ' th ellipsoid']);
                collision = true;
                return;
            end
        end
    end

    % check collision between two robot
    for i = 2: size(poses2,3)
        for k = 2: size(point1,2)
            distanceFromXYZ = robot2.model.links(i-1).a';
            radii = [distanceFromXYZ/1.5, 0.3, 0.3];

            cubePointsAndOnes = [inv(poses2(:,:,i)) * [point1{k},ones(size(point1{k},1),1)]']';
            updatedCubePoints = cubePointsAndOnes(:,1:3);
            base = robot2.model.base.T;
            algebraicDist = GetAlgebraicDist(updatedCubePoints, base(1:3,4)', radii);
            pointsInside = find(algebraicDist < 1);
            if length(pointsInside) > 0
                disp(['Base on robot2 There are ', num2str(size(pointsInside,1)),' tr inside the ',num2str(i), ' points inside the ',num2str(k), ' th ellipsoid']);
                collision = true;
                return;
            end
        end
    end
end


% function collision = CheckCollision(robot1, robot2)
%     % Get link poses for both robots
%     poses1 = GetLinkPoses(robot1.model.getpos, robot1.model);
%     poses2 = GetLinkPoses(robot2.model.getpos, robot2.model);
% 
%     % Check for collisions between the links
%     for i = 2:size(poses1, 3)
%         distanceFromXYZ = sqrt(robot1.model.links(i-1).a^2 + robot1.model.links(i-1).d^2);
%         radii = [distanceFromXYZ/1.2, 0.1, 0.1];  % Adjust as needed
%         center = poses1(1:3, 4, i)';  % Use the transformation matrix to get the center
% 
%         for k = 2:size(robot2.model.points, 2)
%             PointsAndOnes = [inv(poses1(:,:,i)) * [robot2.model.points{k},ones(size(robot2.model.points{k},1),1)]']';
%             updatedPoints = PointsAndOnes(:,1:3);
%             algebraicDist = GetAlgebraicDist(updatedPoints, center, radii);
%             pointsInside = find(algebraicDist < 1);
%             if (length(pointsInside) > 1)
%                 disp(['Collision detected: ', num2str(size(pointsInside,1)),' points inside']);
%                 collision = true;
%                 return;
%             end
%         end
%     end
%     collision = false;
% end
% function collision = CheckCollision2(robot1, robot2)
%     % Get link poses for both robots
%     poses1 = GetLinkPoses(robot1.model.getpos, robot1.model);
%     poses2 = GetLinkPoses(robot2.model.getpos, robot2.model);
% 
%     % Check for collisions between the links
%     for i = 2:size(poses1, 3)
%         distanceFromXYZ = sqrt(robot1.model.links(i-1).a^2 + robot1.model.links(i-1).d^2);
%         radii = [distanceFromXYZ/1.5, 0.1, 0.1];  % Adjust as needed
%         center = poses1(1:3, 4, i)';  % Use the transformation matrix to get the center
% 
%         for k = 2:size(poses2, 3)
%             % Transform the points of robot2 using poses2
%             PointsAndOnes = (poses2(:,:,k) * [robot2.model.points{k-1}, ones(size(robot2.model.points{k-1},1),1)]')';
%             updatedPoints = PointsAndOnes(:,1:3);
% 
%             algebraicDist = GetAlgebraicDist(updatedPoints, center, radii);
%             pointsInside = find(algebraicDist < 1, 1);
%             if ~isempty(pointsInside)
%                 disp(['Collision detected between link ', num2str(i), ' of robot1 and link ', num2str(k), ' of robot2']);
%                 collision = true;
%                 return;
%             end
%         end
%     end
%     collision = false;
% end

%% function 
function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
    try
        algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                      + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                      + ((points(:,3)-centerPoint(3))/radii(3)).^2;
    catch
        return
    end
end
