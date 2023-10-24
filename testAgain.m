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

% Setup Robots
robot = cell(1,2);
robot{1} = SetupRobot(baseRobot{1}, false);
robot{2} = SetupRobot(baseRobot{2}, true);

% Setup Ellipsoid
robot = SetupEllipsoid(robot);

links{1} = UpdateEachLink(robot{1});
links{2} = UpdateEachLink(robot{2});
% Setup Environment
setupEnvironment();

% Setup Swords
swords = cell(1,2);
sword_vertices = cell(1,2);
[swords{1}, sword_vertices{1}, swords{2}, sword_vertices{2}] = setupSwords(swordfile{1}, swordfile{2}, swordStartLoc{1}, swordStartLoc{2});

pose = cell(1,2);
pose{1} = robot{1}.model.fkine(robot{1}.homeQ).T;
% robot{2}.homeQ = zeros(1, length(robot{2}.model.links));
pose{2} = robot{2}.model.fkine(robot{2}.homeQ).T;

% Interleave the operations of robot1 and robot2
    % pickupSwordsStep(robot, swordStartLoc, swords, sword_vertices, steps);
%%
    MoveRobotToLocation(robot, pose1, swords, sword_vertices, steps)

%% check collistion 

poses1 = GetLinkPoses(robot{1}.model.getpos, robot{1}.model);
poses2 = GetLinkPoses(robot{2}.model.getpos, robot{2}.model);
collisionDetected = CheckCollision2(robot{1}, robot{2}, poses1, poses2);

if collisionDetected
    disp('Collision detected!');
else
    disp('No collision detected.');
end


%%
workspace_limits = [-2 3 -1 2 -0 3]; % Example limits, adjust as needed
robot{1}.model.plot3d(robot{1}.model.getpos, 'workspace', workspace_limits);
robot{2}.model.plot3d(robot{2}.model.getpos, 'workspace', workspace_limits);

%% Functions
function robot = SetupRobot(base, newRobot)
    if newRobot
        robot = LinearUR5(base); 
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

function setupEnvironment()
    hold on;
    axis equal
    PlaceObject('table_v1.ply', [-0.4,0,0]); % Assuming PlaceObject is a function or another script
    PlaceObject('table_v1.ply', [-0.4,1,0]);
    view(3);
    axis([-2 2 -1 2 0 2.5]);
    camlight;
end

function [sword1, sword1_vertices, sword2, sword2_vertices] = setupSwords(swordfile1, swordfile2, sword1StartLoc, sword2StartLoc)
    sword1 = PlaceObject(swordfile1);
    sword2 = PlaceObject(swordfile2);
    sword1_vertices = get(sword1, 'Vertices');
    sword2_vertices = get(sword2, 'Vertices');
    RotateObject(sword1, (transl(sword1StartLoc) * trotx(90, 'deg')));
    RotateObject(sword2, (transl(sword2StartLoc) * trotx(90, 'deg')));
end

function pickupSwordsStep(robots, locations, swords, swordVertices, steps)
    qMatrix = cell(1, length(robots));
    
    for i = 1:length(robots)
        robot = robots{i};
        location = locations{i};
        qMatrix{i} = CalculateQmatrix(robot, location, steps);
    end
    
    for s = 1:steps
        for i = 1:length(robots)
            robot = robots{i};
            vertices = swordVertices{i};
            try 
                robot.model.animate(qMatrix{i}(s,:)); 
            catch 
                disp('Unable to move in this matrix : ');
                display(qMatrix{i}(s,:));
            end
            MoveSwords(robot, [], vertices);
            pause(0.001);
        end
    end

    for i = 1:length(robots)
        robot = robots{i};
        sword = swords{i}
        vertices = swordVertices{i};
        % try 
        %     robot.model.animate(qMatrix{i}(s,:)); 
        % catch 
        %     disp('Unable to move in this matrix : ');
        %     display(qMatrix{i}(s,:));
        % end
        MoveSwords(robot, sword, vertices);
        pause(0.001);
    end

end

function MoveRobotToLocation(robots, locations, swords, swordVertices, steps)
    qMatrix = cell(1, length(robots));
    
    for i = 1:length(robots)
        robot = robots{i};
        location = locations{i};
        qMatrix{i} = CalculateQmatrix(robot, location, steps);
    end
    
    for s = 1:steps
        for i = 1:length(robots)
            robot = robots{i};
            sword = swords{i};
            vertices = swordVertices{i};
            collision = CheckCollision(robots{1}, robots{2});
            if collision 
                c =0;
            end
            try 
                robot.model.animate(qMatrix{i}(s,:)); 
            catch 
                disp('Unable to move in this matrix : ');
                display(qMatrix{i}(s,:));
            end
            MoveSwords(robot, sword, vertices);
            pause(0.001);
        end
    end
end

function qMatrix = CalculateQmatrix(robot, location, steps)
    % Ensure output is initialized
    qMatrix = [];
    
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
    % centerPoint = [0,0,0];
    % radii = [1,0.2,0.2];
    % [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    % workspace_limits = [-2 3 -1 2 -0 3]; % Example limits, adjust as needed
    % robots{1}.model.plot3d(robots{1}.model.getpos, 'workspace', workspace_limits);
    % robots{2}.model.plot3d(robots{2}.model.getpos, 'workspace', workspace_limits);
    for i = 1:length(robots)
        robot = robots{i};
        % for k = 1:(robot.model.n + 1)
        %     robot.model.points{k} = [X(:),Y(:),Z(:)];
        %     warning off
        %     robot.model.faces{k} = delaunay(robot.model.points{k});
        %     warning on;
        % end
        % robots{i} = robot;

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
    axis equal
    % view(3)
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
        
        current_transform = transforms(:,:, i);
        
        current_transform = current_transform * trotz(q(1,i)) * transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
        transforms(:,:,i + 1) = current_transform;
    end
end

function collision = CheckCollision(robot1, robot2)
    % Get link poses for both robots
    poses1 = GetLinkPoses(robot1.model.getpos, robot1.model);
    poses2 = GetLinkPoses(robot2.model.getpos, robot2.model);

    % Check for collisions between the links
    for i = 2:size(poses1, 3)
        distanceFromXYZ = sqrt(robot1.model.links(i-1).a^2 + robot1.model.links(i-1).d^2);
        radii = [distanceFromXYZ/1.5, 0.1, 0.1];  % Adjust as needed
        center = poses1(1:3, 4, i)';  % Use the transformation matrix to get the center

        for k = 2:size(robot2.model.points, 2)
            PointsAndOnes = [inv(poses1(:,:,i)) * [robot2.model.points{k},ones(size(robot2.model.points{k},1),1)]']';
            updatedPoints = PointsAndOnes(:,1:3);
            algebraicDist = GetAlgebraicDist(updatedPoints, center, radii);
            pointsInside = find(algebraicDist < 1);
            if (length(pointsInside) > 1)
                disp(['Collision detected: ', num2str(size(pointsInside,1)),' points inside']);
                collision = true;
                return;
            end
        end
    end
    collision = false;
end
function collision = CheckCollision2(robot1, robot2, poses1, poses2)
    % Get link poses for both robots
    % poses1 = GetLinkPoses(robot1.model.getpos, robot1.model);
    % poses2 = GetLinkPoses(robot2.model.getpos, robot2.model);

    % Check for collisions between the links
    for i = 2:size(poses1, 3)
        distanceFromXYZ = sqrt(robot1.model.links(i-1).a^2 + robot1.model.links(i-1).d^2);
        radii = [distanceFromXYZ/1.5, 0.1, 0.1];  % Adjust as needed
        center = poses1(1:3, 4, i)';  % Use the transformation matrix to get the center

        for k = 2:size(poses2, 3)
            % Transform the points of robot2 using poses2
            PointsAndOnes = (poses2(:,:,k) * [robot2.model.points{k-1}, ones(size(robot2.model.points{k-1},1),1)]')';
            updatedPoints = PointsAndOnes(:,1:3);

            algebraicDist = GetAlgebraicDist(updatedPoints, center, radii);
            pointsInside = find(algebraicDist < 1, 1);
            if ~isempty(pointsInside)
                disp(['Collision detected between link ', num2str(i), ' of robot1 and link ', num2str(k), ' of robot2']);
                collision = true;
                return;
            end
        end
    end
    collision = false;
end

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
