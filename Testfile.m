% Initialization and Setup
clf;
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
pose1{1} = transl([-0.5 1 1]) * troty(90, 'deg');
pose1{2} = transl([0 0 1]) * trotz(-90, 'deg');

% Setup Robots
robot = cell(1,2);
robot{1} = SetupRobot(baseRobot{1}, false);
robot{2} = SetupRobot(baseRobot{2}, true);

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
    pickupSwordsStep(robot, swordStartLoc, swords, sword_vertices, steps);
%%
    MoveRobotToLocation(robot, pose1, swords, sword_vertices, steps)




%% Functions
function robot = SetupRobot(base, newRobot)
    if newRobot
        robot = FanucM20(base); 
    else
        robot = LinearUR5(base);
    end
    robot.model.delay = 0.01;
    q = zeros(1, length(robot.model.links));
    if newRobot
        scale = 0.3;
        robot.model.plot(q, 'scale', scale);
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

function tr = UpdateEachLink(robot)
    q = zeros(1,robot.model.n); 
    tr = zeros(4,4,robot.model.n+1);
    tr(:,:,1) = robot.model.base;
    L = robot.model.links;
    for i = 1 : robot.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end
end
% ... [Any other functions you had in the class]
