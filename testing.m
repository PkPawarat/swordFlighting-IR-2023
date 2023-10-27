close all
hold on
robot = LinearUR5(transl(0,0,0.5));
u = PlaceObject('table_v1.ply', [-0.4,0,0]);
u = PlaceObject('table_v1.ply', [-0.4,2,0]);
robot2 = babyYODA(transl(0,2,0.5));
%%
clf
% close all

r = FanucM20;
r.model.teach();
axis([-2 2 -2 2 0 3])

%% USE THIS FOR CALLING
close all
hold on
axis([-2 2 -2 2 -2 3]);
scale = 0.5;
steps = 25;
r = FanucM20;
q = [0,pi/2,0,0,0,0]
r.model.plot(q, 'scale', scale);
r.model.teach;
% a = r.model.getpos
% q2 = [0,pi/2,pi/2,0,0,0]
% qpath = jtraj(r.model.getpos, q2, steps);
% for i = 1:steps
%     r.model.animate(qpath(i,:));
% end

%%
close all
hold on
axis([-2 2 -2 2 -2 3]);
link(1) = Link([0        0.475     0.1        pi/2   0]);
link(2) = Link([0        0         0.840    pi     0]);
link(3) = Link([0        0         0.27627  -pi/2      0]);
link(4) = Link([0        0         0        0   0]);
link(5) = Link([0       1.1665     0        pi/2   0]);
link(6) = Link([0        0.075     0        0      0]);
name = 'myrobot'

robot = SerialLink(link,'name', name);

q = [0,0,0,0,0,0]
robot.plot(q, 'scale', 0.5)
robot.teach;

%%
close all
hold on
axis equal
L(1) = Link([0        0.425     0  -pi/2   0  ]);
L(2) = Link([0        0     0.840   pi     0  ]);
L(3) = Link([0        0     0.27627    0  0  ]);
L(4) = Link([0        0     0   -pi/2  0  ]);
L(5) = Link([0       1.1665      0      pi/2   0  ]);
L(6) = Link([0        0.075     0     0  0  ]);


q =[0   -pi/2   0   0   0   0];
robot=SerialLink(L, 'name', 'Fanuc AM120iB/10L');


robot.plot(q)
robot.teach;


%%
close all
robot2 = LinearUR5;
% robot2.model.teach;
axis([-2 2 -2 2 0 3])

q1 = [-0.1,0,0,0,0,0,0];

q2 = [-0.8,0,0,0,0,0,0];
steps = 25;
qpath = jtraj(robot2.model.getpos, q2, steps);
pause(5);
for l = 1:length(qpath)
        robot2.model.animate(qpath(l,:));
        drawnow();
        pause(0.01);
end



%%
q = zeros(1, length(robot2.model.links));
robot2.model.plot(q)
%%
a = robot2.model.fkine(robot2.model.getpos).T * troty(-90/2, 'deg') 

b = robot2.model.ikine(a)

robot2.model.plot(b)
%% create cube

clf
clear

% One side of the cube
[Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
sizeMat = size(Y);
X = repmat(0.75,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];         
         
% Plot the cube's point cloud         
cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
axis equal

%%
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])        
robot = SerialLink([L1 L2 L3],'name','myRobot');  
robot.plot([0 0 0])
% New values for the ellipsoid (guessed these, need proper model to work out correctly)
centerPoint = [0,0,0];
radii = [1,0.2,0.2];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:4
    robot.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.faces{i} = delaunay(robot.points{i});    
    warning on;
end

robot.plot3d([0,0,0]);
axis equal
camlight
% 2.9
q = [0,0,0];

% UPDATE: fkine function now returns an SE3 object.
% To obtain the Transform Matrix, access the
% variable in the object 'T' with '.T'.
%% at the end effector how make point is in the cube
q = [0,pi/2,0];
tr = robot.fkine(q).T;
cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
disp(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);


%% 2.10     in each link how many point of ellipsoid is in the cube
q = [0,0,0];
robot.plot3d(q)
view(3)
axis([-1 5 -2 2 -1 3]);

tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% Go through each ellipsoid
for i = 1: size(tr,3)
    cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
    updatedCubePoints = cubePointsAndOnes(:,1:3);
    algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);
    disp(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
end

%% create new robot 
clf
clear
clc
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',1,'a',0.5,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',3,'alpha',-pi/2,'qlim',[-pi pi]);        
robot = SerialLink([L1 L2 L3],'name','myRobot');  

q = [0,0,0];
robot.plot(q)
axis([-1 5 -2 5 -1 3]);

% Extract 'a' values from each link to determine the major axis of the ellipsoid
link_lengths = [L1.a, L2.a, L3.a];

% Determine radii for each ellipsoid based on link lengths
radiiMatrix = [link_lengths' link_lengths'*0.3 link_lengths'*0.3];

% Get transformation matrices for each link
T0 = robot.base.T;
T1 = T0 * L1.A(0).T;
T2 = T1 * L2.A(0).T; % Using pi/2 for the second joint angle
T3 = T2 * L3.A(0).T;

transformations = {T0, T1, T2, T3};

numLinks = robot.n+1;  % Get the number of links
% point = []
for i = 2:numLinks  % Start from the second link
    % Compute the distance between two consecutive transformation matrices
    distanceFromXYZ = robot.links(i-1).a'
    % Use the computed distance to determine the radii of the ellipsoid
    radii = [distanceFromXYZ/1.5, 0.8, 0.8];  % Adjust as needed
    center = [-distanceFromXYZ/2 0 0];
    [X,Y,Z] = ellipsoid(center(1), center(2), center(3), radii(1), radii(2), radii(3));
    robot.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.faces{i} = delaunay(robot.points{i});    
    warning on;
    point{i} = robot.points{i};
    % Visualize the ellipsoid
    % figure;
    % hold on;
    % surf(X, Y, Z);
    % title(['Ellipsoid for Link ', num2str(i)]);
    % xlabel('X'); ylabel('Y'); zlabel('Z');
    % axis equal;
    % shading interp;
    % colormap jet;
    % light;
    % lighting gouraud;
end

workspace_limits = [-2 5 -2 5 -1 3]; % Define the workspace limits//
axis([-2 10 -5 5 0 3]);
% robot.plot3d(q, 'workspace', workspace_limits);
%%
hold on
robot2 = SerialLink([L1 L2 L3],'name','myRobot2');  
robot2.base = [2 3 0];
robot2.plot(q)
axis equal

% Extract 'a' values from each link to determine the major axis of the ellipsoid
link_lengths = [L1.a, L2.a, L3.a];
% Determine radii for each ellipsoid based on link lengths
radiiMatrix = [link_lengths' link_lengths'*0.3 link_lengths'*0.3];
% Get transformation matrices for each link
T0 = robot2.base.T;
T1 = T0 * L1.A(0).T;
T2 = T1 * L2.A(0).T; % Using pi/2 for the second joint angle
T3 = T2 * L3.A(0).T;
transformations = {T0, T1, T2, T3};
numLinks = robot2.n+1;  % Get the number of links
for i = 2:numLinks  % Start from the second link
    % Compute the distance between two consecutive transformation matrices
    distanceFromXYZ = robot2.links(i-1).a';
    % Use the computed distance to determine the radii of the ellipsoid
    radii = [distanceFromXYZ/1.5, 0.3, 0.3];  % Adjust as needed
    center = [-distanceFromXYZ/2 0 0];
    [X,Y,Z] = ellipsoid(center(1), center(2), center(3), radii(1), radii(2), radii(3));
    robot2.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot2.faces{i} = delaunay(robot2.points{i});    
    warning on;
end
robot2.plot3d(q, 'workspace', workspace_limits);
axis([-2 10 -5 5 0 3]);
view(3)
%%
robot2.teach('rpy/zyx',robot2.getpos);
qCurrent = [];
newpoint = [];
while(~isempty(findobj(gcf,'Title','Teach')))
    if ~isequal(qCurrent, robot2.getpos)
        qCurrent = robot2.getpos;
         % disp(['The Jacobian for the current joint angles ([', ...
         %    num2str(robot2.getpos),']) is:']);
         J = robot2.jacob0(robot2.getpos);
        % disp(J);

        %
        q = robot2.getpos;
        tr = zeros(4,4,robot2.n+1);
        tr(:,:,1) = robot2.base;
        L = robot2.links;
        for i = 1 : robot2.n
            tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            % point{i+1} = point{i+1} * tr(1:3,4,i+1);
        end

        
        q = robot.getpos;
        tr1 = zeros(4,4,robot.n+1);
        tr1(:,:,1) = robot.base;
        L = robot.links;
        for i = 1 : robot.n
            tr1(:,:,i+1) = tr1(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            newpoint{i+1} = bsxfun(@plus, point{i+1}, tr(1:3,4,i+1)');

        end

        numLinks = robot.n+1;
        for i = 2:numLinks  % Start from the second link
            % Compute the distance between two consecutive transformation matrices
            distanceFromXYZ = robot.links(i-1).a';
            % Use the computed distance to determine the radii of the ellipsoid
            radii = [distanceFromXYZ/1.5, 0.3, 0.3];
            center = [tr1(1,4,i)/1.5,tr1(2,4,i),tr1(3,4,i)];
            [X,Y,Z] = ellipsoid(center(1), center(2), center(3), radii(1), radii(2), radii(3));
            surf(X, Y, Z);

            points{i} = [X(:),Y(:),Z(:)];
            warning off
            faces{i}  = delaunay(robot.points{i});  
            warning on;
        end
        
        
        % Go through each ellipsoid
        for i = 1: size(tr,3)
            for k = 2: size(points,2)
                % distanceFromXYZ = robot2.links(i-1).a';
                % Use the computed distance to determine the radii of the ellipsoid
                distanceFromXYZ = robot.links(k-1).a';
                radii = [distanceFromXYZ/1.5, 0.3, 0.3];
                % radii = [1,  0.8, 0.8];  
    
                cubePointsAndOnes = [inv(tr(:,:,i)) * [points{k},ones(size(points{k},1),1)]']';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedCubePoints, [0 0 0], radii);
                pointsInside = find(algebraicDist < 1);
                if length(pointsInside) > 0
                    disp(['2.10: There are ', num2str(size(pointsInside,1)),' tr inside the ',num2str(i), ' points inside the ',num2str(k), ' th ellipsoid']);
                end
            end
        end
    end
    pause(0)
end


%%
qCurrent = robot2.getpos;
 % disp(['The Jacobian for the current joint angles ([', ...
 %    num2str(robot2.getpos),']) is:']);
 J = robot2.jacob0(robot2.getpos);
% disp(J);

%
tr = zeros(4,4,robot2.n+1);
tr(:,:,1) = robot2.base;
L = robot2.links;
for i = 1 : robot2.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end


% Go through each ellipsoid
for i = 2: size(tr,3)

    % distanceFromXYZ = robot2.links(i-1).a';
    % Use the computed distance to determine the radii of the ellipsoid
    radii = [1, 0.3, 0.3];  
    poserobot1 = robot.getpos;
    cubePointsAndOnes = [inv(tr(:,:,i)) * [robot.points{i},ones(size(robot.points{i},1),1)]']';
    updatedCubePoints = cubePointsAndOnes(:,1:3);
    algebraicDist = GetAlgebraicDist(updatedCubePoints, [0 0 0], radii);
    pointsInside = find(algebraicDist < 1);
    disp(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
end






























%% function 
function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end

