


clc;
clf;

% Parameters for the box
box_center = [0, 0, 0]; % The center of the box
box_width = 10; % Width of the box
box_height = 10; % Height of the box
box_depth = 10; % Depth of the box

% Create the box
box_planes = createBox(box_center, box_width, box_height, box_depth);

% Plot the planes of the box
colors = ['r', 'g', 'b', 'y']; % Different color for each plane
for i = 1:4
    surf(box_planes(i).x_grid, box_planes(i).y_grid, box_planes(i).z_grid, ...
         'FaceColor', colors(i), 'FaceAlpha', 0.5);
    hold on;
end

xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
xlim([-20, 20]);
ylim([-20, 20]);
zlim([-20, 20]);
grid on;

% Define the object's path (a line in 3D space)
% Starting point
obj_start = [-15, 0, 0];

% Direction vector (for example, [1, 0, 0])
obj_direction = [10, 0, 0];

% Parameter t for the line equation
t = linspace(-20, 20, 400);

% Animate the object
for i = 1:length(t)
    % Calculate the current point on the object's trajectory
    point = obj_start + t(i) * obj_direction;                               % IN THE MAIN we can you vertices as point 
    
    % Plot the object's current position        
    h = plot3(point(1), point(2), point(3), 'ko', 'MarkerFaceColor', 'k');
    
    % Check for intersection with the box
    if checkInsideBox(point, box_center, box_width, box_height, box_depth)
        set(h, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g'); % Change color to green if inside the box
        disp('OBJECT INSIDE THE BOXXXXXX')
        pause(0.01);

    else
        set(h, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k'); % Keep color black if outside the box
        % disp('OBJECT OUTSIDE THE BOXXXXXX')
    end
    
    pause(0.01); % Pause to visualize the animation
    
    % Optionally, delete the previous point for a cleaner animation
    if exist('h', 'var') && ishandle(h)
        delete(h);
    end
end



function [x_grid, y_grid, z_grid] = createPlane(point, width, height)
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
function intersect_point = lineIntersection(obj_start, obj_direction, plane_x)
    % obj_start: A 1x3 vector [x, y, z] representing the starting point of the line
    % obj_direction: A 1x3 vector [dx, dy, dz] representing the direction of the line
    % plane_x: The x-coordinate of the vertical plane
    
    % Calculate the parameter t at which the line intersects the plane
    t_intersect = (plane_x - obj_start(1)) / obj_direction(1);
    
    % Calculate the intersection point
    intersect_point = obj_start + t_intersect * obj_direction;
end
function [planes] = createBox(center, width, height, depth)
    % center: A 1x3 vector [x, y, z] representing the center of the box
    % width: The width of the box along the X-axis
    % height: The height of the box along the Y-axis
    % depth: The depth of the box along the Z-axis
    hold on;
    % Initialize the planes structure
    planes = struct('x_grid', [], 'y_grid', [], 'z_grid', []);
    
    % Half-dimensions
    halfWidth = width / 2;
    halfHeight = height / 2;
    halfDepth = depth / 2;
    
    % Define the grid for each plane
    [y_grid, z_grid] = meshgrid(linspace(-halfHeight, halfHeight, 10), linspace(-halfDepth, halfDepth, 10));
    
    % Front plane (x is constant)
    planes(1).x_grid = (center(1) + halfWidth) * ones(size(y_grid));
    planes(1).y_grid = y_grid + center(2);
    planes(1).z_grid = z_grid + center(3);
    
    % surf(planes(1).x_grid, planes(1).y_grid, planes(1).z_grid, ...
    %      'FaceColor', 'r', 'FaceAlpha', 0.5);
    
    % Back plane (x is constant)
    planes(2).x_grid = (center(1) - halfWidth) * ones(size(y_grid));
    planes(2).y_grid = y_grid + center(2);
    planes(2).z_grid = z_grid + center(3);
    
    % surf(planes(2).x_grid, planes(2).y_grid, planes(2).z_grid, ...
    %      'FaceColor', 'r', 'FaceAlpha', 0.5);
    
    % Left plane (y is constant)
    planes(3).y_grid = (center(2) - halfHeight) * ones(size(z_grid));
    planes(3).x_grid = z_grid + center(1); % z_grid reused for x since y is constant
    planes(3).z_grid = y_grid + center(3); % y_grid reused for z
    
    % surf(planes(3).x_grid, planes(3).y_grid, planes(3).z_grid, ...
    %      'FaceColor', 'r', 'FaceAlpha', 0.5);
    
    % Right plane (y is constant)
    planes(4).y_grid = (center(2) + halfHeight) * ones(size(z_grid));
    planes(4).x_grid = z_grid + center(1); % z_grid reused for x since y is constant
    planes(4).z_grid = y_grid + center(3); % y_grid reused for z

    % surf(planes(4).x_grid, planes(4).y_grid, planes(4).z_grid, ...
    %      'FaceColor', 'r', 'FaceAlpha', 0.5);
    
end
function isInside = checkInsideBox(point, box_center, width, height, depth)
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
