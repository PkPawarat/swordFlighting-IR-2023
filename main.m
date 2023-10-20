classdef main
    %MAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot1;
        robot2;
        
        sword1;
        sword2;
        
        sword1StartLoc = [-0.4 -0.5 0.5];       % adjust along the x axis for station position
        sword2StartLoc = [-0.4 1.55 0.5];       % adjust along the x axis for station position
        
        swordfile1 = "model3D/Lightsaber2.ply";
        swordfile2 = "model3D/Lightsaber2.ply";

        sword1_vertices;
        sword2_vertices;

        baseRobot1 = transl(0,0,0.5);
        baseRobot2 = transl(0,1,0.5);
        
        steps = 50;
        advance = false;            % set advance to true if want to turn on GUI
    end
    
    methods
        function self = main()
            clf;
            hold on;
            axis equal
            camlight
            self = self.SetupRobots();
            
            
            self.setupEnvironment();
            self = self.setupSwords();
            
            pose1 = self.robot1.model.fkine(self.robot1.homeQ).T;
            pose2 = self.robot2.model.fkine(self.robot2.homeQ).T;

            self = self.pickupSwords(self.robot1, self.sword1StartLoc, [], self.swordfile1);
            self.MoveRobotToLocation(self.robot1, pose1, self.sword1, self.sword1_vertices) ;
            
            self = self.pickupSwords(self.robot2, self.sword2StartLoc, [], self.swordfile2);
            self.MoveRobotToLocation(self.robot2, pose2, self.sword2, self.sword2_vertices) ;

            % q = self.robot2.model.ikine(q, self.robot2.model.getpos, 'mask', [1,1,1,1,1,0])
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

        function self = SetupRobots(self)
            % set the location according table
            self.robot1 = LinearUR5(self.baseRobot1);
            self.robot2 = FanucM20(self.baseRobot2);

            self.robot1.model.delay = 0.01;
            self.robot2.model.delay = 0.01;
            
            q1 = zeros(1, length(self.robot1.model.links));
            self.robot1.model.animate(q1);

            q2 = zeros(1, length(self.robot2.model.links));
            self.robot2.homeQ = q2;
            scale = 0.3;
            self.robot2.model.plot(q2, 'scale', scale);

        end

        
        function self = setupEnvironment(self)
            hold on;
            PlaceObject('table_v1.ply', [-0.4,0,0]);
            PlaceObject('table_v1.ply', [-0.4,1,0]);
            view(3);

            axis([-2 2 -2 2 0 3]);
        end
        
        function self = setupSwords(self)
            self.sword1 = self.DeleteObject(self.sword1);
            self.sword2 = self.DeleteObject(self.sword2);

            self.sword1 = PlaceObject(self.swordfile1);
            self.sword2 = PlaceObject(self.swordfile2);

            self.sword1_vertices = get(self.sword1, 'Vertices');
            self.sword2_vertices  = get(self.sword2, 'Vertices');
            % pose1 = self.robot1.model.fkine(self.robot1.model.getpos).T;
            self.RotateObject(self.sword1, (transl(self.sword1StartLoc) * trotx(90, 'deg') ));
            self.RotateObject(self.sword2, (transl(self.sword2StartLoc) * trotx(90, 'deg') ));

            % self.sword1_vertices = get(self.sword1, 'Vertices');
            % self.sword2_vertices  = get(self.sword2, 'Vertices');
            % [minXYZ, maxXYZ] = getObjectMinMaxVertices(self, self.sword1);
        end
        
        function self = pickupSwords(self, robot, location, object, objectVertic)
            location = transl(location) * troty(90, 'deg') * trotx(90, 'deg');
            self.MoveRobotToLocation(robot, location, object, objectVertic);
        end
        
        function [self, sword]= MoveSwords(self, robot, sword, sword_vertices)
            if (isempty(sword)) 
                return;  
            end
            robotPose = robot.model.fkine(robot.model.getpos).T * trotz(90, 'deg');
            transformedVertices = [sword_vertices, ones(size(sword_vertices,1),1)] * robotPose';
            set(sword,'Vertices',transformedVertices(:,1:3));
        end
        
        
        function self = MoveRobotToLocation(self, robot, location, object, objectVertic)
            if length(location) == 3
                MoveToObject = transl(location) * troty(-90,'deg');% * trotz(90,'deg');
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
            qMatrix = jtraj(initialGuess,newQ1,self.steps);
            for i = 1:length(qMatrix)
                try 
                    robot.model.animate(qMatrix(i,:)); 
                catch 
                    disp('Unable to move in this matrix : ')
                    display(qMatrix(i,:))
                end
                [self, object] = self.MoveSwords(robot, object, objectVertic)
                pause(0.001);
               
            end
        end

        function self = DeleteObject(self, object)
            try 
                delete(object); 
            end
        end
        function RotateObject(self, object, transformMatrix)
            % Get the object's vertices
            vertices = get(object, 'Vertices');
            % Apply the rotation transformation to the object's vertices
            transformedVertices = (transformMatrix * [vertices, ones(size(vertices, 1), 1)]')';
            % Update the object's vertices with the rotated vertices
            set(object, 'Vertices', transformedVertices(:, 1:3));
        end

        function MoveObject(object, rotationMatrix, rotationCenter)
            % Get the object's vertices
            vertices = get(object, 'Vertices');
            
            % Translate vertices to origin based on rotation center
            vertices = vertices - repmat(rotationCenter, size(vertices, 1), 1);
            
            % Rotate the vertices
            rotatedVertices = (rotationMatrix * vertices')';
            
            % Translate back to original location
            rotatedVertices = rotatedVertices + repmat(rotationCenter, size(vertices, 1), 1);
            
            % Update the object's vertices
            set(object, 'Vertices', rotatedVertices);
        end


        function [minXYZ, maxXYZ] = getObjectMinMaxVertices(self, object)
            % Extracting the vertices of the object
            vertices = get(object, 'Vertices');
            
            % Finding the min and max points along each axis
            minX = min(vertices(:, 1));
            minY = min(vertices(:, 2));
            minZ = min(vertices(:, 3));
            
            maxX = max(vertices(:, 1));
            maxY = max(vertices(:, 2));
            maxZ = max(vertices(:, 3));
            
            minXYZ = [minX, minY, minZ];
            maxXYZ = [maxX, maxY, maxZ];
            
            % Displaying the results
            % fprintf('Min XYZ: [%f, %f, %f]\n', minXYZ);
            % fprintf('Max XYZ: [%f, %f, %f]\n', maxXYZ);
        end
        
        
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



