classdef main
    %MAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot1;
        robot2;
    end
    
    methods
        function self = main()
            clf;
            hold on;
            self.setupEnvironment();
            self = self.SetupRobots();
            q = [-0.1, 0,-pi/2,pi/2,0, 0];
            
            % q = self.robot2.model.ikine(q, self.robot2.model.getpos, 'mask', [1,1,1,1,1,0])
            self.robot2.model.animate(q);
        end

        function self = SetupRobots(self)
            self.robot1 = LinearUR5(transl(0,0,0.5));
            self.robot2 = babyYODA(transl(0,1,0.5));
            % set the location according table

        end

        function AssignTarget(self)

        end
            
        function setupEnvironment(self)
            u1 = PlaceObject('table_v1.ply', [-0.4,0,0]);
            u2 = PlaceObject('table_v1.ply', [-0.4,1,0]);
            % robot = LinearUR5(transl(0,0,0.5));
            %second robot
        end
        
    end
end




