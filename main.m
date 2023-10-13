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
            self.setupEnvironment();
            self = self.SetupRobots();
        end

        function self = SetupRobots(self)
            self.robot1 = LinearUR5(transl(-0.4,0,0.5));
            self.robot2 = babyYODA(transl(-0.4,1,0.5));

            % set the location according table

        end
        function AssignTarget(self, ss)

        end
            
        function setupEnvironment(self)
            u1 = PlaceObject('table_v1.ply', [-0.4,0,0]);
            hold on;
            u2 = PlaceObject('table_v1.ply', [-0.4,1,0]);
            hold on;
            % robot = LinearUR5(transl(0,0,0.5));
            %second robot
        end
        
    end
end

