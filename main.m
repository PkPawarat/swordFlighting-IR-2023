classdef main
    %MAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot1;
        robot2;


    end
    
    methods
        function self = main()
            
        end
        
        function SetupEnvironment(self)

        end
        function SetupRobots(self)
            self.robot1 = LinearUR5;
            self.robot2 = babyYODA;

            % set the location according table

        end
        function AssignTarget(self)
            
        end
        
    end
end

