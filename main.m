classdef main
    %MAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function self = main()
            
        end
        
        function setupEnvironment()
            u = PlaceObject('table_v1.ply', [-0.4,0,0]);
            u = PlaceObject('table_v1.ply', [-0.4,2,0]);
            robot = LinearUR5(transl(0,0,0.5));
            %second robot
        end
    end
end

