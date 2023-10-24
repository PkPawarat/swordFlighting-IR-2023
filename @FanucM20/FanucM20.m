classdef FanucM20 < RobotBaseClass
    %% FANUC M20 12L model
    properties(Access = public)              
        plyFileNameStem = 'FanucM20';
    end

    methods
%% Define robot Function 
    function self = FanucM20(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * baseTr * trotx(0) * troty(0);
            
            self.PlotAndColourRobot();         
    end

%% Create the robot model
    function CreateModel(self)   
        % Create the M20 model mounted on a linear rail
        
        link(1) = Link([0        0.475     0.1        pi/2   0]);
        link(2) = Link([0        0         0.840    pi     0]);
        link(3) = Link([0        0         0.27627  0      0]);
        link(4) = Link([0        0         0       -pi/2   0]);
        link(5) = Link([0       1.1665     0        pi/2   0]);
        link(6) = Link([0        0.075     0        0      0]);
        
        % Incorporate joint limits
        link(1).qlim = [-180 180]*pi/180;
        link(2).qlim = [-100 160]*pi/180;
        link(3).qlim = [-125 125]*pi/180;
        link(4).qlim = [-225 50]*pi/180;
        link(5).qlim = [-180 180]*pi/180;
        link(6).qlim = [-450 450]*pi/180;
    
        link(2).offset = pi/2;
        
        self.model = SerialLink(link,'name',self.name);
    end
     
    end
end