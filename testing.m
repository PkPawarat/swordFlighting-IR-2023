close all
hold on
robot = LinearUR5(transl(0,0,0.5));
u = PlaceObject('table_v1.ply', [-0.4,0,0]);
u = PlaceObject('table_v1.ply', [-0.4,2,0]);
robot2 = babyYODA(transl(0,2,0.5));
%% USE THIS FOR CALLING
close all
hold on
axis([-2 2 -2 2 -3 3]);
scale = 0.5;
r = FanucM20;
q = [0,0,0,0,0,0]
r.model.plot(q, 'scale', scale)
r.model.teach;

% r.model.animate(q);

%%
close all
hold on
axis equal
link(1) = Link([0        0.425     0  -pi/2   0  ]);
link(2) = Link([0        0     0.840   pi     0  ]);
link(3) = Link([0        0     0.27627    0  0  ]);
link(4) = Link([0        0     0   -pi/2  0  ]);
link(5) = Link([0       1.1665      0      pi/2   0  ]);
link(6) = Link([0        0.075     0     0  0  ]);
name = 'myrobot'

robot = SerialLink(link,'name', name);

q = [0,0,0,0,0,0]
robot.plot(q)
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
clf
robot2 = FanucM20();
%%
q = zeros(1, length(robot2.model.links));
robot2.model.plot(q)
%%
a = robot2.model.fkine(robot2.model.getpos).T * troty(-90/2, 'deg') 

b = robot2.model.ikine(a)

robot2.model.plot(b)

