close all
hold on
robot = LinearUR5(transl(0,0,0.5));
u = PlaceObject('table_v1.ply', [-0.4,0,0]);
u = PlaceObject('table_v1.ply', [-0.4,2,0]);
robot2 = babyYODA(transl(0,2,0.5));
%%
clf
close all

r = FanucM20;
r.model.teach();


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
link(1) = Link([0        0.525     0.1  -pi/2   0  ]);
link(2) = Link([0        0     0.840   pi     0  ]);
link(3) = Link([0        0     0.27627    0  0  ]);
link(4) = Link([0        0     0   -pi/2  0  ]);
link(5) = Link([0       1.1665      0      pi/2   0  ]);
link(6) = Link([0        0.075     0     0  0  ]);
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
