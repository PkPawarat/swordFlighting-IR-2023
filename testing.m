close all
hold on
robot = LinearUR5(transl(0,0,0.5));
u = PlaceObject('table_v1.ply', [-0.4,0,0]);
u = PlaceObject('table_v1.ply', [-0.4,2,0]);