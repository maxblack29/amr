%% Square robot

function [robot, rawCorners] = Black_Max_SquareRobot(x, y, theta)

center = [x, y]; % what we rotate about

% square shape
a = [4 4];
b = [4 -4];
c = [-4 -4];
d = [-4 4];

% output the raw corner point information for homogenous transformation
rawCorners = [a; b; c; d];

% rotation matrix
rotmat = [cos(theta) -sin(theta); sin(theta) cos(theta)];

rota = (rotmat * (a'));
rotb = (rotmat * (b'));
rotc = (rotmat * (c'));
rotd = (rotmat * (d'));

% post transform configuration

robot1 = [rota(1) + center(1), rota(2) + center(2)];
robot2 = [rotb(1) + center(1), rotb(2) + center(2)];
robot3 = [rotc(1) + center(1), rotc(2) + center(2)];
robot4 = [rotd(1) + center(1), rotd(2) + center(2)];

robot = [robot1;robot2;robot3;robot4];

end
