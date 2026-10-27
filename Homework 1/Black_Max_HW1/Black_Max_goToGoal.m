%% Question 2 - Go to Goal
% by Max Black, xgv2zx

clear
close all
clc

%% Parameters

% Workspace Size
xlim([0 200])
ylim([0 200])


%Initialize a vector of positions for the robot
x=[]; 
y=[];

% Initialize a vector to save velocity info and time
vd = [];
t = [];
t(1) = 0;
% Robot initial config

x(1) = 100;
y(1) = 100; 
theta(1) = rand()*2*pi; 

% Go to goal setup
kv = 0.2;
kp = 2;
tolerance = 0.5; % used to compare with distance of robot to goal
xg = rand()*200;
yg = rand()*200;
dist = hypot(xg-x(1), yg-y(1)); % initial distance
dt = 0.1;
i = 1; % index to build x(i+1), y(i+1), theta(i+1)

% Build Robot Model and show where the goal is
robot = Black_Max_SquareRobot(x,y,theta(1));

fill (robot(:,1), robot(:,2), 'b');
xlim([0 200]);
ylim([0 200]);

% go to goal loop
while (dist > tolerance)
    dist = hypot(xg - x(i), yg - y(i));
    vd(i) = kv*hypot(xg-x(i), yg-y(i));
    thetad = atan2(yg - y(i), xg - x(i));
    gamma = kp*(thetad-theta(i));
    x(i+1) = x(i) + vd(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vd(i) * sin(theta(i)) * dt;
    theta(i+1) = theta(i) + gamma * dt;
    t(i+1) = t(i) + 0.1;
    i = i + 1;

    robot = Black_Max_SquareRobot(x(i),y(i),theta(i));
    cla
    fill(robot(:,1), robot(:,2), 'b');
    hold on
    plot(x,y,'-');
    plot(xg, yg, 'r*');
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
end 

% plots
vd(end+1) = 0; % velocity ends at 0

figure
subplot(3, 1, 1);
plot(t, x);
xlabel('time (s)');
ylabel('x position (m)');
title('x position vs. time');

subplot(3, 1, 2);
plot(t, y);
xlabel('time (s)');
ylabel('y position (m)');
title('y position vs. time');

subplot(3, 1, 3);
plot(t, vd);
xlabel('time (s)');
ylabel('velocity (m/s)');
title('velocity vs. time');