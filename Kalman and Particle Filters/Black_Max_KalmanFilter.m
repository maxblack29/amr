%% Homework 3 Question 1 - Kalman Filter
% By: Max Black
% Due: 11/6/2026

clear
close all
clc

% Workspace Size
x_space = 0:1:100;
y_space = 0:1:100;
start = [25 25];
goal = [25 75];

% variables to store and save
i = 1; % first array index
x = [];
x1_sens = [];
x2_sens = [];
y = [];
y1_sens = [];
y2_sens = [];
vx = [];
n_vx = [];
vy = [];
n_vy = [];
dist = [];

% cruise control
speed = 2; % 2 m/s
dt = 0.1;
tol = 0.5;
xg = goal(1);
yg = goal(2);

% kalman filter
A = [1 0 0 0;
    0 1 0 0;
    0 0 0 0;
    0 0 0 0];
B = [dt 0;
    0 dt;
    1 0;
    0 1];

% Initialize the robot state at the specified start position
x(1) = start(1);
y(1) = start(2);
x1_sens(1) = x(1) + sqrt(6)*randn;
y1_sens(1) = y(1) + sqrt(6)*randn;
x2_sens(1) = x(1) + 2*randn;
y2_sens(1) = y(1) + 2*randn;
dist(1) = hypot(xg-x, yg-y); 

while (dist(i) > tol && i < 2000)
    vx(i) = speed*(xg-x(i))/dist(i);
    vy(i) = speed*(yg-y(i))/dist(i);
    nx = randn;
    ny = randn;
    n_vx(i) = vx(i) + nx;
    n_vy(i) = vy(i) + ny;
    s1x = sqrt(6)*randn;
    s1y = sqrt(6)*randn;
    s2x = 2*randn;
    s2y = 2*randn;
    x(i+1) = x(i) + n_vx(i)*dt;
    y(i+1) = y(i) + n_vy(i)*dt;
    x1_sens(i+1) = x(i+1) + s1x;
    y1_sens(i+1) = y(i+1) + s1y;
    x2_sens(i+1) = x(i+1) + s2x;
    y2_sens(i+1) = y(i+1) + s2y;
    i = i+1; 
    dist(i) = hypot(xg-x(i), yg-y(i));

    cla
    plot(x(i), y(i), 'ro');
    hold on
    plot(x,y,'-');
    plot(xg, yg, 'r*');
    xlim([0 100])
    ylim([0 100])
    pause(0.01)
end

length(x1_sens)
length(y1_sens)
length(x)
length(y)

figure;
hold on
scatter(x1_sens, y1_sens, 'r');
scatter(x2_sens, y2_sens, 'b');
plot(x, y, 'Color', 'g');
axis equal