%% Homework 2 Question 6
% By: Max Black
% Due: 10/13/26

clear;
clc;
close all;

% workspace definition
x = 0:1:100;
y = 0:1:100;
[X, Y] = meshgrid(x, y);

% intialize known variables and constants
q_start = [10 80];
q_goal = [80 20];
obstacle = [30 50;
    50 50;
    50 70;
    30 70];
center = [40 60];
epsilon = 0.01;
eta = 100000;
d0 = 20;
rho_min = 11;
tol = 0.5; 
dt = 0.1;
vmax = 5;
nstep = 20000;

q = q_start; % initial q 
path = q; % initial path vector

% non holomic robot
theta = 0; 
kp = 2;
robot_path = [q theta];

for step = 1:nstep
    
    % attractive force
    F_att = -epsilon * (q-q_goal); 
    
    % repulsive force
    rho = sqrt((q(1)-center(1))^2 + (q(2)-center(2))^2);
    if rho <= d0
        grad_rho = (q-center)/rho;
        rho_eff = max(rho, rho_min);
        F_rep = eta*((1/rho_eff)-1/d0)*(1/rho_eff^2)*grad_rho;
    else
        F_rep = [0 0];
    end

    u = F_att + F_rep;

    theta_d = atan2(u(2), u(1));

    gamma = kp * (theta_d-theta); 
    v = sqrt(u(1)^2+u(2)^2);
    if v > vmax
        v = vmax;
    end

    q(1) = q(1) + v*cos(theta)*dt;
    q(2) = q(2) + v*sin(theta)*dt;
    theta = theta + gamma*dt;

    path(end+1, :) = q;
    robot_path(end+1, :) = [q theta];

    if sqrt((q(1)-q_goal(1))^2 + (q(2)-q_goal(2))^2) < tol 
        break 
    end
end

d_goal = sqrt((q_goal(1)-X).^2 + (q_goal(2)-Y).^2);
d_obs  = sqrt((center(1)-X).^2 + (center(2)-Y).^2);
U_att  = 0.5 * epsilon * d_goal.^2;
U_rep  = 0.5 * eta * (1./max(d_obs, rho_min) - 1/d0).^2;
U_rep(d_obs > d0) = 0;
U_total = U_att + U_rep;

%% Animation
figure; 
hold on;
contour(X, Y, U_total, 30);
plot(q_start(1), q_start(2), 'go');
plot(q_goal(1),  q_goal(2),  'bx');
axis([0 100 0 100]);
xlabel('X (m)'); 
ylabel('Y (m)');
title('Question 6: Non-holomic Robot Animation');

body = fill(0, 0, 'b');
trail = plot(path(1,1), path(1,2), 'r-');

for i = 1:1:size(robot_path, 1)
    corners = Black_Max_SquareRobot(robot_path(i,1), robot_path(i,2), robot_path(i,3));
    set(body,  'XData', corners(:,1), 'YData', corners(:,2));
    set(trail, 'XData', path(1:i,1),  'YData', path(1:i,2));
    drawnow;
end

corners = Black_Max_SquareRobot(robot_path(end, 1), robot_path(end, 2), robot_path(end, 3));
set(body,  'XData', corners(:,1), 'YData', corners(:,2));
set(trail, 'XData', path(:,1),    'YData', path(:,2));