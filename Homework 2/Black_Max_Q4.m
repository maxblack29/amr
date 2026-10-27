%% Homework 2 Question 4
% By: Max Black
% Due: 10/13/26

clear
close all
clc

% workspace definition
x = 0:1:100;
y = 0:1:100;
[X, Y] = meshgrid(x, y);

% attractive potential field
goal = [80, 20];
epsilon = 0.01;
d_goal = sqrt((goal(1)-X).^2 + (goal(2)-Y).^2); 
U_att = 0.5 * epsilon * d_goal.^2; 

% repulsive potential field
obstacle = [30 50;
    50 50;
    50 70;
    30 70];
center = [40 60];
eta = 100000;
rho0 = 20;
rho = sqrt((center(1)-X).^2+(center(2)-Y).^2);
rho_eff = max(rho, 11);
U_rep = 0.5 * eta * (1./rho_eff - 1/rho0).^2;
U_rep(rho>rho0) = 0;

% combination and visualization
U_total = U_att + U_rep;
figure;
surf(X, Y, U_total, 'EdgeColor', 'none');
xlabel('X'); 
ylabel('Y'); 
zlabel('Potential');
title('Question 4: Combined Attractive and Repulsive Potential Field');
colorbar; 
view(3);