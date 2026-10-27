%% Homework 2 Question 3
% By: Max Black
% Due: 10/13/26

clear
close all
clc

% workspace and obstacle definition
x = 0:1:100;
y = 0:1:100;
[X, Y] = meshgrid(x, y);
obstacle = [30 50;
    50 50;
    50 70;
    30 70];
center = [40 60];
eta = 0.5;
rho0 = 20;

% distance
d = sqrt((center(1)-X).^2+(center(2)-Y).^2);
rho_eff = max(d, 11);

% repulsive 
U_rep = 0.5 * eta * (1./rho_eff - 1/rho0).^2;
U_rep(d > rho0) = 0;

% visualize
figure;
surf(X, Y, U_rep);
xlabel('X'); 
ylabel('Y'); 
zlabel('Attractive Potential');
title('Question 3: Square Obstacle ');
shading interp;