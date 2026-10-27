%% Homework 2 Question 2
% By: Max Black
% Due: 10/13/26

clear
close all
clc

% Define workspace and goal
x = 0:1:100; 
y = 0:1:100;
[X, Y] = meshgrid(x, y);
goal = [80, 20];
epsilon = 0.5;

% Attractive potential field
d = sqrt((goal(1)-X).^2 + (goal(2)-Y).^2); % find distance from every point in the matrix

% potential function U_att
U_att = 0.5 * epsilon * d.^2; 

%visualize
figure;
surf(X, Y, U_att);
xlabel('X'); 
ylabel('Y'); 
zlabel('Attractive Potential');
title('Question 2: Attractive Potential Field');
shading interp;