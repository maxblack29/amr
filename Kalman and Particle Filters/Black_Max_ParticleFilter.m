%% Homework 3 Question 2 - Particle Filter
% By: Max Black
% Due: 11/6/2026

clear
close all
clc

% landmark definitions and other givens
landmarks = [25 25;
    25 70;
    70 25;
    70 70;
    10 40;
    80 60];
particles = zeros(1000, 3);
err = 8*randn;
noise = 0.5*randn;

% robot initial pose
x = 30 + rand()*40;
y = 30 + rand()*40;
theta = rand()*2*pi;

% generate and store the random particles
for i=1:1000
    x_p = rand()*100;
    y_p = rand()*100;
    theta_p = rand()*2*pi;
    particles(i,1) = x_p;
    particles(i,2) = y_p;
    particles(i,3) = theta_p;
end

hold on
plot(particles(:,1), particles(:,2), 'g.', 'MarkerSize', 8);
plot(landmarks(:,1), landmarks(:,2), 'co', 'MarkerSize', 15);
plot(x, y, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlim([0 100]);
ylim([0 100]);
hold off
axis equal
grid on