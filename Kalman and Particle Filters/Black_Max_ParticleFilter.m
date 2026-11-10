%% Homework 3 Question 2 - Particle Filter
% By: Max Black
% Due: 11/6/2026

clear
close all
clc

% landmark definitions and other intializations
landmarks = [25 25;
    25 70;
    70 25;
    70 70;
    10 40;
    80 60];
particles = zeros(1000, 3);
weights = zeros(1000,1);
pstar = zeros(6,1000);
norm_w = zeros(1000,1);

% robot initial pose and store the initial pose
x = 30 + rand()*40;
x0 = x;
y = 30 + rand()*40;
y0 = y;
theta = rand()*2*pi;
theta0 = theta;

% generate and store the random particles
for i=1:1000
    x_p = rand()*100;
    y_p = rand()*100;
    theta_p = rand()*2*pi;
    particles(i,1) = x_p;
    particles(i,2) = y_p;
    particles(i,3) = theta_p;
end

particles0 = particles; % save initial particle layout

% robot movement and adding noise to the particles
for j = 1:10
    n_t = sqrt(0.5)*randn;
    n_dist = sqrt(0.5)*randn;
    theta = theta + 0.2 + n_t;
    dist = 1 + n_dist;
    x = x + dist*cos(theta);
    y = y + dist*sin(theta);

    % update the particles with noise
    particles(:,3) = particles(:,3) + 0.2 + sqrt(0.5)*randn(1000, 1);
    dist_p = 1 + sqrt(0.5)*randn(1000,1);
    particles(:,1) = particles(:,1) + dist_p.*cos(particles(:,3));
    particles(:,2) = particles(:,2) + dist_p.*sin(particles(:,3));

    %find the robot's distance to the landmarks
    lm_dist = sqrt((x - landmarks(:,1)).^2 + (y-landmarks(:,2)).^2);
    lm_meas = lm_dist + sqrt(8)*randn(6,1);

    % find the weights
    for p = 1:1000
        dist_i = sqrt((particles(p,1)-landmarks(:,1)).^2+(particles(p,2)-landmarks(:,2)).^2);
        pstar(:,p) = (1/sqrt(2*pi*8))*exp(-0.5*((dist_i-lm_meas).^2)/8);
    end
    weights = prod(pstar, 1)';
    index = randi(1000);
    beta = 0;
    wmax = max(weights);
    new_particles = zeros(1000,3);
    for i = 1:1000
        beta = beta + rand()*2*wmax;
        while weights(index) < beta
            beta = beta - weights(index);
            index = mod(index, 1000) + 1;
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