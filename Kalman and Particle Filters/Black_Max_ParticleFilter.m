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
particle_save = zeros(1000,3,10);
truth = zeros(10,3);
est_save = zeros(10,3);
err_save = zeros(10,2);

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
    for r = 1:1000
        beta = beta + rand()*2*wmax;
        while weights(index) < beta
            beta = beta - weights(index);
            index = mod(index, 1000) + 1;
        end
        new_particles(r,:) = particles(index,:);
    end
    particles = new_particles;

    % find the estimate and the error
    x_est = mean(particles(:,1));
    y_est = mean(particles(:,2));
    theta_est = atan2(mean(sin(particles(:,3))), mean(cos(particles(:,3))));
    pos_err = hypot(x-x_est, y-y_est);
    head_err = atan2(sin(theta-theta_est), cos(theta-theta_est));
    particle_save(:,:,j) = particles;
    truth(j,:) = [x y theta];
    est_save(j,:) = [x_est y_est theta_est];
    err_save(j,:) = [pos_err head_err];
    
    % print all the info that was asked for
    th_true = atan2(sin(theta), cos(theta));   % This is just to wrap the actual heading
    fprintf('Step %d\n', j);
    fprintf('actual pose: %.2f, %.2f, %.2f\n', x, y, mod(theta, 2*pi));
    fprintf('estimated pose: %.2f, %.2f, %.2f\n', x_est, y_est, mod(theta_est, 2*pi));
    fprintf('position error: %.2f\n', pos_err);
    fprintf('heading error: %.2f\n', head_err);
    disp('particles (x, y, theta):');
    disp(particles);
end

% plotting

% initial setup
figure(1); 
hold on
plot(particles0(:,1), particles0(:,2), 'g.', 'MarkerSize', 8);
plot(landmarks(:,1), landmarks(:,2), 'c+', 'MarkerSize', 12, 'LineWidth', 2);
plot(x0, y0, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
quiver(x0, y0, 5*cos(theta0), 5*sin(theta0), 'r', 'AutoScale', 'off', 'LineWidth', 1.5); % this is just to show the intial theta
axis equal; 
axis([0 100 0 100]); 
grid on
xlabel('x (m)'); 
ylabel('y (m)');
title('Initial 1000 particles, landmarks, and robot');
legend('Particles', 'Landmarks', 'Robot start');
hold off

% plot of the particles and robot after each step
for s = 1:10
    figure(1+s); 
    hold on
    plot(particle_save(:,1,s), particle_save(:,2,s), 'g.', 'MarkerSize', 8);
    plot(landmarks(:,1), landmarks(:,2), 'c+', 'MarkerSize', 12, 'LineWidth', 2);
    plot(truth(s,1), truth(s,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    plot(est_save(s,1), est_save(s,2), 'x', 'MarkerSize', 12, 'LineWidth', 2);
    axis equal; 
    axis([0 100 0 100]); 
    grid on
    xlabel('x (m)'); 
    ylabel('y (m)');
    title(sprintf('Step %d: particles after resampling', s));
    legend('Particles','Landmarks','True robot','Estimate');
    hold off
end

% plot of the error between the actual pose and each estimate over 10 steps
figure(12)
subplot(2,1,1)
plot(1:10, err_save(:,1), 'b-o', 'LineWidth', 1.5);
xlabel('Step'); 
ylabel('Position error (m)'); 
grid on
title('Position error of the estimate');
subplot(2,1,2)
plot(1:10, err_save(:,2), 'm-o', 'LineWidth', 1.5);
xlabel('Step'); 
ylabel('heading error (rad)'); 
grid on
title('Heading error of the estimate');