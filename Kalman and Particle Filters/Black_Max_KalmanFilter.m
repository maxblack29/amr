%% Homework 3 Question 1 - Kalman Filter
% By: Max Black
% Due: 11/6/2026

clear
close all
clc

% starting and goal
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
C = [1 0 0 0;
    0 1 0 0];
Q = B*B';
sig_o1 = [6 0;
    0 6];
sig_o2 = [4 0;
    0 4];
state = [];
u = [];

% Initialize the robot state at the specified start position
x(1) = start(1);
y(1) = start(2);
x1_sens(1) = x(1) + sqrt(6)*randn;
y1_sens(1) = y(1) + sqrt(6)*randn;
x2_sens(1) = x(1) + 2*randn;
y2_sens(1) = y(1) + 2*randn;
dist(1) = hypot(xg-x, yg-y); 
state(:,1) = [x1_sens(1); y1_sens(1); 0; 0]; % initial state
P = 10*eye(4); 

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

% Kalman filter loop
for k = 2:length(x)
    u = [vx(k-1); vy(k-1)];
    xPred = A*state(:, k-1) + B*u;
    pPred = A*P*A'+ Q;
    K1 = pPred*C'*inv((C*pPred*C'+sig_o1));
    z = [x1_sens(k); y1_sens(k)];
    x1Upd = xPred + K1*(z - C*xPred);
    P = (eye(4) - K1*C)*pPred;
    K2 = P*C'*inv((C*P*C'+sig_o2));
    z1 = [x2_sens(k); y2_sens(k)];
    x2Upd = x1Upd + K2*(z1 - C*x1Upd);
    P1 = (eye(4) - K2*C)*P;
    P = P1;
    state(:, k) = x2Upd;
end

% Plotting
N = length(x);
t = dt*(0:N-1);
idx = 2:N;

est_x = state(1,:);
est_y = state(2,:);
est_speed = sqrt(state(3,:).^2 + state(4,:).^2);
est_dist = hypot(xg - est_x, yg - est_y);

figure(2);
subplot(3,1,1);
plot(t, est_x, 'b', t, est_y, 'r');
xlabel('Time (s)'); 
ylabel('Position (m)');
legend('Estimated x', 'Estimated y');
title('Estimated robot position');
grid on

subplot(3,1,2);
plot(t(idx), est_speed(idx), 'c');
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Estimated linear velocity');
grid on

subplot(3,1,3);
plot(t, est_dist, 'm');
xlabel('Time (s)');
ylabel('Distance (m)');
title('Estimated distance to goal');
grid on

% sensor measurements
figure(3);
subplot(2,1,1);
hold on
plot(t, x1_sens, 'r.');
plot(t, x2_sens, 'b.');
plot(t, x, 'g');
plot(t, est_x, 'c');
xlabel('Time(s)');
ylabel('x position (m)');
legend('Sensor 1', 'Sensor 2', 'True x', 'Estimated x');
title('Sensor measurements vs Kalman filter estimate for x');


subplot(2,1,2);
hold on
plot(t, y1_sens, 'r.');
plot(t, y2_sens, 'b.');
plot(t, y, 'g');
plot(t, est_y, 'c');
xlabel('Time (s)');
ylabel('y position (m)');
legend('Sensor 1', 'Sensor 2', 'True y', 'Estimated y');
title('Sensor measurements vs Kalman filter estimate for y');
grid on