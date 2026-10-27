%% Question 4 - follow a wall
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
d=[];
dw = 10; % desired distance
t = []; % time array
t(1) = 0; 

% Robot initial config
x(1) = 100;
y(1) = 100;
theta(1) = rand()*2*pi;

% Velocity 
v = 5;
dt = 0.1;
nstep = 1000;

% Line is ax+by+c=0
a = 0;
b = 1;
c = -50;
d(1) = (a*x(1)+b*y(1)+c)/hypot(a, b) - dw;

% K gains
kt = 0.05; % 0.05 is tuned
kh = 1; % 1 is tuned

thetad = atan2(-a, b);

% Build Robot Model and draw the line
robot = SquareRobot(x,y,theta(1));
fill(robot(:,1), robot(:,2), 'b');
hold on
plot([0 200], [50 50], 'r--');
xlim([0 200]);
ylim([0 200]);

for n = 1:nstep
    d(n) = (a*x(n)+b*y(n)+c)/hypot(a, b) - dw;
    alpha_t = -kt*d(n);
    alpha_h = kh*(thetad-theta(n));
    gamma = alpha_t + alpha_h;
    x(n+1) = x(n) + v*cos(theta(n))*dt;
    y(n+1) = y(n) + v*sin(theta(n))*dt;
    theta(n+1) = theta(n) + gamma*dt;
    t(n+1) = t(n) + 0.1;

    robot = SquareRobot(x(n),y(n),theta(n));
    cla
    fill(robot(:,1), robot(:,2), 'b');
    hold on
    plot([0 200], [50 50], 'r--');
    plot(x,y,'-');
    xlim([0 200])
    ylim([0 200])

    if x(n) >= 200
        break % just so code doesn't run longer than it has to
    end
    pause(0.01)
end

figure
plot(t(1:length(d)), d+dw);
xlabel('time (s)')
ylabel('distance between the robot and the line (m)')
title('Relative distance between robot and line over time')