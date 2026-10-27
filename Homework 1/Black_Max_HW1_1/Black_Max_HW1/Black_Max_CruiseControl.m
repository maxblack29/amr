%% Question 3 - Cruise Controller
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

% Initialize a vector to save velocity info and time
v = [];
t = [];
t(1) = 0;

% Robot initial config
x(1) = 100;
y(1) = 100; 
theta(1) = rand()*2*pi; 

% Go to goal setup
kv = 0.2;
kp = 2;
tolerance = 0.5; % used to compare with distance of robot to goal
goals = [rand()*200 rand()*200;
    rand()*200 rand()*200;
    rand()*200 rand()*200]; % 3 random goals matrix
dt = 0.1; % recycled for PID controller
i = 1; % index to build x(i+1), y(i+1), theta(i+1)

% PID controller constants
vref = 3;
kp_pid = 1;
ki = 0.15;
kd = 0.1;
prev_error = 0;
integral = 0;
v(1) = 0; % initial velocity is 0
output = 0; % used to start the loop

% Build Robot Model and show where the goals are
robot = SquareRobot(x,y,theta(1));
fill(robot(:,1), robot(:,2), 'b');
hold on
plot(goals(:,1), goals(:,2), 'r*');
xlim([0 200]);
ylim([0 200]);

for j=1:3
    cur_goal = goals(j,:); %identify first goal and the xg and yg
    xg=cur_goal(1);
    yg=cur_goal(2);
    dist = hypot(xg-x(1), yg-y(1)); % initial distance
    while (dist > tolerance)
        dist = hypot(xg - x(i), yg - y(i));
        thetad = atan2(yg - y(i), xg - x(i));
        gamma = kp*(thetad-theta(i));

        % PID controller
        error = vref - v(i);
        integral = integral + error*dt;
        derivative = (error - prev_error)/dt;
        output = output + kp_pid*error + ki*integral + kd*derivative;
        prev_error = error;
        v(i+1) = v(i) + output*dt-0.01*v(i);

        x(i+1) = x(i) + v(i) * cos(theta(i)) * dt;
        y(i+1) = y(i) + v(i) * sin(theta(i)) * dt;
        theta(i+1) = theta(i) + gamma * dt;
        t(i+1) = t(i) + 0.1;
        i = i + 1;
    
        robot = SquareRobot(x(i),y(i),theta(i));
        cla
        fill(robot(:,1), robot(:,2), 'b');
        hold on
        plot(x,y,'-');
        plot(xg, yg, 'r*');
        % plot(goals(1,1), goals(1,2), 'r*');
        % plot(goals(2,1), goals(2,2), 'r*');
        % plot(goals(3,1), goals(3,2), 'r*');
        xlim([0 200])
        ylim([0 200])
        pause(0.01)
    end
end

% velocity plot
figure
plot(t, v);
xlabel('time (s)');
ylabel('velocity (m/s)');
title('velocity vs. time');