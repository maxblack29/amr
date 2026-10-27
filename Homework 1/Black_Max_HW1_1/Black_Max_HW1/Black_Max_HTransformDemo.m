%% Move Robot Sample Code

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Nicola Bezzo (UVA)
% AMR 2026 
% Date: 09/09/2026
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all
clc

%% Parameters

% Workspace Size
xlim([0 200])
ylim([0 200])

%Velocity (constant for this demo example) 
vel = 5;

%Steering angle
steering = pi/4; 

%Initialize a vector of positions for the robot
x=[]; 
y=[];

% Robot Initial Pose

x(1) = 100;
y(1) = 100;

% Initial Orientation 
theta(1) = rand()*2*pi; 

% Build Robot Model
robot = SquareRobot(x,y,theta(1));

fill (robot(:,1), robot(:,2), 'b');
xlim([0 200])
ylim([0 200])

%% Question 1: Homogeneous Transformation Test

% corners of the square
[~, rawCorners] = SquareRobot(x(1), y(1), theta(1));

%initial configuration with current pose
initialConfig = HomogenousTransform(rawCorners, x(1), y(1), theta(1)); 

%final configuration after transform
xFinal = x(1) + 20;
yFinal = y(1) + 20;
thetaFinal = theta(1) + pi/6;

finalConfig = HomogenousTransform(rawCorners, xFinal, yFinal, thetaFinal);

figure
fill(initialConfig(:,1), initialConfig(:,2), 'b');
hold on
fill(finalConfig(:,1), finalConfig(:,2), 'r');
legend('Initial Configuration', 'Final Configuration');
xlim([0 200]);
ylim([0 200]);
hold off