%% Question 1 - Homogenous Transformation
% by Max Black, xgv2zx

function [transformedPoints] = HomogenousTransform(points, x, y, theta)
% Define rotation matrix
T = [cos(theta) -sin(theta) x;
    sin(theta) cos(theta) y;
    0 0 1];
homogeneousPoints = [points'; ones(1, size(points, 1))]; % dealing with the 4 corners of the robot, 3x4 matrix
transformedHomogeneousPoints = T * homogeneousPoints;
transformedPoints = transformedHomogeneousPoints(1:2, :)';
end