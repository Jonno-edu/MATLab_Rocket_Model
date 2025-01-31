% filepath: /Users/jonno/MATLAB-Drive/Masters/SimpleRocketModel/testRocketModel.m

% Test script for Rocket_model

clear
clc
close all

% Define constants
g = 9.81; % Gravitational acceleration (m/s^2)
m = 1.0; % Mass of the rocket (kg)
l_p = 0.5; % Distance from CoG to thrust point (m)
Ib = m * 40.07; % Simplified inertia for 2D (example value)
invIb = 1 / Ib; % Inverse of inertia

% Define initial state variables
x1 = 0; % Initial velocity in x direction (m/s)
x2 = 0; % Initial velocity in y direction (m/s)
x3 = 0; % Initial angular velocity (rad/s)
x4 = 0; % Initial angular position (rad)
x5 = 0; % Initial angular rate (rad/s)
x8 = 0; % Initial pitch angle (rad)

% Define inputs
u1 = 10; % Thrust force (N)
u2 = 0;

% Combine state variables into a vector
X = [x1; x2; x3; x4; x5; x8];

% Combine inputs into a vector
U = [u1 ; u2];

% Call the Rocket_model function
XDOT = RocketModel(X, U);

% Display the results
disp('State derivatives (XDOT):')
disp(XDOT)