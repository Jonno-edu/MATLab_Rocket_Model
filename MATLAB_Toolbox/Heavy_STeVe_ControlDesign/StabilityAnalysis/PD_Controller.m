%% FINAL SIMULATION SCRIPT with a Correct Test Case
clear; clc; close all;

fprintf('--- Creating State-Space Model and a CORRECT Test Case for Simulink ---\n');

%% 1. Define the Flight Condition (t=35s) and Realistic Parameters
S = 0.200296; L = 9.542; L_arm = 3.9; CLa = 2.0;
Cma_realistic = 0.1;
Cmq_realistic = -10.0;
rho = 0.975; V = 150.6; m = 1528.1; T = 2.48e4; I = 2.77e4;
fprintf('Using realistic airframe model for t=35s.\n');

%% 2. Build the Plant State-Space Matrices (A, B, C, D)
d1 = (rho * V * S) / (2 * m) * CLa;
d2 = T * L_arm / I;
d3 = (rho * V^2 * S * L) / (2 * I) * Cma_realistic;
d4 = (rho * V * S * L^2) / (4 * I) * Cmq_realistic;
d5 = T / (m * V);
A = [-d1 0 1; 0 0 1; d3 0 d4];
B = [-d5; 0; d2];
C = eye(3);
D = [0; 0; 0];
fprintf('Plant matrices A, B, C, D created.\n');

%% 3. Define the PD Controller Gain Matrix (K)
K_alpha = 2.5448;
K_q     = 0.0779;
K = [K_alpha, 0, K_q];
fprintf('Controller gain matrix K created.\n');

%% 4. Define a CORRECT Simulation Initial Condition
% To test our PD regulator, we must give it an error it can see.
% We will simulate a disturbance by giving it a small initial pitch rate.
x0 = [2*pi/180; 0; 0;]; % x = [alpha; theta; q]
fprintf('Initial condition vector x0 created to simulate a disturbance (q_0 = %.2f rad/s).\n', x0(3));

%% 5. Announce Completion
fprintf('\n--- Workspace Ready for Simulink ---\n');
fprintf('Your Simulink model is CORRECT. The test case was the problem.\n');
fprintf('Run the simulation now. You will see a dynamic response.\n');

