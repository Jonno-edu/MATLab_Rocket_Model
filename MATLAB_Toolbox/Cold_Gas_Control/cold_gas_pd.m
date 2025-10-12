clc; close all; clear;

% System parameters
T_max = 0.2;
I_roll = 80;

% Design specifications
settling_time = 10;
omega_n = 4 / settling_time;
zeta = 1.0;

% Plant transfer function
s = tf('s');
G_plant = 1 / (I_roll * s^2);

% Weight functions for loop shaping
omega_b = omega_n;
M = 1.5;
A = 1e-4;

% Performance weight (low frequency tracking)
Wp = (s/M + omega_b) / (s + omega_b*A);

% Control weight (limit high frequency gain)
Wu = 0.01 * (s + 100) / (100*s + 1);

% Shaped plant
Gs = G_plant * T_max;

% Augmented plant for mixsyn
P = augw(Gs, Wp, Wu, []);

% H-infinity synthesis
[K_hinf, CL, gamma] = mixsyn(Gs, Wp, Wu, []);

% Closed-loop system
T_cl = feedback(K_hinf * Gs, 1);

% Display controller
fprintf('=== H-INFINITY CONTROLLER ===\n');
fprintf('Controller order: %d\n', order(K_hinf));
fprintf('H-infinity norm (gamma): %.4f\n\n', gamma);

% Print transfer function
disp('Controller K(s):');
K_hinf

% Get state-space representation
[A, B, C, D] = ssdata(K_hinf);
fprintf('\nState-space representation:\n');
fprintf('A matrix (%dx%d):\n', size(A,1), size(A,2));
disp(A);
fprintf('B matrix:\n');
disp(B);
fprintf('C matrix:\n');
disp(C);
fprintf('D matrix:\n');
disp(D);

% Get numerator and denominator
[num, den] = tfdata(K_hinf, 'v');
fprintf('\nTransfer function form:\n');
fprintf('Numerator coefficients:\n');
disp(num);
fprintf('Denominator coefficients:\n');
disp(den);

