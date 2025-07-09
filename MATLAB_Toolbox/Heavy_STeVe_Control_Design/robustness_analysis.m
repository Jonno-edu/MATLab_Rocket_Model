% Robustness Analysis of LQR Controller
% This script analyzes the stability margins of a single-point LQR design
% using frequency-domain techniques (Nyquist and Bode plots).
clear; clc; close all;

%% 1. Define Plant Model at a Single Flight Point (t=66s)
% Physical parameters
m = 1218.1;             % rocket mass (kg)
rho = 0.4;              % air density (kg/m³)
V = 397.9;              % velocity (m/s)
S = 0.200296;           % reference area (m²)
L = 9.542;              % reference length (m)
I = 2.29e+04;           % moment of inertia (kg⋅m²)
T = 2.67e+04;           % thrust (N)
L_arm = 3.9;            % TVC moment arm (m)

% Aerodynamic coefficients
CNa = 3.5;              % Normal force derivative (1/rad)
Cma = 29.661;           % Pitching moment derivative (1/rad) - unstable
Cmq = -7.01;            % Pitch damping derivative (1/rad)

% Actuator model (states are [position; rate])
natural_frequency = 62;
damping_ratio = 0.5858;
A_act = [0 1; -natural_frequency^2 -2*damping_ratio*natural_frequency];
B_act = [0; natural_frequency^2]; % CORRECTED: Used natural_frequency directly
C_act = [1 0];

%% 2. Construct the Combined Open-Loop State-Space System
% This is the 5-state model (airframe + actuator) for which the
% controller is designed.

% Calculate dimensional stability derivatives
q_bar = 0.5 * rho * V^2;
Z_alpha = (q_bar * S / m) * CNa + (T / m);
Z_delta = T / m;
M_alpha = (q_bar * S * L / I) * Cma;
M_q = (q_bar * S * L^2) / (2 * V * I) * Cmq;
M_delta = (T * L_arm) / I;

% Airframe state-space model (3 states)
% States: [alpha; q; theta]
A_plant = [-Z_alpha/V,  1,   0;
            M_alpha,    M_q, 0;
            0,          1,   0];
B_plant = [-Z_delta/V;
            M_delta;
            0];

% Combined open-loop system model (5 states)
% States: [alpha; q; theta; act_pos; act_rate]
A_ol = [[A_plant, B_plant*C_act]; [zeros(2,3), A_act]];
B_ol = [zeros(3,1); B_act]; % Input affects actuator directly
C_ol = eye(5);
D_ol = zeros(5,1);
sys_open_loop = ss(A_ol, B_ol, C_ol, D_ol);


%% 3. Design the Nominal LQR Controller
% LQR Tuning Weights
max_alpha       = 5 * pi/180;
max_q           = 0.1 * pi/180;
max_theta       = 5 * pi/180;
max_act_pos     = 1 * pi/180;
max_act_rate    = 10 * pi/180;

max_TVC_command = 2 * pi/180;

% Construct Q and R matrices
Q_diag = [1/max_alpha^2, 1/max_q^2, 1/max_theta^2, 1/max_act_pos^2, 1/max_act_rate^2];
Q = diag(Q_diag);
R = 1/max_TVC_command^2;

% Calculate the LQR gain matrix K
K_lqr = lqr(A_ol, B_ol, Q, R);

fprintf('--- LQR Controller Design Complete ---\n');
disp('LQR Gain Matrix K:');
disp(K_lqr);


%% 4. Robustness Analysis
% We analyze the robustness by breaking the loop at the plant input (u).
% The loop transfer function is L(s) = K * (sI - A)^-1 * B.
% This represents the transfer function from the control input 'u' to the
% feedback signal 'K*x' that would be subtracted from the command.

fprintf('\n--- Performing Robustness Analysis ---\n');

% Form the loop transfer function L(s)
% This is equivalent to connecting the output of the open-loop system
% (which is the full state vector) to the LQR gain matrix K.
sys_loop_transfer = series(sys_open_loop, K_lqr);

% Calculate the stability margins using the 'margin' command.
% For a MIMO system like this, 'margin' computes the disk-based margins,
% which give a single, reliable measure of robustness.
figure;
margin(sys_loop_transfer);
title('Bode Plot of the Loop Transfer Function at Plant Input');
grid on;

% The disk margin is a more reliable MIMO robustness metric.
% It represents the smallest circle centered on -1 that touches the
% Nyquist plot, indicating simultaneous gain/phase changes.
[DM, Freq] = diskmargin(sys_loop_transfer);
fprintf('\n--- Stability Margins ---\n');
fprintf('Disk-Based Gain Margin:   [%.2f, %.2f] (Can tolerate gain change by these factors)\n', DM.GainMargin(1), DM.GainMargin(2));
fprintf('Disk-Based Phase Margin:  +/- %.2f deg\n', DM.PhaseMargin);
fprintf('The system is robust if the gain margin does not include 1.0 and phase margin is large.\n');

% Generate the Nyquist plot for visualization
figure;
nyquist(sys_loop_transfer);
title('Nyquist Plot of the Loop Transfer Function');
grid on;
% For stability, the plot must not encircle the -1 point.

