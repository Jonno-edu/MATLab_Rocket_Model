% --- Test script for 3D Acceleration Transformations ---
clc; clear; close all;

%% --- 1. INPUTS: Define the "Ground Truth" ---

% Vehicle Attitude (in radians)
phi   = deg2rad(179);          % Roll
theta = deg2rad(150);       % Pitch (30 degrees)
psi   = deg2rad(-40);          % Yaw

% Vehicle's Actual Inertial Acceleration (in the body frame)
% For a vehicle at rest, this is [0, 0, 0]
% For a vehicle accelerating purely forward, this is [accel, 0, 0]
actual_a_body = [50; 10; 800]; % m/s^2

% --- Constants ---
g = 9.81;

%% --- 2. FORWARD SIMULATION: From Truth to Sensor Reading ---

fprintf('--- Ground Truth ---\n');
fprintf('Attitude (deg): Roll=%.1f, Pitch=%.1f, Yaw=%.1f\n', rad2deg(phi), rad2deg(theta), rad2deg(psi));
fprintf('Actual Body Acceleration [ax, ay, az]: [%.2f, %.2f, %.2f]\n\n', actual_a_body(1), actual_a_body(2), actual_a_body(3));

% First, we need to know the gravity vector in the body frame.
% To do this, we rotate the navigation gravity vector (g_n) into the body frame.
% We need the rotation matrix from navigation to body (C_bn).
c_ph = cos(phi);   s_ph = sin(phi);
c_th = cos(theta); s_th = sin(theta);
c_ps = cos(psi);   s_ps = sin(psi);

% C_nb rotates from BODY to NAV
C_nb = [c_ps*c_th,  c_ps*s_th*s_ph - s_ps*c_ph,  c_ps*s_th*c_ph + s_ps*s_ph;
        s_ps*c_th,  s_ps*s_th*s_ph + c_ps*c_ph,  s_ps*s_th*c_ph - c_ps*s_ph;
        -s_th,                  c_th*s_ph,                  c_th*c_ph];
        
% The gravity vector in the navigation frame (NED) is simple:
g_n = [0; 0; g];

% The gravity vector as felt in the body frame is:
g_b = C_nb' * g_n; % Note the transpose C_nb' = C_bn

% The definition of specific force is f_b = g_b - a_actual
f_b = g_b - actual_a_body;

fprintf('--- Simulated Sensor Output ---\n');
fprintf('Raw Specific Force (Accelerometer Reading) [fx, fy, fz]: [%.2f, %.2f, %.2f]\n\n', f_b(1), f_b(2), f_b(3));


%% --- 3. REVERSE CALCULATION: From Sensor Reading to NED Acceleration ---

fprintf('--- EKF-Style Calculation ---\n');
% This is what our EKF will do. It takes the sensor reading f_b and the
% current attitude estimate (C_nb) to find the inertial acceleration in
% the navigation frame.

a_n_calculated = -C_nb * f_b + g_n;

fprintf('Final Calculated Inertial Acceleration in NED [a_N; a_E; a_D]:\n');
disp(a_n_calculated);

% Verification: We can also find the true inertial acceleration in the NED
% frame by simply rotating the actual body acceleration.
a_n_truth = C_nb * actual_a_body;
fprintf('Ground Truth Inertial Acceleration in NED for verification:\n');
disp(a_n_truth);

