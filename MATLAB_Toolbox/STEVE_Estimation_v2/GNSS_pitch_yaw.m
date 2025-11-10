%% 2-Axis Attitude Estimation Using Optimization (CORRECTED MATLAB SYNTAX)

clear; clc; close all;

%% 1. Define the "Ground Truth" Scenario
true_pitch = 30;  % degrees
true_yaw   = 10;  % degrees
g0 = 9.80665;

fprintf('--- Ground Truth ---\n');
fprintf('True Pitch: %.2f deg, Yaw: %.2f deg\n\n', true_pitch, true_yaw);

% Define the TRUE rotation matrix for pitch and yaw (zero roll)
R_ned_to_body_true = eul2rotm(deg2rad([true_yaw, true_pitch, 0]), 'ZYX');

% Define the rocket's true inertial acceleration in the NED frame.
a_inertial_NED_true = [20 * g0 * cosd(15); 20 * g0 * sind(5); -20 * g0 * sind(15)];

%% 2. Simulate the Measurements (with ZERO noise to prove the logic)
noise_std_dev = 0.11;
noise = noise_std_dev ; %* randn(3, 1);
a_inertial_NED_measured = a_inertial_NED_true + noise;

% Model the gravity vector
a_gravity_NED = [0; 0; g0];

% Construct the "sensed acceleration" reference vector in the NED frame.
a_sensed_NED = a_inertial_NED_measured - a_gravity_NED;

% Simulate the Onboard Accelerometer Measurement (Body Frame)
a_sensed_NED_true = a_inertial_NED_true - a_gravity_NED;
a_sensed_body = R_ned_to_body_true * a_sensed_NED_true;

fprintf('--- Simulated Measurements ---\n');
fprintf('Noisy GNSS Inertial Accel (NED):   [%.2f, %.2f, %.2f]\n', a_inertial_NED_measured);
fprintf('Constructed Sensed Accel (NED):    [%.2f, %.2f, %.2f]\n', a_sensed_NED);
fprintf('Onboard Accelerometer Reading (Body): [%.2f, %.2f, %.2f]\n\n', a_sensed_body);

%% 3. Solve for Pitch and Yaw using Optimization

% Define the error function to be minimized.
% It takes a 2-element vector [yaw, pitch] in radians.
% **CORRECTION IS HERE**: We build a 3-element vector inside the function.
error_func = @(angles) sum((a_sensed_body - eul2rotm([angles(1), angles(2), 0], 'ZYX') * a_sensed_NED).^2);

% Initial guess for the optimizer [yaw, pitch]
initial_guess_rad = [0; 0];

% Use fminsearch to find the angles that minimize the error function
estimated_angles_rad = fminsearch(error_func, initial_guess_rad);

% Extract and convert results
est_yaw   = rad2deg(estimated_angles_rad(1));
est_pitch = rad2deg(estimated_angles_rad(2));

%% 4. Display Results
fprintf('--- 2-Axis Attitude Estimation Results (Optimizer) ---\n');
fprintf('Estimated Pitch: %.2f deg\n', est_pitch);
fprintf('Estimated Yaw:   %.2f deg\n\n', est_yaw);

error_pitch = est_pitch - true_pitch;
error_yaw   = est_yaw - true_yaw;

fprintf('Estimation Error: Pitch: %.4f deg, Yaw: %.4f deg\n', error_pitch, error_yaw);

%% 3. Solve for Pitch and Yaw using Axis-Angle Rotation

% We have a_sensed_NED and a_sensed_body
v_ned  = a_sensed_NED;
v_body = a_sensed_body;

% Normalize the vectors to get unit vectors
u = v_body / norm(v_body);
v = v_ned / norm(v_ned);

% The rotation matrix R should satisfy: v = R * u

% Find the rotation axis and angle
k_axis = cross(u, v);
k_axis = k_axis / norm(k_axis); % The unit vector for the axis of rotation

theta = acos(dot(u, v)); % The angle of rotation

% Create the skew-symmetric cross-product matrix of k
K = [  0,    -k_axis(3),  k_axis(2);
     k_axis(3),   0,    -k_axis(1);
    -k_axis(2), k_axis(1),   0      ];

% Use Rodrigues' rotation formula to find the rotation matrix
% This is the rotation from the Body frame to the NED frame
R_est_body_to_ned = eye(3) + sin(theta) * K + (1 - cos(theta)) * K^2;

% Convert the estimated rotation matrix back to Euler angles
% We need to transpose the result to get the NED-to-Body rotation for rotm2eul
eul_est_rad = rotm2eul(R_est_body_to_ned', 'ZYX');
est_yaw   = rad2deg(eul_est_rad(1));
est_pitch = rad2deg(eul_est_rad(2));
% We are ignoring the roll component from this calculation
% est_roll  = rad2deg(eul_est_rad(3));



%% 4. Display Results
fprintf('--- 2-Axis Attitude Estimation Results (Trig) ---\n');
fprintf('Estimated Pitch: %.2f deg\n', est_pitch);
fprintf('Estimated Yaw:   %.2f deg\n\n', est_yaw);

error_pitch = est_pitch - true_pitch;
error_yaw   = est_yaw - true_yaw;

fprintf('Estimation Error: Pitch: %.4f deg, Yaw: %.4f deg\n', error_pitch, error_yaw);