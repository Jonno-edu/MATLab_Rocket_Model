%% Full 3-Axis Attitude Estimation Using GPS-Derived Accel and Magnetometer
% This script combines the GPS-based acceleration method with a magnetometer
% measurement to achieve a full 3-axis attitude solution using the TRIAD algorithm.

clear; clc; close all;

%% 1. Define the "Ground Truth" Scenario
true_roll  = 0;  % degrees
true_pitch = 10;  % degrees
true_yaw   = 20;  % degrees
g0 = 9.80665;

fprintf('--- Ground Truth ---\n');
fprintf('True Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg\n\n', true_roll, true_pitch, true_yaw);

% Define the TRUE rotation matrix that converts NED vectors to Body vectors
R_ned_to_body_true = eul2rotm(deg2rad([true_yaw, true_pitch, true_roll]), 'ZYX');

% Define the rocket's true inertial acceleration (thrust) in the NED frame.
a_inertial_NED_true = [20 * g0 * cosd(15); 20 * g0 * sind(5); -20 * g0 * sind(15)];

% Define the Earth's magnetic field vector in the NED frame (points North).
mag_field_NED = [27; 0; 0]; % [microTesla]

%% 2. Simulate the Measurements

% a) Simulate Noisy GNSS Measurement of Inertial Acceleration
accel_noise_std_dev = 1; % m/s^2
accel_noise = accel_noise_std_dev * randn(3, 1);
a_inertial_NED_measured = a_inertial_NED_true + accel_noise;

% b) Simulate Noisy Magnetometer Measurement in the Body Frame
%    First find the true body-frame magnetic field, then add noise.
mag_field_body_true = R_ned_to_body_true * mag_field_NED;
mag_noise_std_dev = 0.1; % microTesla
mag_noise = mag_noise_std_dev * randn(3, 1);
mag_field_body_measured = mag_field_body_true + mag_noise;

% c) Simulate the "Perfect" Onboard Accelerometer Measurement
%    The accelerometer feels the combination of true inertial accel and gravity,
%    rotated into the body frame.
a_gravity_NED = [0; 0; g0];
a_sensed_NED_true = a_inertial_NED_true - a_gravity_NED;
a_sensed_body_measured = R_ned_to_body_true * a_sensed_NED_true; % Assuming no accel noise for this sensor

%% 3. Construct Reference and Observation Vectors for TRIAD

% --- Reference Vectors (in the NED frame) ---
% Vector 1: The 'sensed acceleration' constructed from noisy GPS and gravity model
ref_vector_1_NED = a_inertial_NED_measured - a_gravity_NED;
% Vector 2: The known magnetic field model
ref_vector_2_NED = mag_field_NED;

% --- Observation Vectors (in the Body frame) ---
% Vector 1: The direct reading from the onboard accelerometer
obs_vector_1_body = a_sensed_body_measured;
% Vector 2: The noisy reading from the onboard magnetometer
obs_vector_2_body = mag_field_body_measured;

fprintf('--- Constructed Vectors for TRIAD ---\n');
fprintf('Ref Accel (NED): [%.2f, %.2f, %.2f]\n', ref_vector_1_NED);
fprintf('Obs Accel (Body):[%.2f, %.2f, %.2f]\n', obs_vector_1_body);
fprintf('Ref Mag (NED):   [%.2f, %.2f, %.2f]\n', ref_vector_2_NED);
fprintf('Obs Mag (Body):  [%.2f, %.2f, %.2f]\n\n', obs_vector_2_body);

%% 4. Implement TRIAD Algorithm to Find Full 3-Axis Attitude

% Normalize the primary vectors (accelerations are usually more reliable)
r1 = ref_vector_1_NED / norm(ref_vector_1_NED);
b1 = obs_vector_1_body / norm(obs_vector_1_body);

% Normalize the secondary vectors (magnetometer readings)
r2 = ref_vector_2_NED / norm(ref_vector_2_NED);
b2 = obs_vector_2_body / norm(obs_vector_2_body);

% --- Create the TRIAD coordinate systems ---
t1_ref = r1;
t1_obs = b1;
t2_ref = cross(r1, r2) / norm(cross(r1, r2));
t2_obs = cross(b1, b2) / norm(cross(b1, b2));
t3_ref = cross(t1_ref, t2_ref);
t3_obs = cross(t1_obs, t2_obs);

% Construct the attitude matrix for each frame
M_ref = [t1_ref'; t2_ref'; t3_ref'];
M_obs = [t1_obs'; t2_obs'; t3_obs'];

% Calculate the DCM that rotates from the Body frame to the NED frame
R_est_body_to_ned = M_ref' * M_obs;

%% 5. Display the Results

% Convert the estimated rotation matrix back to Euler angles
eul_est_rad = rotm2eul(R_est_body_to_ned', 'ZYX');
est_yaw   = rad2deg(eul_est_rad(1));
est_pitch = rad2deg(eul_est_rad(2));
est_roll  = rad2deg(eul_est_rad(3));

fprintf('--- TRIAD Algorithm 3-Axis Results ---\n');
fprintf('Estimated Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg\n', est_roll, est_pitch, est_yaw);

% Calculate the error
error_roll  = est_roll - true_roll;
error_pitch = est_pitch - true_pitch;
error_yaw   = est_yaw - true_yaw;

fprintf('Estimation Error: Roll: %.4f deg, Pitch: %.4f deg, Yaw: %.4f deg\n', error_roll, error_pitch, error_yaw);
