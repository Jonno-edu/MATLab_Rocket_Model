clc; clear; close all;
%% 1. Simulation Setup
% Constants & Time
g = 9.81;              % m/s^2
dt = 0.01;             % 100 Hz
t_pad = 100;
t_end = 100 + t_pad;
N = round(t_end / dt);
t = (0:N-1)' * dt;
% True Vehicle Motion Profile (pitch-only)
true_pitch_rate = zeros(N, 1);
% Set angular rate to 5 deg/s for the first 5 seconds
true_pitch_rate(t > 0 & t <= 5) = deg2rad(9);
true_pitch = cumsum(true_pitch_rate) * dt;
% True total inertial acceleration (NED: North, Down)
true_accel_n = zeros(N, 1);
true_accel_d = zeros(N, 1);
true_accel_n(t > 15 & t <= 50) = 5;
true_accel_n(t > 70 & t <= 80) = -2;
true_accel_d(t > 25 & t <= 30) = -9.81*4;
true_accel_d(t > 30 & t <= 100) = 9.81;
true_vel_n = cumsum(true_accel_n) * dt;
true_vel_d = cumsum(true_accel_d) * dt;
true_pos_n = cumsum(true_vel_n) * dt;
true_pos_d = cumsum(true_vel_d) * dt;
%% 2. Generate Physically Consistent Sensor Data with BIAS
% IMU Noise & Bias Levels
gyro_noise_std  = deg2rad(0.5);
gyro_bias_true  = deg2rad(0.0);   % Constant bias
accel_noise_std = 0.05;
accel_bias_true = 0.02;           % Constant bias
% Specific force in body (pitch-only, N-D coupling)
% This is the correct formulation: a_body = C_bn * (a_nav - g_nav)
perfect_accel_data_x =  true_accel_n.*cos(true_pitch) + (true_accel_d - g).*sin(true_pitch);
perfect_accel_data_z = -true_accel_n.*sin(true_pitch) + (true_accel_d - g).*cos(true_pitch);
perfect_gyro_data    = true_pitch_rate;
% Add noise & bias
accel_data_x = perfect_accel_data_x + randn(N, 1) * accel_noise_std + accel_bias_true;
accel_data_z = perfect_accel_data_z + randn(N, 1) * accel_noise_std + accel_bias_true;
gyro_data    = perfect_gyro_data    + randn(N, 1) * gyro_noise_std    + gyro_bias_true;
% Plot: Raw Gyro X-axis Data and Angle Dead Reckoning
figure('Name','Gyro X-axis (Pitch Rate) and Dead Reckoned Angle');
tiledlayout(2,1);
% 1. Gyro Data (Pitch Rate)
nexttile;
plot(t, rad2deg(gyro_data), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Gyro X [deg/s]');
title('Raw Gyro X-axis (Pitch Rate) with Bias/Noise');
grid on;
% 2. Angle Dead Reckoning
angle_gyro_deadreck = cumsum(gyro_data) * dt;
nexttile;
plot(t, rad2deg(angle_gyro_deadreck), 'r-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Angle [deg]');
title('Angle Dead Reckoned from Raw Gyro');
grid on;

