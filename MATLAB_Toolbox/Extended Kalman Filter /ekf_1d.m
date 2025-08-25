clc; clear; close all;

%% 1. Simulation Setup
% Constants & Time
g = 9.81; % m/s^2
dt = 0.001; % Simulation time step (100 Hz)
t_end = 100;
N = t_end / dt;
t = (0:N-1)' * dt;

% True Vehicle Motion Profile
true_pitch_rate = zeros(N, 1);
true_pitch_rate(t > 20 & t <= 50) = deg2rad(2);
true_pitch_rate(t > 50 & t <= 80) = deg2rad(-1);
true_pitch = cumsum(true_pitch_rate) * dt;

% True total inertial acceleration 
true_accel_e = zeros(N, 1);
true_accel_d = zeros(N, 1);
true_accel_e(t > 15 & t <= 50) = 1;
true_accel_e(t > 70 & t <= 80) = -2;
true_accel_d(t > 15 & t <= 70) = -9.81*2; % Adjusted for better visualization
true_accel_d(t > 70 & t <= 100) = 9.81;

% Integrate to get true velocity and position
true_vel_e = cumsum(true_accel_e) * dt;
true_vel_d = cumsum(true_accel_d) * dt;
true_pos_e = cumsum(true_vel_e) * dt;
true_pos_d = cumsum(true_vel_d) * dt;

%% 2. Generate Physically Consistent Sensor Data 
% IMU Noise Levels
gyro_noise_std = deg2rad(0.5); 
gyro_noise_offset = deg2rad(0.1);

accel_noise_std = 0.05;
accel_noise_offset = 0.01; 

% Generate IMU Data
perfect_accel_data_x = true_accel_e .* cos(true_pitch) + true_accel_d .* sin(true_pitch) - g .* sin(true_pitch);
perfect_accel_data_z = -true_accel_e .* sin(true_pitch) + true_accel_d .* cos(true_pitch) - g .* cos(true_pitch);
perfect_gyro_data = true_pitch_rate;

accel_data_x = perfect_accel_data_x + randn(N, 1) * accel_noise_std + accel_noise_offset;
accel_data_z = perfect_accel_data_z + randn(N, 1) * accel_noise_std + accel_noise_offset;
gyro_data = perfect_gyro_data + randn(N, 1) * gyro_noise_std + gyro_noise_offset;

% GPS & Absolute Angle (Low Rate)
gps_rate = 20; % Hz
gps_pos_noise_std = 2.5;
gps_vel_noise_std = 0.1;
gps_pitch_noise_std = deg2rad(0.1);

gps_pos_e_data = nan(N, 1); gps_pos_d_data = nan(N, 1);
gps_vel_e_data = nan(N, 1); gps_vel_d_data = nan(N, 1);
gps_pitch_data = nan(N, 1);

for k = 1:N
    if mod(k - 1, round(1/dt / gps_rate)) == 0
        gps_pos_e_data(k) = true_pos_e(k) + randn() * gps_pos_noise_std;
        gps_pos_d_data(k) = true_pos_d(k) + randn() * gps_pos_noise_std;
        gps_vel_e_data(k) = true_vel_e(k) + randn() * gps_vel_noise_std;
        gps_vel_d_data(k) = true_vel_d(k) + randn() * gps_vel_noise_std;
        gps_pitch_data(k) = true_pitch(k) + randn() * gps_pitch_noise_std;
    end
end

%% 3. EKF Initialization
x_hat = zeros(5, 1);
P = eye(5) * 1; % Initial covariance

% Process Noise (Q)
sigma_a = 0.1; 
sigma_w = deg2rad(0.05);
Q_pv = [[dt^4/4, dt^3/2]; [dt^3/2, dt^2]] * sigma_a^2;
Q = zeros(5);
Q(1:2, 1:2) = Q_pv;
Q(3:4, 3:4) = Q_pv;
Q(5,5) = (dt * sigma_w)^2;

% Measurement Noise (R)
R = diag([gps_pos_noise_std^2, gps_pos_noise_std^2, ...
          gps_vel_noise_std^2, gps_vel_noise_std^2, ...
          gps_pitch_noise_std^2]);
          
% Measurement Matrix (H)
H = eye(5);

x_hat_hist = zeros(N, 5);
x_hat_hist(1, :) = x_hat';

%% 4. EKF Main Loop
for k = 2:N
    % --- I. Prediction Step ---
    p_e_prev = x_hat(1); p_d_prev = x_hat(2);
    v_e_prev = x_hat(3); v_d_prev = x_hat(4);
    theta_prev = x_hat(5);
    
    a_x_meas = accel_data_x(k-1);
    a_z_meas = accel_data_z(k-1);
    w_meas = gyro_data(k-1);
    
    theta_mid = theta_prev + w_meas * dt / 2;
    ct = cos(theta_mid);
    st = sin(theta_mid);
    
    % Reconstruct kinematic acceleration in the BODY frame
    g_body_x = g * st; g_body_z = g * ct;
    a_kin_body_x = a_x_meas + g_body_x;
    a_kin_body_z = a_z_meas + g_body_z;

    % Rotate kinematic acceleration to the INERTIAL frame
    a_kin_inertial_e = a_kin_body_x * ct - a_kin_body_z * st;
    a_kin_inertial_d = a_kin_body_x * st + a_kin_body_z * ct;

    % Add inertial gravity to get TOTAL acceleration
    a_total_e = a_kin_inertial_e;
    a_total_d = a_kin_inertial_d + g;

    % Predict state
    x_hat(1) = p_e_prev + v_e_prev * dt + 0.5 * a_total_e * dt^2;
    x_hat(2) = p_d_prev + v_d_prev * dt + 0.5 * a_total_d * dt^2;
    x_hat(3) = v_e_prev + a_total_e * dt;
    x_hat(4) = v_d_prev + a_total_d * dt;
    x_hat(5) = theta_prev + w_meas * dt;
    x_hat(5) = atan2(sin(x_hat(5)), cos(x_hat(5))); % Normalize angle
    
    % Predict covariance
    F = eye(5);
    F(1, 3) = dt; F(2, 4) = dt;
    F(1,5) = 0.5 * dt^2 * (-st * a_x_meas - ct * a_z_meas);
    F(2,5) = 0.5 * dt^2 * ( ct * a_x_meas - st * a_z_meas);
    F(3,5) = dt * (-st * a_x_meas - ct * a_z_meas);
    F(4,5) = dt * ( ct * a_x_meas - st * a_z_meas);
    P = F * P * F' + Q;
    
    % --- II. Correction Step ---
    if ~isnan(gps_pos_e_data(k))
        % Form measurement vector
        z = [gps_pos_e_data(k); gps_pos_d_data(k); ...
             gps_vel_e_data(k); gps_vel_d_data(k); ...
             gps_pitch_data(k)];
        
        % Calculate measurement residual (innovation)
        y = z - H * x_hat;
        y(5) = atan2(sin(y(5)), cos(y(5))); % Normalize angle residual

        % Calculate innovation covariance and Kalman Gain
        S = H * P * H' + R;
        K = P * H' / S;
        
        % Update state estimate and covariance
        x_hat = x_hat + K * y;
        P = (eye(5) - K * H) * P;
        
        % Normalize angle again after update
        x_hat(5) = atan2(sin(x_hat(5)), cos(x_hat(5)));
    end
    
    x_hat_hist(k, :) = x_hat';
end

%% 5. Plot Results
% Figure 1: East Position
figure('Name', 'East Position Analysis');
subplot(2,1,1);
hold on; grid on;
plot(t, true_pos_e, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,1), 'b--');
plot(t, gps_pos_e_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF Estimate', 'GPS');
title('East Position Comparison');
ylabel('Position (m)');
subplot(2,1,2);
hold on; grid on;
plot(t, x_hat_hist(:,1) - true_pos_e, 'r');
title('East Position Error');
ylabel('Error (m)'); xlabel('Time (s)');

% Figure 2: Down Position
figure('Name', 'Down Position Analysis');
subplot(2,1,1);
hold on; grid on;
plot(t, true_pos_d, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,2), 'b--');
plot(t, gps_pos_d_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF Estimate', 'GPS');
title('Down Position Comparison');
ylabel('Position (m)');
subplot(2,1,2);
hold on; grid on;
plot(t, x_hat_hist(:,2) - true_pos_d, 'r');
title('Down Position Error');
ylabel('Error (m)'); xlabel('Time (s)');

% Figure 3: East Velocity
figure('Name', 'East Velocity Analysis');
subplot(2,1,1);
hold on; grid on;
plot(t, true_vel_e, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,3), 'b--');
plot(t, gps_vel_e_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF Estimate', 'GPS');
title('East Velocity Comparison');
ylabel('Velocity (m/s)');
subplot(2,1,2);
hold on; grid on;
plot(t, x_hat_hist(:,3) - true_vel_e, 'r');
title('East Velocity Error');
ylabel('Error (m/s)'); xlabel('Time (s)');

% Figure 4: Down Velocity
figure('Name', 'Down Velocity Analysis');
subplot(2,1,1);
hold on; grid on;
plot(t, true_vel_d, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,4), 'b--');
plot(t, gps_vel_d_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF Estimate', 'GPS');
title('Down Velocity Comparison');
ylabel('Velocity (m/s)');
subplot(2,1,2);
hold on; grid on;
plot(t, x_hat_hist(:,4) - true_vel_d, 'r');
title('Down Velocity Error');
ylabel('Error (m/s)'); xlabel('Time (s)');

% Figure 5: Pitch Angle
figure('Name', 'Pitch Angle Analysis');
subplot(2,1,1);
hold on; grid on;
plot(t, rad2deg(true_pitch), 'g', 'LineWidth', 2);
plot(t, rad2deg(x_hat_hist(:,5)), 'b--');
plot(t, rad2deg(gps_pitch_data), 'ro', 'MarkerSize', 4);
legend('True', 'EKF Estimate', 'GPS');
title('Pitch Angle Comparison');
ylabel('Angle (deg)');
subplot(2,1,2);
hold on; grid on;
plot(t, rad2deg(x_hat_hist(:,5) - true_pitch), 'r');
title('Pitch Angle Error');
ylabel('Error (deg)'); xlabel('Time (s)');
