clc; clear; close all;

%% 1. Simulation Setup
% Constants & Time
g = 9.81; % m/s^2
dt = 0.01; % Simulation time step (100 Hz)
t_end = 30;
N = t_end / dt;
t = (0:N-1)' * dt;

% True Vehicle Motion Profile
true_pitch_rate = zeros(N, 1);
true_pitch_rate(t > 2 & t <= 7) = deg2rad(2);
true_pitch_rate(t > 7 & t <= 12) = deg2rad(-1);
true_pitch = cumsum(true_pitch_rate) * dt;


% Integrate total acceleration to get true velocity and position
true_accel_e = zeros(N, 1);
true_accel_d = zeros(N, 1);
true_accel_e(t > 2 & t <= 20) = 1;
true_accel_e(t > 20 & t <= 28) = -2;
true_accel_d(t > 2 & t <= 20) = -9.81*7;
true_accel_d(t > 20 & t <= 28) = 9.81;

true_vel_e = cumsum(true_accel_e) * dt;
true_vel_d = cumsum(true_accel_d) * dt;
true_pos_e = cumsum(true_vel_e) * dt;
true_pos_d = cumsum(true_vel_d) * dt;

%% 2. Generate Physically Consistent Sensor Data (WITH NOISE)
% Define noise levels
gyro_noise_std = deg2rad(0.1); % Gyro noise (rad/s)
accel_noise_std = 0.05;       % Accelerometer noise (m/s^2)

% Generate noisy sensor data
perfect_accel_data_x = true_accel_e .* cos(true_pitch) + true_accel_d .* sin(true_pitch) - g .* sin(true_pitch);
perfect_accel_data_z = true_accel_e .* sin(true_pitch) + true_accel_d .* cos(true_pitch) - g .* cos(true_pitch);
perfect_gyro_data = true_pitch_rate;

accel_data_x = perfect_accel_data_x + randn(N, 1) * accel_noise_std;
accel_data_z = perfect_accel_data_z + randn(N, 1) * accel_noise_std;
gyro_data = perfect_gyro_data + randn(N, 1) * gyro_noise_std;

% GPS & Absolute Angle (Low Rate) - Not used in this test
gps_pos_e_data = nan(N, 1); gps_pos_d_data = nan(N, 1);
gps_vel_e_data = nan(N, 1); gps_vel_d_data = nan(N, 1);
gps_pitch_data = nan(N, 1);

%% 3. EKF Initialization
x_hat = zeros(5, 1);
P = eye(5) * 0.01;

% Process Noise (Q) must be non-zero for the filter to trust its model less
sigma_a = 0.1; % Uncertainty in our acceleration model
sigma_w = deg2rad(0.05); % Uncertainty in our gyro model
Q_pv = [[dt^4/4, dt^3/2]; [dt^3/2, dt^2]] * sigma_a^2;
Q = zeros(5);
Q(1:2, 1:2) = Q_pv;
Q(3:4, 3:4) = Q_pv;
Q(5,5) = (dt * sigma_w)^2;

x_hat_hist = zeros(N, 5);
x_hat_hist(1, :) = x_hat';

%% 4. EKF Main Loop (Prediction Only)
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
    g_body_x = g * st;
    g_body_z = g * ct;
    a_kin_body_x = a_x_meas + g_body_x;
    a_kin_body_z = a_z_meas + g_body_z;

    % Rotate kinematic acceleration to the INERTIAL frame
    a_kin_inertial_e = a_kin_body_x * ct - a_kin_body_z * st;
    a_kin_inertial_d = a_kin_body_x * st + a_kin_body_z * ct;

    % Add inertial gravity to get TOTAL acceleration
    a_total_e = a_kin_inertial_e;
    a_total_d = a_kin_inertial_d + g;

    % Predict state by integrating total acceleration
    x_hat(1) = p_e_prev + v_e_prev * dt + 0.5 * a_total_e * dt^2;
    x_hat(2) = p_d_prev + v_d_prev * dt + 0.5 * a_total_d * dt^2;
    x_hat(3) = v_e_prev + a_total_e * dt;
    x_hat(4) = v_d_prev + a_total_d * dt;
    x_hat(5) = theta_prev + w_meas * dt;
    
    % --- Jacobian Calculation for Covariance Prediction ---
    F = eye(5);
    F(1, 3) = dt; F(2, 4) = dt;
    d_ax_d_th = -st * a_x_meas - ct * a_z_meas;
    d_az_d_th =  ct * a_x_meas - st * a_z_meas;
    F(1,5) = 0.5 * dt^2 * d_ax_d_th;
    F(2,5) = 0.5 * dt^2 * d_az_d_th;
    F(3,5) = dt * d_ax_d_th;
    F(4,5) = dt * d_az_d_th;

    % Predict covariance
    P = F * P * F' + Q;

    % --- II. Correction Step (Disabled) ---
    if 1 == 2 % Disabled
    end
    
    x_hat_hist(k, :) = x_hat';
end

%% 5. Plot Results
figure('Name', 'EKF Drift with Noisy Sensors (No GPS Correction)');
subplot(3,2,1); hold on; grid on; plot(t, true_pos_e, 'g', 'LineWidth', 2); plot(t, x_hat_hist(:,1), 'b'); legend('True', 'EKF Prediction'); title('East Position');
subplot(3,2,2); hold on; grid on; plot(t, true_pos_d, 'g', 'LineWidth', 2); plot(t, x_hat_hist(:,2), 'b'); legend('True', 'EKF Prediction'); title('Down Position');
subplot(3,2,3); hold on; grid on; plot(t, true_vel_e, 'g', 'LineWidth', 2); plot(t, x_hat_hist(:,3), 'b'); legend('True', 'EKF Prediction'); title('East Velocity');
subplot(3,2,4); hold on; grid on; plot(t, true_vel_d, 'g', 'LineWidth', 2); plot(t, x_hat_hist(:,4), 'b'); legend('True', 'EKF Prediction'); title('Down Velocity');
subplot(3,2,5); hold on; grid on; plot(t, rad2deg(true_pitch), 'g', 'LineWidth', 2); plot(t, rad2deg(x_hat_hist(:,5)), 'b'); legend('True', 'EKF Prediction'); title('Pitch Angle (deg)');
subplot(3,2,6); hold on; grid on; plot(t, x_hat_hist(:,2) - true_pos_d, 'r'); title('Position Error (Down)'); ylabel('Error (m)');
