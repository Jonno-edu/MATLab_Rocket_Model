clc; clear; close all;

%% 1. Simulation Setup
% Constants & Time
g = 9.81;
dt = 0.01;
N = 3000;
t = (0:N-1)' * dt;

% True Vehicle Motion Profile
% Pitch motion
true_pitch_rate = zeros(N, 1);
true_pitch_rate(t > 2 & t <= 7) = deg2rad(10);
true_pitch_rate(t > 7 & t <= 12) = deg2rad(-10);
true_pitch = cumsum(true_pitch_rate) * dt;

% Inertial motion (East, Down)
true_accel_x_body = zeros(N, 1); % Vehicle's forward acceleration
true_accel_x_body(t > 5 & t <= 15) = 2.0;
true_accel_x_body(t > 15 & t <= 20) = -2.0;

% Convert body acceleration to inertial acceleration using true pitch
true_accel_e = true_accel_x_body .* cos(true_pitch);
true_accel_d = true_accel_x_body .* sin(true_pitch) + g; % Add gravity

true_vel_e = cumsum(true_accel_e) * dt;
true_vel_d = cumsum(true_accel_d) * dt;
true_pos_e = cumsum(true_vel_e) * dt;
true_pos_d = cumsum(true_vel_d) * dt;

%% 2. Generate Realistic Noisy Sensor Data
% IMU (High Rate: 100 Hz)
gyro_bias = deg2rad(0.0);
gyro_noise_std = deg2rad(0.1);
gyro_data = true_pitch_rate + gyro_bias + randn(N, 1) * gyro_noise_std;

accel_noise_std = 0.1;
% Accelerometer measures body-frame acceleration (gravity is not included here
% as it's part of the process model, not a direct measurement)
accel_data_x = true_accel_x_body + randn(N, 1) * accel_noise_std;

% GPS & Absolute Angle (Low Rate)
gps_rate = 5;
gps_pos_noise_std = 2.0;
gps_vel_noise_std = 0.5;
gps_pitch_noise_std = deg2rad(2.0);

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
% State Vector: [p_e, p_d, v_e, v_d, theta]
x_hat = zeros(5, 1);
x_hat(5) = gps_pitch_data(find(~isnan(gps_pitch_data), 1)); % Init with first angle measurement

% State Covariance Matrix (P)
P = diag([gps_pos_noise_std^2, gps_pos_noise_std^2, ...
          gps_vel_noise_std^2, gps_vel_noise_std^2, ...
          gps_pitch_noise_std^2]);

% Process Noise Covariance (Q)
sigma_a = 0.5;
sigma_w = deg2rad(0.5);
Q_pv = [[dt^4/4, dt^3/2]; [dt^3/2, dt^2]] * sigma_a^2;
Q_theta = (dt * sigma_w)^2;
Q = zeros(5);
Q(1:2, 1:2) = Q_pv; Q(3:4, 3:4) = Q_pv; Q(5,5) = Q_theta;

% Measurement Covariance (R) and Matrix (H)
R = diag([gps_pos_noise_std^2, gps_pos_noise_std^2, ...
          gps_vel_noise_std^2, gps_vel_noise_std^2, ...
          gps_pitch_noise_std^2]);
H = eye(5);

% History logging
x_hat_hist = zeros(N, 5);

%% 4. EKF Main Loop
for k = 2:N
    % --- I. Prediction Step ---
    % Get previous state
    p_e_prev = x_hat(1); p_d_prev = x_hat(2);
    v_e_prev = x_hat(3); v_d_prev = x_hat(4);
    theta_prev = x_hat(5);
    
    % Get IMU measurements
    a_x_meas = accel_data_x(k);
    w_meas = gyro_data(k); % This has bias! We are not estimating it yet.
    
    % Predict next state using nonlinear model
    x_hat(1) = p_e_prev + v_e_prev * dt;
    x_hat(2) = p_d_prev + v_d_prev * dt;
    x_hat(3) = v_e_prev + (a_x_meas * cos(theta_prev)) * dt;
    x_hat(4) = v_d_prev + (a_x_meas * sin(theta_prev) + g) * dt;
    x_hat(5) = theta_prev + w_meas * dt;
    
    % Calculate Jacobian (F) of the process model
    F = eye(5);
    F(1, 3) = dt;
    F(2, 4) = dt;
    F(3, 5) = -a_x_meas * sin(theta_prev) * dt;
    F(4, 5) =  a_x_meas * cos(theta_prev) * dt;
    
    % Propagate covariance
    P = F * P * F' + Q;
    
    % --- II. Correction Step ---
    if ~isnan(gps_pos_e_data(k))
        % Form measurement vector
        z = [gps_pos_e_data(k); gps_pos_d_data(k); ...
             gps_vel_e_data(k); gps_vel_d_data(k); ...
             gps_pitch_data(k)];
        
        % Calculate innovation (y), covariance (S), and Kalman Gain (K)
        y = z - H * x_hat;
        S = H * P * H' + R;
        K = P * H' / S;
        
        % Update state and covariance
        x_hat = x_hat + K * y;
        P = (eye(5) - K * H) * P;
    end
    
    x_hat_hist(k, :) = x_hat';
end

%% 5. Plot Results
figure('Name', 'EKF State Estimation');
% Position East
subplot(3,2,1); hold on; grid on;
plot(t, true_pos_e, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,1), 'b', 'LineWidth', 1.5);
plot(t, gps_pos_e_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF', 'GPS'); title('East Position');

% Position Down
subplot(3,2,2); hold on; grid on;
plot(t, true_pos_d, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,2), 'b', 'LineWidth', 1.5);
plot(t, gps_pos_d_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF', 'GPS'); title('Down Position');

% Velocity East
subplot(3,2,3); hold on; grid on;
plot(t, true_vel_e, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,3), 'b', 'LineWidth', 1.5);
plot(t, gps_vel_e_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF', 'GPS'); title('East Velocity');

% Velocity Down
subplot(3,2,4); hold on; grid on;
plot(t, true_vel_d, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,4), 'b', 'LineWidth', 1.5);
plot(t, gps_vel_d_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF', 'GPS'); title('Down Velocity');

% Pitch Angle
subplot(3,2,5); hold on; grid on;
plot(t, rad2deg(true_pitch), 'g', 'LineWidth', 2);
plot(t, rad2deg(x_hat_hist(:,5)), 'b', 'LineWidth', 1.5);
plot(t, rad2deg(gps_pitch_data), 'ro', 'MarkerSize', 4);
legend('True', 'EKF', 'GPS'); title('Pitch Angle (deg)');

