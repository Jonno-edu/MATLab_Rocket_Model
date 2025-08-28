clc; clear; close all;

%% 1. Simulation Setup
% Constants & Time
g = 9.81;              % m/s^2
dt = 0.01;             % 100 Hz
t_pad = 100;
t_end = 100 + t_pad;
N = t_end / dt;
t = (0:N-1)' * dt;

% True Vehicle Motion Profile
true_pitch_rate = zeros(N, 1);
true_pitch_rate(t > 20 + t_pad & t <= 50 + t_pad) = deg2rad(2);
true_pitch_rate(t > 50 + t_pad & t <= 80 + t_pad) = deg2rad(-1);
true_pitch = cumsum(true_pitch_rate) * dt;

% True total inertial acceleration 
true_accel_e = zeros(N, 1); 
true_accel_d = zeros(N, 1);
true_accel_e(t > 15 & t <= 50 + t_pad) = 5;
true_accel_e(t > 70 + t_pad & t <= 80 + t_pad) = -2;
true_accel_d(t > 15 + t_pad & t <= 30 + t_pad) = -9.81*7;
true_accel_d(t > 30 + t_pad & t <= 100 + t_pad) = 9.81;
true_vel_e = cumsum(true_accel_e) * dt;
true_vel_d = cumsum(true_accel_d) * dt;
true_pos_e = cumsum(true_vel_e) * dt;
true_pos_d = cumsum(true_vel_d) * dt;


%% 2. Generate Physically Consistent Sensor Data with BIAS
% IMU Noise & Bias Levels
noise_multiplier = 1;
bias_multiplier = 10;
gyro_noise_std  = deg2rad(0.5*noise_multiplier); 
gyro_bias_true  = deg2rad(0.2*bias_multiplier);   % Constant bias
accel_noise_std = 0.05*noise_multiplier;
accel_bias_true = 0.02*bias_multiplier;           % Constant bias

% Generate IMU Data
perfect_accel_data_x =  true_accel_e.*cos(true_pitch) + true_accel_d.*sin(true_pitch) - g.*sin(true_pitch);
perfect_accel_data_z = -true_accel_e.*sin(true_pitch) + true_accel_d.*cos(true_pitch) - g.*cos(true_pitch);
perfect_gyro_data    = true_pitch_rate;

accel_data_x = perfect_accel_data_x + randn(N, 1) * accel_noise_std + accel_bias_true;
accel_data_z = perfect_accel_data_z + randn(N, 1) * accel_noise_std + accel_bias_true;
gyro_data    = perfect_gyro_data    + randn(N, 1) * gyro_noise_std    + gyro_bias_true;

% GPS Sensor
gps_rate = 1; % Hz
gps_pos_noise_std = 2.5*noise_multiplier; % m (horizontal)
gps_alt_noise_std = 3*noise_multiplier;   % m (vertical)
gps_vel_noise_std = 0.1*noise_multiplier; % m/s

gps_pos_e_data = nan(N, 1); gps_pos_d_data = nan(N, 1);
gps_vel_e_data = nan(N, 1); gps_vel_d_data = nan(N, 1);
for k = 1:N
    if mod(k - 1, round(1/dt / gps_rate)) == 0
        gps_pos_e_data(k) = true_pos_e(k) + randn() * gps_pos_noise_std;
        gps_pos_d_data(k) = true_pos_d(k) + randn() * gps_alt_noise_std;
        gps_vel_e_data(k) = true_vel_e(k) + randn() * gps_vel_noise_std;
        gps_vel_d_data(k) = true_vel_d(k) + randn() * gps_vel_noise_std;
    end
end

%% 3. EKF Implementation (Fully Integrated)
% State: [p_e, p_d, v_e, v_d, theta, b_w, b_ax, b_az]'

% Initial state & covariance
x_hat = zeros(8, 1);
P     = eye(8) * 1;
P(7,7) = (0.5)^2;  % larger initial var for b_ax to adapt faster
P(8,8) = (0.5)^2;

% Process noise parameters
sigma_a_proc  = 0.2;                 % accel process noise (m/s^2)
sigma_w_proc  = deg2rad(0.1);        % gyro process noise (rad/s)
sigma_bw_proc = deg2rad(0.001);      % gyro bias RW (rad/s^2)
sigma_ba_proc = 0.005;               % accel bias RW (m/s^3)

% Build Q with per-axis [pos, vel] constant-accel blocks
Q = zeros(8);
Q_pv = [[dt^4/4, dt^3/2]; [dt^3/2, dt^2]] * sigma_a_proc^2;
Q([1 3],[1 3]) = Q_pv;   % East: [p_e, v_e]
Q([2 4],[2 4]) = Q_pv;   % Down: [p_d, v_d]
Q(5,5) = (dt * sigma_w_proc)^2;
Q(6,6) = (dt * sigma_bw_proc)^2;
Q(7,7) = (dt * sigma_ba_proc)^2;
Q(8,8) = (dt * sigma_ba_proc)^2;

% Measurement model (GPS pos, vel)
R = diag([gps_pos_noise_std^2, gps_alt_noise_std^2, ...
          gps_vel_noise_std^2, gps_vel_noise_std^2]);
H = [eye(4), zeros(4, 4)];

% History
x_hat_hist = zeros(N, 8);
P_hist     = zeros(N, 8);

I8 = eye(8);

for k = 1:N
    % --- Prediction ---
    % Unpack
    p_e = x_hat(1); p_d = x_hat(2);
    v_e = x_hat(3); v_d = x_hat(4);
    theta = x_hat(5);
    b_w = x_hat(6); b_ax = x_hat(7); b_az = x_hat(8);

    % Bias-corrected IMU
    axb = accel_data_x(k) - b_ax;
    azb = accel_data_z(k) - b_az;
    qw  = gyro_data(k)     - b_w;

    % Precompute
    st = sin(theta); ct = cos(theta);

    % Process model
    x_hat(1) = p_e + v_e * dt;
    x_hat(2) = p_d + v_d * dt;
    x_hat(3) = v_e + (axb*ct - azb*st) * dt;
    x_hat(4) = v_d + (axb*st + azb*ct + g) * dt;
    x_hat(5) = theta + qw * dt;
    % Bias states are random-walk (no deterministic drift)

    % Jacobian F
    F = I8;
    F(1,3) = dt; F(2,4) = dt;
    F(3,5) = (-axb*st - azb*ct) * dt;
    F(4,5) = ( axb*ct - azb*st) * dt;
    F(3,7) = -ct*dt; F(3,8) =  st*dt;
    F(4,7) = -st*dt; F(4,8) = -ct*dt;
    F(5,6) = -dt;

    % Covariance predict
    P = F * P * F' + Q;

    % --- Correction (GPS available) ---
    if ~isnan(gps_pos_e_data(k))
        z = [gps_pos_e_data(k);
             gps_pos_d_data(k);
             gps_vel_e_data(k);
             gps_vel_d_data(k)];

        y = z - H * x_hat;
        S = H * P * H' + R;
        K = P * H' / S;

        % State update
        x_hat = x_hat + K * y;

        % Joseph-form covariance update (stable)
        A = (I8 - K*H);
        P = A * P * A' + K * R * K';
        P = 0.5 * (P + P');  % enforce symmetry
    end

    % Log
    x_hat_hist(k, :) = x_hat';
    P_hist(k, :)     = diag(P)';
end

%% 6. Plot Results
% Figure 1: East Position
figure('Name', 'East Position Analysis');
subplot(2,1,1);
hold on; grid on;
plot(t, true_pos_e, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,1), 'b--');
plot(t, gps_pos_e_data, 'ro', 'MarkerSize', 4); 
legend('True', 'EKF Estimate', 'GPS Measurement');
title('East Position Comparison');
ylabel('Position (m)');
subplot(2,1,2);
hold on; grid on;
plot(t, x_hat_hist(:,1) - true_pos_e, 'r');
plot(t, gps_pos_e_data - true_pos_e, 'mo', 'MarkerSize', 4);
legend('EKF Error', 'GPS Error');
title('East Position Error');
ylabel('Error (m)'); xlabel('Time (s)');

% Figure 2: Down Position
figure('Name', 'Down Position Analysis');
subplot(2,1,1);
hold on; grid on;
plot(t, true_pos_d, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,2), 'b--');
plot(t, gps_pos_d_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF Estimate', 'GPS Measurement');
title('Down Position Comparison');
ylabel('Position (m)');
subplot(2,1,2);
hold on; grid on;
plot(t, x_hat_hist(:,2) - true_pos_d, 'r');
plot(t, gps_pos_d_data - true_pos_d, 'mo', 'MarkerSize', 4);
legend('EKF Error', 'GPS Error');
title('Down Position Error');
ylabel('Error (m)'); xlabel('Time (s)');

% Figure 3: East Velocity
figure('Name', 'East Velocity Analysis');
subplot(2,1,1);
hold on; grid on;
plot(t, true_vel_e, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,3), 'b--');
plot(t, gps_vel_e_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF Estimate', 'GPS Measurement');
title('East Velocity Comparison');
ylabel('Velocity (m/s)');
subplot(2,1,2);
hold on; grid on;
plot(t, x_hat_hist(:,3) - true_vel_e, 'r');
plot(t, gps_vel_e_data - true_vel_e, 'mo', 'MarkerSize', 4);
legend('EKF Error', 'GPS Error');
title('East Velocity Error');
ylabel('Error (m/s)'); xlabel('Time (s)');

% Figure 4: Down Velocity
figure('Name', 'Down Velocity Analysis');
subplot(2,1,1);
hold on; grid on;
plot(t, true_vel_d, 'g', 'LineWidth', 2);
plot(t, x_hat_hist(:,4), 'b--');
plot(t, gps_vel_d_data, 'ro', 'MarkerSize', 4);
legend('True', 'EKF Estimate', 'GPS Measurement');
title('Down Velocity Comparison');
ylabel('Velocity (m/s)');
subplot(2,1,2);
hold on; grid on;
plot(t, x_hat_hist(:,4) - true_vel_d, 'r');
plot(t, gps_vel_d_data - true_vel_d, 'mo', 'MarkerSize', 4);
legend('EKF Error', 'GPS Error');
title('Down Velocity Error');
ylabel('Error (m/s)'); xlabel('Time (s)');

% Figure 5: Pitch Angle
figure('Name', 'Pitch Angle Analysis');
subplot(2,1,1);
hold on; grid on;
plot(t, rad2deg(true_pitch), 'g', 'LineWidth', 2);
plot(t, rad2deg(x_hat_hist(:,5)), 'b--');
legend('True', 'EKF Estimate');
title('Pitch Angle Comparison');
ylabel('Angle (deg)');
subplot(2,1,2);
hold on; grid on;
plot(t, rad2deg(x_hat_hist(:,5) - true_pitch), 'r');
legend('EKF Error');
title('Pitch Angle Error');
ylabel('Error (deg)'); xlabel('Time (s)');

% Figure 6: Sensor Bias Estimation
figure('Name','Sensor Bias Estimation');
subplot(3,1,1); hold on; grid on;
plot(t, rad2deg(x_hat_hist(:,6)), 'b', 'LineWidth', 1.5);
plot(t, rad2deg(gyro_bias_true)*ones(N,1), 'r--', 'LineWidth', 1.5);
legend('EKF Estimate','True Bias');
title('Gyroscope Bias (deg/s)');
ylabel('Bias (deg/s)');
subplot(3,1,2); hold on; grid on;
plot(t, x_hat_hist(:,7), 'b', 'LineWidth', 1.5);
plot(t, accel_bias_true*ones(N,1), 'r--', 'LineWidth', 1.5);
legend('EKF Estimate','True Bias');
title('Accelerometer X Bias (m/s^2)');
ylabel('Bias (m/s^2)');
subplot(3,1,3); hold on; grid on;
plot(t, x_hat_hist(:,8), 'b', 'LineWidth', 1.5);
plot(t, accel_bias_true*ones(N,1), 'r--', 'LineWidth', 1.5);
legend('EKF Estimate','True Bias');
title('Accelerometer Z Bias (m/s^2)');
ylabel('Bias (m/s^2)'); xlabel('Time (s)');
