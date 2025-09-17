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
true_pitch_rate(t > 40 & t <= 50) = deg2rad(2);
true_pitch_rate(t > 50 & t <= 80) = deg2rad(-1);
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

gyro_noise_std  = deg2rad(0.1);
gyro_bias_true  = deg2rad(0.1);   % Constant bias
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

%% 3. GPS Sensor (N, D)
gps_rate = 20; % Hz
gps_noise_multiplier = 10;

gps_pos_noise_std = 2.5*gps_noise_multiplier; % m (horizontal)
gps_alt_noise_std = 3*gps_noise_multiplier;   % m (vertical)
gps_vel_noise_std = 0.1*gps_noise_multiplier; % m/s
gps_pos_n_data = nan(N, 1); gps_pos_d_data = nan(N, 1);
gps_vel_n_data = nan(N, 1); gps_vel_d_data = nan(N, 1);

for k = 1:N
    if mod(k - 1, round(1/dt / gps_rate)) == 0
        gps_pos_n_data(k) = true_pos_n(k) + randn() * gps_pos_noise_std;
        gps_pos_d_data(k) = true_pos_d(k) + randn() * gps_alt_noise_std;
        gps_vel_n_data(k) = true_vel_n(k) + randn() * gps_vel_noise_std;
        gps_vel_d_data(k) = true_vel_d(k) + randn() * gps_vel_noise_std;
    end
end

%% 4. EKF Implementation (Fully Integrated)
% State: [p_n, p_d, v_n, v_d, theta, b_w, b_ax, b_az]'
x_hat = zeros(8, 1);
P     = eye(8) * 1;
P(6,6) = (deg2rad(0.5))^2; % Initial gyro bias uncertainty
P(7,7) = (0.5)^2;          % Initial accel bias uncertainty
P(8,8) = (0.5)^2;

% Process noise parameters - these are your tuning knobs
sigma_a_proc  = 0.5;                 % accel process noise (m/s^2)
sigma_w_proc  = deg2rad(0.2);        % gyro process noise (rad/s)
sigma_bw_proc = deg2rad(0.001);      % gyro bias random walk (rad/s/sqrt(Hz))
sigma_ba_proc = 0.005;               % accel bias random walk (m/s^2/sqrt(Hz))

% Process noise covariance Q
Q = zeros(8);
Q(1,1) = (dt^4/4)*sigma_a_proc^2; Q(1,3) = (dt^3/2)*sigma_a_proc^2;
Q(3,1) = (dt^3/2)*sigma_a_proc^2; Q(3,3) = dt^2*sigma_a_proc^2;
Q(2,2) = (dt^4/4)*sigma_a_proc^2; Q(2,4) = (dt^3/2)*sigma_a_proc^2;
Q(4,2) = (dt^3/2)*sigma_a_proc^2; Q(4,4) = dt^2*sigma_a_proc^2;
Q(5,5) = (dt * sigma_w_proc)^2;
Q(6,6) = (dt * sigma_bw_proc)^2;
Q(7,7) = (dt * sigma_ba_proc)^2;
Q(8,8) = (dt * sigma_ba_proc)^2;

% Measurement model R and H
R = diag([gps_pos_noise_std^2, gps_alt_noise_std^2, ...
          gps_vel_noise_std^2, gps_vel_noise_std^2]);
H = [eye(4), zeros(4, 4)];

% History logging
x_hat_hist = zeros(N, 8);
P_full_hist = zeros(N, 8, 8); % Store full P for NEES
NIS_hist = nan(N, 1);
I8 = eye(8);

% True state vector for NEES calculation
x_true = [true_pos_n, true_pos_d, true_vel_n, true_vel_d, ...
          true_pitch, ones(N,1)*gyro_bias_true, ...
          ones(N,1)*accel_bias_true, ones(N,1)*accel_bias_true];

for k = 1:N
    % --- Prediction ---
    % Unpack state
    p_n=x_hat(1); p_d=x_hat(2); v_n=x_hat(3); v_d=x_hat(4);
    theta=x_hat(5); b_w=x_hat(6); b_ax=x_hat(7); b_az=x_hat(8);
    
    % Bias-corrected IMU
    axb = accel_data_x(k) - b_ax;
    azb = accel_data_z(k) - b_az;
    qw  = gyro_data(k)    - b_w;
    
    st = sin(theta); ct = cos(theta);
    
    % Process model: x_hat_pred = f(x_hat)
    x_hat_pred = zeros(8,1);
    x_hat_pred(1) = p_n + v_n * dt;
    x_hat_pred(2) = p_d + v_d * dt;
    x_hat_pred(3) = v_n + (axb*ct - azb*st) * dt;
    x_hat_pred(4) = v_d + (axb*st + azb*ct + g) * dt;
    x_hat_pred(5) = theta + qw * dt;
    x_hat_pred(6) = b_w; % Biases are random walks
    x_hat_pred(7) = b_ax;
    x_hat_pred(8) = b_az;
    
    % Jacobian F
    F = I8;
    F(1,3) = dt; F(2,4) = dt;
    F(3,5) = (-axb*st - azb*ct) * dt;
    F(4,5) = ( axb*ct - azb*st) * dt;
    F(3,7) = -ct*dt; F(3,8) =  st*dt;
    F(4,7) = -st*dt; F(4,8) = -ct*dt;
    F(5,6) = -dt;
    
    P_pred = F * P * F' + Q;
    
    % --- Correction (GPS available) ---
    x_hat = x_hat_pred; % Assume no update first
    P = P_pred;
    
    if ~isnan(gps_pos_n_data(k))
        z = [gps_pos_n_data(k); gps_pos_d_data(k); gps_vel_n_data(k); gps_vel_d_data(k)];
        y = z - H * x_hat_pred;
        S = H * P_pred * H' + R;
        K = P_pred * H' / S;
        x_hat = x_hat_pred + K * y;
        A = (I8 - K*H);
        P = A * P_pred * A' + K * R * K';
        
        NIS_hist(k) = y' / S * y; % Log NIS
    end
    
    P = 0.5 * (P + P');
    
    % Log history
    x_hat_hist(k, :) = x_hat';
    P_full_hist(k, :, :) = P;
end

%% 5. Performance Analysis
fprintf('--- EKF Performance Analysis ---\n');

% NEES Calculation
NEES_hist = zeros(N, 1);
for k = 1:N
    err = x_hat_hist(k,:)' - x_true(k,:)';
    P_k = squeeze(P_full_hist(k,:,:));
    if rcond(P_k) < 1e-12, NEES_hist(k) = NaN; continue; end
    NEES_hist(k) = err' / P_k * err;
end
avg_NEES = mean(NEES_hist, 'omitnan');
dof_nees = 8;
alpha = 0.05;
nees_ci_lower = chi2inv(alpha/2, dof_nees);
nees_ci_upper = chi2inv(1 - alpha/2, dof_nees);

fprintf('\nNormalized Estimation Error Squared (NEES) Test:\n');
fprintf('Degrees of Freedom: %d\n', dof_nees);
fprintf('Average NEES: %.4f (Expected: %.4f)\n', avg_NEES, dof_nees);
fprintf('95%% Confidence Interval: [%.4f, %.4f]\n', nees_ci_lower, nees_ci_upper);

% NIS Calculation
NIS_clean = NIS_hist(~isnan(NIS_hist));
avg_NIS = mean(NIS_clean);
dof_nis = 4;
nis_ci_lower = chi2inv(alpha/2, dof_nis);
nis_ci_upper = chi2inv(1 - alpha/2, dof_nis);

fprintf('\nNormalized Innovation Squared (NIS) Test:\n');
fprintf('Degrees of Freedom: %d\n', dof_nis);
fprintf('Average NIS: %.4f (Expected: %.4f)\n', avg_NIS, dof_nis);
fprintf('95%% Confidence Interval: [%.4f, %.4f]\n', nis_ci_lower, nis_ci_upper);

%% 6. Plot Results
lw = 1.5;
% Figure 1: Position and Velocity Errors
figure('Name', 'State Errors');
subplot(2,2,1); hold on; grid on; plot(t, x_hat_hist(:,1) - x_true(:,1)); title('North Position Error'); ylabel('[m]');
subplot(2,2,2); hold on; grid on; plot(t, x_hat_hist(:,2) - x_true(:,2)); title('Down Position Error'); ylabel('[m]');
subplot(2,2,3); hold on; grid on; plot(t, x_hat_hist(:,3) - x_true(:,3)); title('North Velocity Error'); ylabel('[m/s]');
subplot(2,2,4); hold on; grid on; plot(t, x_hat_hist(:,4) - x_true(:,4)); title('Down Velocity Error'); ylabel('[m/s]');

% Figure 2: Attitude and Bias Errors
figure('Name', 'Attitude and Bias Errors');
subplot(3,1,1); hold on; grid on; plot(t, rad2deg(x_hat_hist(:,5) - x_true(:,5))); title('Pitch Angle Error'); ylabel('[deg]');
subplot(3,1,2); hold on; grid on; plot(t, rad2deg(x_hat_hist(:,6) - x_true(:,6))); title('Gyro Bias Error'); ylabel('[deg/s]');
subplot(3,1,3); hold on; grid on; plot(t, x_hat_hist(:,7) - x_true(:,7)); title('Accel X Bias Error'); ylabel('[m/s^2]');

% Figure 3: Bias Estimation
figure('Name','Sensor Bias Estimation');
subplot(3,1,1); hold on; grid on;
plot(t, rad2deg(x_hat_hist(:,6)), 'b', 'LineWidth', lw);
plot(t, rad2deg(gyro_bias_true)*ones(N,1), 'r--', 'LineWidth', lw);
legend('EKF Estimate','True Bias'); title('Gyroscope Bias (deg/s)');
subplot(3,1,2); hold on; grid on;
plot(t, x_hat_hist(:,7), 'b', 'LineWidth', lw);
plot(t, accel_bias_true*ones(N,1), 'r--', 'LineWidth', lw);
legend('EKF Estimate','True Bias'); title('Accelerometer X Bias (m/s^2)');
subplot(3,1,3); hold on; grid on;
plot(t, x_hat_hist(:,8), 'b', 'LineWidth', lw);
plot(t, accel_bias_true*ones(N,1), 'r--', 'LineWidth', lw);
legend('EKF Estimate','True Bias'); title('Accelerometer Z Bias (m/s^2)');

% Figure 4: NEES and NIS Plots
figure('Name', 'Consistency Checks');
subplot(2,1,1); hold on; grid on;
plot(t, NEES_hist, '.');
line(xlim, [nees_ci_lower nees_ci_lower], 'Color', 'r', 'LineStyle', '--');
line(xlim, [nees_ci_upper nees_ci_upper], 'Color', 'r', 'LineStyle', '--');
title(sprintf('NEES (Average = %.2f)', avg_NEES)); ylabel('NEES');
subplot(2,1,2); hold on; grid on;
plot(t, NIS_hist, '.');
line(xlim, [nis_ci_lower nis_ci_lower], 'Color', 'r', 'LineStyle', '--');
line(xlim, [nis_ci_upper nis_ci_upper], 'Color', 'r', 'LineStyle', '--');
title(sprintf('NIS (Average = %.2f)', avg_NIS)); ylabel('NIS');
xlabel('Time (s)');
