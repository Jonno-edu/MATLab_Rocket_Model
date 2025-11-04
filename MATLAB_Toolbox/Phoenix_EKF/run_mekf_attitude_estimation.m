% run_MEKF_mag_aided_v2.m
%
% Magnetometer-aided MEKF for spinning rocket attitude estimation.
% This version includes:
% - Correct magnetometer noise calculation from PSD.
% - Logging and plotting of the filter innovation for diagnostics.
% - Full plotting and statistical analysis suite.
%
% Compares MEKF (with magnetometer) vs Dead Reckoning (gyro-only).
% Uses adaptive measurement noise for high spin rates.

clear;
clc;
close all;

% --- 1. Load Sensor Data ---
% This assumes 'phoenix_sensor_sim_output' is loaded into the workspace
% and has the required columns (Time_s, Gyro_*, Mag_*, Ref_Att_*, New_Mag_Data_Flag)
import_sensor_data;
sensor_data = phoenix_sensor_sim_output;

% --- 2. Define Constants and Tuning Parameters ---
% Geomagnetic Field Vector (NED frame, for your location)
mag_ref_NED = [27.5550; -2.4169; -16.0849]; % microTesla
mag_ref_norm = mag_ref_NED / norm(mag_ref_NED); % Normalize

% Sampling
dt_gyro = 0.001; % 1000 Hz gyro (from simulation)
dt_mag = 0.01;   % 100 Hz magnetometer updates

% Number of samples
N = height(sensor_data);

% Warmup period for bias estimation
warmup_time = 30; % seconds
warmup_idx = find(sensor_data.Time_s > warmup_time, 1);

% --- 3. Process and Measurement Noise Tuning ---
% Gyro noise parameters
gyro_noise_std = 0.001; % rad/s (angle random walk)
Q_gyro = (gyro_noise_std^2 * dt_gyro) * eye(3); % Attitude process noise

% Gyro bias process noise - VERY LOW for short flight after warmup
bias_drift_std = 1e-5; % rad/s^2 (increase if bias needs to be more dynamic)
Q_bias = (bias_drift_std^2 * dt_gyro) * eye(3);

% Combined process noise for the error-state vector [att_error; bias_error]
Q = blkdiag(Q_gyro, Q_bias);

% Magnetometer measurement noise (from sensor datasheet)
mag_psd = 0.001265; % (µT)/√Hz
mag_sampling_rate = 1/dt_mag; % 100 Hz
mag_noise_std = mag_psd * sqrt(mag_sampling_rate); % Correct calculation
fprintf('Calculated Magnetometer Noise Std Dev: %.6f uT\n', mag_noise_std);
R_mag_base = (mag_noise_std^2) * eye(3);

% Adaptive magnetometer noise parameters for high spin
spin_threshold = 10; % rad/s - when to start increasing R
max_spin_rate = 30; % rad/s - maximum expected spin
max_noise_multiplier = 100; % Maximum R increase factor

% --- 4. Initialize State ---
% Initial orientation: [roll, pitch, yaw] = [0, 80, 0] degrees
roll_init = 0 * pi/180;
pitch_init = 80 * pi/180;
yaw_init = 0 * pi/180;

% Convert Euler angles to quaternion (ZYX convention)
q_mekf = euler_to_quat(roll_init, pitch_init, yaw_init);
q_dr = q_mekf; % Dead reckoning starts with same initial quaternion

fprintf('Initial quaternion from [%.1f, %.1f, %.1f] deg: [%.4f, %.4f, %.4f, %.4f]\n', ...
        roll_init*180/pi, pitch_init*180/pi, yaw_init*180/pi, ...
        q_mekf(1), q_mekf(2), q_mekf(3), q_mekf(4));

% Initial bias estimate (from warmup)
bias_init = zeros(3, 1);
if warmup_idx > 10
    gyro_warmup = [sensor_data.Gyro_X_rad_s(1:warmup_idx), ...
                   sensor_data.Gyro_Y_rad_s(1:warmup_idx), ...
                   sensor_data.Gyro_Z_rad_s(1:warmup_idx)];
    bias_init = mean(gyro_warmup, 1)';
    fprintf('Warmup complete. Bias estimate: [%.6f, %.6f, %.6f] rad/s\n', ...
            bias_init(1), bias_init(2), bias_init(3));
end
bias_mekf = bias_init;
bias_dr = bias_init;

% Initial covariance (only for MEKF)
P = blkdiag((1*pi/180)^2 * eye(3), (0.001)^2 * eye(3)); % [att_err_cov; bias_err_cov]

% --- 5. Storage for Results ---
q_mekf_history = zeros(N, 4);
bias_mekf_history = zeros(N, 3);
euler_mekf_history = zeros(N, 3);
euler_error_mekf_history = zeros(N, 3);
spin_rate_history = zeros(N, 1);
R_multiplier_history = zeros(N, 1);
innovation_history = zeros(N, 3); % For innovation logging
q_dr_history = zeros(N, 4);
euler_dr_history = zeros(N, 3);
euler_error_dr_history = zeros(N, 3);

% --- 6. Main Filter Loop ---
fprintf('Running MEKF with adaptive magnetometer noise and Dead Reckoning comparison...\n');
for k = 1:N
    t = sensor_data.Time_s(k);
    
    % Get sensor measurements and reference attitude
    gyro = [sensor_data.Gyro_X_rad_s(k); sensor_data.Gyro_Y_rad_s(k); sensor_data.Gyro_Z_rad_s(k)];
    mag_meas = [sensor_data.Mag_X_uT(k); sensor_data.Mag_Y_uT(k); sensor_data.Mag_Z_uT(k)];
    mag_meas_norm = mag_meas / norm(mag_meas);
    q_ref = euler_to_quat(sensor_data.Ref_Att_Roll_deg(k)*pi/180, sensor_data.Ref_Att_Pitch_deg(k)*pi/180, sensor_data.Ref_Att_Yaw_deg(k)*pi/180);
    
    % ===== DEAD RECKONING (Gyro Only) =====
    omega_dr = gyro - bias_dr;
    q_dr = quaternion_propagate(q_dr, omega_dr, dt_gyro);
    
    % ===== MEKF (Gyro + Magnetometer) =====
    % --- PREDICTION STEP ---
    omega_mekf = gyro - bias_mekf;
    q_mekf = quaternion_propagate(q_mekf, omega_mekf, dt_gyro);
    spin_rate = norm(omega_mekf);
    spin_rate_history(k) = spin_rate;
    Omega_skew = skew_symmetric(omega_mekf);
    F = [-Omega_skew, -eye(3); zeros(3,3), zeros(3,3)];
    Phi = eye(6) + F * dt_gyro;
    P = Phi * P * Phi' + Q;
    
    % --- ADAPTIVE MEASUREMENT NOISE ---
    if spin_rate > spin_threshold
        spin_factor = max(0, min(1, (spin_rate - spin_threshold) / (max_spin_rate - spin_threshold)));
        noise_multiplier = 1 + (max_noise_multiplier - 1) * spin_factor^2;
        R_mag_adaptive = R_mag_base * noise_multiplier;
    else
        noise_multiplier = 1.0;
        R_mag_adaptive = R_mag_base;
    end
    R_multiplier_history(k) = noise_multiplier;
    
    % --- UPDATE STEP (Magnetometer) ---
    if sensor_data.New_Mag_Data_Flag(k) && k > warmup_idx
        C_bn = quat_to_dcm(q_mekf);
        mag_pred = C_bn' * mag_ref_norm;
        y = mag_meas_norm - mag_pred;
        innovation_history(k, :) = y';
        H = [skew_symmetric(mag_pred), zeros(3,3)];
        S = H * P * H' + R_mag_adaptive;
        K = P * H' / S;
        delta_x = K * y;
        att_error = delta_x(1:3);
        bias_update = delta_x(4:6);
        delta_q = axis_angle_to_quat(att_error);
        q_mekf = quaternion_multiply(q_mekf, delta_q);
        q_mekf = q_mekf / norm(q_mekf);
        bias_mekf = bias_mekf + bias_update;
        I_KH = eye(6) - K * H;
        P = I_KH * P * I_KH' + K * R_mag_adaptive * K';
    else
        innovation_history(k, :) = NaN;
    end
    
    % --- Calculate Errors and Store Results ---
    q_error_mekf = quaternion_multiply(q_ref, quaternion_conjugate(q_mekf));
    euler_error_mekf_history(k, :) = quat_to_euler(q_error_mekf)' * 180/pi;
    q_error_dr = quaternion_multiply(q_ref, quaternion_conjugate(q_dr));
    euler_error_dr_history(k, :) = quat_to_euler(q_error_dr)' * 180/pi;
    q_mekf_history(k,:)=q_mekf'; bias_mekf_history(k,:)=bias_mekf'; euler_mekf_history(k,:)=quat_to_euler(q_mekf)'*180/pi;
    q_dr_history(k,:)=q_dr'; euler_dr_history(k,:)=quat_to_euler(q_dr)'*180/pi;
    
    if mod(k, 5000) == 0; fprintf('Progress: %.1f%%\n', 100*k/N); end
end
fprintf('MEKF and Dead Reckoning complete!\n');

% --- 7. Plot and Analyze Results ---
% Plot 1: Attitude Comparison
figure('Name', 'Attitude Estimation Comparison');
subplot(3,1,1);
plot(sensor_data.Time_s, sensor_data.Ref_Att_Roll_deg, 'r', 'LineWidth', 1.5); hold on;
plot(sensor_data.Time_s, euler_mekf_history(:,1), 'b', 'LineWidth', 1.2);
plot(sensor_data.Time_s, euler_dr_history(:,1), 'g--', 'LineWidth', 1.2);
ylabel('Roll (deg)'); legend('Reference', 'MEKF', 'Dead Reckoning', 'Location', 'best'); grid on;
title('Attitude Estimation: MEKF vs Dead Reckoning');
subplot(3,1,2);
plot(sensor_data.Time_s, sensor_data.Ref_Att_Pitch_deg, 'r', 'LineWidth', 1.5); hold on;
plot(sensor_data.Time_s, euler_mekf_history(:,2), 'b', 'LineWidth', 1.2);
plot(sensor_data.Time_s, euler_dr_history(:,2), 'g--', 'LineWidth', 1.2);
ylabel('Pitch (deg)'); legend('Reference', 'MEKF', 'Dead Reckoning', 'Location', 'best'); grid on;
subplot(3,1,3);
plot(sensor_data.Time_s, sensor_data.Ref_Att_Yaw_deg, 'r', 'LineWidth', 1.5); hold on;
plot(sensor_data.Time_s, euler_mekf_history(:,3), 'b', 'LineWidth', 1.2);
plot(sensor_data.Time_s, euler_dr_history(:,3), 'g--', 'LineWidth', 1.2);
ylabel('Yaw (deg)'); xlabel('Time (s)'); legend('Reference', 'MEKF', 'Dead Reckoning', 'Location', 'best'); grid on;

% Plot 2: Attitude Errors Comparison
figure('Name', 'Attitude Estimation Errors');
subplot(3,1,1);
plot(sensor_data.Time_s, euler_error_mekf_history(:,1), 'b'); hold on;
plot(sensor_data.Time_s, euler_error_dr_history(:,1), 'g--');
ylabel('Roll Error (deg)'); legend('MEKF', 'Dead Reckoning', 'Location', 'best'); grid on;
title('Attitude Estimation Errors: MEKF vs Dead Reckoning');
subplot(3,1,2);
plot(sensor_data.Time_s, euler_error_mekf_history(:,2), 'b'); hold on;
plot(sensor_data.Time_s, euler_error_dr_history(:,2), 'g--');
ylabel('Pitch Error (deg)'); legend('MEKF', 'Dead Reckoning', 'Location', 'best'); grid on;
subplot(3,1,3);
plot(sensor_data.Time_s, euler_error_mekf_history(:,3), 'b'); hold on;
plot(sensor_data.Time_s, euler_error_dr_history(:,3), 'g--');
ylabel('Yaw Error (deg)'); xlabel('Time (s)'); legend('MEKF', 'Dead Reckoning', 'Location', 'best'); grid on;

% Plot 3: Total Attitude Error Magnitude
figure('Name', 'Total Attitude Error');
total_error_mekf = sqrt(sum(euler_error_mekf_history.^2, 2));
total_error_dr = sqrt(sum(euler_error_dr_history.^2, 2));
plot(sensor_data.Time_s, total_error_mekf, 'b', 'LineWidth', 1.5); hold on;
plot(sensor_data.Time_s, total_error_dr, 'g--', 'LineWidth', 1.5);
ylabel('Total Attitude Error (deg)'); xlabel('Time (s)');
legend('MEKF', 'Dead Reckoning', 'Location', 'best'); grid on;
title('Total Attitude Error Magnitude: MEKF vs Dead Reckoning');

% Plot 4: Spin Rate and Adaptive R Multiplier
figure('Name', 'Adaptive Noise');
subplot(2,1,1);
plot(sensor_data.Time_s, spin_rate_history, 'LineWidth', 1.5); hold on;
yline(spin_threshold, 'r--', 'LineWidth', 1.5); yline(max_spin_rate, 'r:', 'LineWidth', 1.5);
ylabel('Spin Rate (rad/s)'); legend('Angular Velocity', 'Adaptation Threshold', 'Max Spin', 'Location', 'best'); grid on;
title('Spin Rate and Adaptive Noise');
subplot(2,1,2);
semilogy(sensor_data.Time_s, R_multiplier_history, 'LineWidth', 1.5);
ylabel('R Noise Multiplier'); xlabel('Time (s)'); grid on;
title('Adaptive Magnetometer Measurement Noise Multiplier');

% Plot 5: Gyro Bias Estimates
figure('Name', 'Gyro Bias Estimates');
plot(sensor_data.Time_s, bias_mekf_history * 180/pi, 'LineWidth', 1.5);
ylabel('Bias (deg/s)'); xlabel('Time (s)');
legend('X-axis', 'Y-axis', 'Z-axis', 'Location', 'best'); grid on;
title('Gyro Bias Estimation Over Time (MEKF)');

% NEW PLOT: Magnetometer Innovation
figure('Name', 'Magnetometer Innovation Analysis');
plot(sensor_data.Time_s, innovation_history, 'LineWidth', 1.2);
title('Magnetometer Innovation (Measurement - Prediction)');
xlabel('Time (s)'); ylabel('Innovation (unitless vector components)');
legend('Innovation X', 'Innovation Y', 'Innovation Z', 'Location', 'best'); grid on;

% --- 8. Print Final Statistics ---
fprintf('\n==========================================\n');
fprintf('    FINAL STATISTICS (Post-Warmup)\n');
fprintf('==========================================\n');
post_warmup = sensor_data.Time_s > warmup_time;
fprintf('\n*** MEKF (Gyro + Magnetometer) Errors ***\n');
fprintf('Roll RMS Error:   %.4f deg\n', rms(euler_error_mekf_history(post_warmup, 1)));
fprintf('Pitch RMS Error:  %.4f deg\n', rms(euler_error_mekf_history(post_warmup, 2)));
fprintf('Yaw RMS Error:    %.4f deg\n', rms(euler_error_mekf_history(post_warmup, 3)));
fprintf('Total RMS Error:  %.4f deg\n', rms(total_error_mekf(post_warmup)));
fprintf('\n*** Dead Reckoning (Gyro Only) Errors ***\n');
fprintf('Roll RMS Error:   %.4f deg\n', rms(euler_error_dr_history(post_warmup, 1)));
fprintf('Pitch RMS Error:  %.4f deg\n', rms(euler_error_dr_history(post_warmup, 2)));
fprintf('Yaw RMS Error:    %.4f deg\n', rms(euler_error_dr_history(post_warmup, 3)));
fprintf('Total RMS Error:  %.4f deg\n', rms(total_error_dr(post_warmup)));
fprintf('\n==========================================\n');

% ========== HELPER FUNCTIONS ==========
function q = euler_to_quat(roll, pitch, yaw)
    cr=cos(roll/2); sr=sin(roll/2); cp=cos(pitch/2); sp=sin(pitch/2); cy=cos(yaw/2); sy=sin(yaw/2);
    w=cr*cp*cy+sr*sp*sy; x=sr*cp*cy-cr*sp*sy; y=cr*sp*cy+sr*cp*sy; z=cr*cp*sy-sr*sp*cy;
    q = [w; x; y; z]; q = q / norm(q);
end
function q_new = quaternion_propagate(q, omega, dt)
    omega_norm = norm(omega);
    if omega_norm < 1e-12, q_new = q; return; end
    theta = omega_norm * dt; u = omega / omega_norm;
    delta_q = [cos(theta/2); u * sin(theta/2)];
    q_new = quaternion_multiply(q, delta_q); q_new = q_new / norm(q_new);
end
function q_prod = quaternion_multiply(q1, q2)
    w1=q1(1); v1=q1(2:4); w2=q2(1); v2=q2(2:4);
    w = w1*w2 - dot(v1, v2); v = w1*v2 + w2*v1 + cross(v1, v2);
    q_prod = [w; v];
end
function q_conj = quaternion_conjugate(q)
    q_conj = [q(1); -q(2:4)];
end
function C = quat_to_dcm(q)
    w=q(1); x=q(2); y=q(3); z=q(4);
    C=[1-2*(y^2+z^2), 2*(x*y-w*z), 2*(x*z+w*y); 2*(x*y+w*z), 1-2*(x^2+z^2), 2*(y*z-w*x); 2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x^2+y^2)];
end
function euler = quat_to_euler(q)
    w=q(1); x=q(2); y=q(3); z=q(4);
    roll = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));
    pitch = asin(max(-1, min(1, 2*(w*y - z*x))));
    yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2));
    euler = [roll; pitch; yaw];
end
function q = axis_angle_to_quat(aa)
    angle = norm(aa);
    if angle < 1e-12, q = [1; 0; 0; 0]; return; end
    axis = aa / angle;
    q = [cos(angle/2); axis * sin(angle/2)];
end
function S = skew_symmetric(v)
    S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end
