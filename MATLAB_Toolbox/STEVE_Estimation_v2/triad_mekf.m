%% MEKF Attitude Estimation with TRIAD Corrections
% Multiplicative Extended Kalman Filter using gyroscopes for propagation
% and TRIAD algorithm for measurement updates
clear; clc; close all;

%% Load Data
load("data.mat");
if isstruct(steve_sensor_data), data = steve_sensor_data.data; else, data = steve_sensor_data; end
g0 = 9.80665;

%% MEKF Initialization
% State: [attitude_error(3x1); gyro_bias(3x1)]
x = zeros(6, 1);
P = eye(6);
P(1:3, 1:3) = deg2rad(10)^2 * eye(3);  % Initial attitude uncertainty
P(4:6, 4:6) = deg2rad(0.5)^2 * eye(3); % Initial bias uncertainty

% Process noise
sigma_gyro_noise = deg2rad(0.1);
sigma_bias_walk = deg2rad(0.001);

% Measurement noise (adaptive based on acceleration magnitude)
sigma_triad_low = deg2rad(1.0);   % Good acceleration
sigma_triad_high = deg2rad(10.0); % Poor acceleration
accel_threshold = 20.0; % m/s^2

% Initialize quaternion from first TRIAD solution
q_est = [1; 0; 0; 0]; % Will be set from first sample
bias_est = zeros(3, 1);

% GPS update rate parameters
gps_hz = 20;
imu_hz = 1000;
gps_update_interval = imu_hz / gps_hz; % Update every 50 samples
last_gps_index = 1; % Track last GPS sample used

%% Pre-allocate Storage
n_samples = size(data, 1) - 1;
time_s = zeros(n_samples, 1);
true_roll = zeros(n_samples, 1);
true_pitch = zeros(n_samples, 1);
true_yaw = zeros(n_samples, 1);
mekf_roll = zeros(n_samples, 1);
mekf_pitch = zeros(n_samples, 1);
mekf_yaw = zeros(n_samples, 1);
triad_roll = zeros(n_samples, 1);
triad_pitch = zeros(n_samples, 1);
triad_yaw = zeros(n_samples, 1);
error_angle_mekf = zeros(n_samples, 1);
error_angle_triad = zeros(n_samples, 1);
bias_x = zeros(n_samples, 1);
bias_y = zeros(n_samples, 1);
bias_z = zeros(n_samples, 1);
accel_mag = zeros(n_samples, 1);
gps_update_flags = false(n_samples, 1); % Track when GPS updates occur

%% Helper Functions
% Quaternion multiplication
quatmult = @(q1, q2) [q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
                      q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
                      q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
                      q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1)];

% Quaternion conjugate
quatconj = @(q) [q(1); -q(2); -q(3); -q(4)];

% Small angle to quaternion
delta_theta_to_quat = @(dtheta) [1; 0.5*dtheta(1); 0.5*dtheta(2); 0.5*dtheta(3)];

% Skew-symmetric matrix
skew = @(w) [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

%% Main Loop
fprintf('Processing %d samples with MEKF (IMU @ 1kHz, GPS @ 20Hz)...\n', n_samples);
for k = 2:n_samples+1
    index = k;
    
    % Get data
    line1 = data(index-1, :);
    line2 = data(index, :);
    dt = (line2.time_ms - line1.time_ms) / 1000;
    if dt == 0, dt = 0.001; end
    
    time_s(k-1) = line2.time_ms / 1000;
    
    % Gyro measurements (rad/s)
    omega_meas = [line2.gyro_x; line2.gyro_y; line2.gyro_z];
    
    %% PREDICTION STEP (Gyro Propagation at 1 kHz)
    
    % Bias-corrected angular velocity
    omega_corrected = omega_meas - bias_est;
    
    % Propagate quaternion
    omega_norm = norm(omega_corrected);
    if omega_norm > 1e-8
        omega_quat = [0; omega_corrected];
        q_est = quatmult(q_est, delta_theta_to_quat(omega_corrected * dt));
        q_est = q_est / norm(q_est); % Normalize
    end
    
    % State transition matrix
    F = zeros(6, 6);
    F(1:3, 1:3) = -skew(omega_corrected);
    F(1:3, 4:6) = -eye(3);
    F_discrete = eye(6) + F * dt;
    
    % Process noise covariance
    Q = zeros(6, 6);
    Q(1:3, 1:3) = sigma_gyro_noise^2 * eye(3) * dt^2;
    Q(4:6, 4:6) = sigma_bias_walk^2 * eye(3) * dt;
    
    % Propagate covariance
    P = F_discrete * P * F_discrete' + Q;
    
    %% MEASUREMENT UPDATE (TRIAD Correction at 20 Hz only)
    
    % Check if GPS update is available
    gps_update = mod(k-2, gps_update_interval) == 0;
    gps_update_flags(k-1) = gps_update;
    
    if gps_update
        % Calculate dt for GPS velocity differentiation
        gps_line1 = data(last_gps_index, :);
        gps_line2 = data(index, :);
        dt_gps = (gps_line2.time_ms - gps_line1.time_ms) / 1000;
        
        if dt_gps > 0
            % Calculate inertial acceleration from GPS velocity
            v_ref_1 = [gps_line1.gps_Ve_North, gps_line1.gps_Ve_East, gps_line1.gps_Ve_Down]';
            v_ref_2 = [gps_line2.gps_Ve_North, gps_line2.gps_Ve_East, gps_line2.gps_Ve_Down]';
            a_inertial_NED = (v_ref_2 - v_ref_1) / dt_gps;
            accel_mag(k-1) = norm(a_inertial_NED);
            
            % Construct TRIAD vectors
            a_gravity_NED = [0; 0; g0];
            ref_vector_1_NED = a_inertial_NED - a_gravity_NED;
            ref_vector_2_NED = [0; 27; 0];
            
            obs_vector_1_body = -[line2.accel_x; line2.accel_y; line2.accel_z];
            obs_vector_2_body = [line2.mag_x; line2.mag_y; line2.mag_z];
            
            % TRIAD solution
            B = obs_vector_1_body * ref_vector_1_NED' + obs_vector_2_body * ref_vector_2_NED';
            [U, ~, V] = svd(B);
            M = diag([1, 1, det(U) * det(V)]);
            R_triad = V * M * U';
            q_triad = rotm2quat(R_triad)';
            if q_triad(1) < 0, q_triad = -q_triad; end
            
            % Compute attitude error measurement
            q_err = quatmult(quatconj(q_est), q_triad);
            if q_err(1) < 0, q_err = -q_err; end
            
            % Small angle approximation
            z = 2 * q_err(2:4);
            
            % Adaptive measurement noise
            if accel_mag(k-1) > accel_threshold
                R_meas = sigma_triad_low^2 * eye(3);
            else
                R_meas = sigma_triad_high^2 * eye(3);
            end
            
            % Measurement matrix
            H = [eye(3), zeros(3, 3)];
            
            % Kalman gain
            S = H * P * H' + R_meas;
            K = P * H' / S;
            
            % State update
            x = K * z;
            
            % Apply attitude correction
            delta_q = delta_theta_to_quat(x(1:3));
            q_est = quatmult(q_est, delta_q);
            q_est = q_est / norm(q_est);
            
            % Update bias estimate
            bias_est = bias_est + x(4:6);
            
            % Reset error state
            x(1:3) = zeros(3, 1);
            
            % Covariance update
            P = (eye(6) - K * H) * P;
            
            % Update last GPS index
            last_gps_index = index;
            
            % Store TRIAD estimate
            triad_eul = rad2deg(rotm2eul(R_triad, 'ZYX'));
            triad_roll(k-1) = triad_eul(3);
            triad_pitch(k-1) = triad_eul(2);
            triad_yaw(k-1) = triad_eul(1);
        end
    else
        % No GPS update - propagate accel_mag from previous
        if k > 2
            accel_mag(k-1) = accel_mag(k-2);
        end
        
        % Propagate TRIAD values
        if k > 2
            triad_roll(k-1) = triad_roll(k-2);
            triad_pitch(k-1) = triad_pitch(k-2);
            triad_yaw(k-1) = triad_yaw(k-2);
        end
    end
    
    %% Store Results
    
    % Ground truth
    true_quat = [line2.ref_qw, line2.ref_qx, line2.ref_qy, line2.ref_qz];
    true_eul = rad2deg(quat2eul(true_quat, 'ZYX'));
    true_roll(k-1) = true_eul(3);
    true_pitch(k-1) = true_eul(2);
    true_yaw(k-1) = true_eul(1);
    
    % MEKF estimate
    mekf_eul = rad2deg(quat2eul([q_est(1), q_est(2), q_est(3), q_est(4)], 'ZYX'));
    mekf_roll(k-1) = mekf_eul(3);
    mekf_pitch(k-1) = mekf_eul(2);
    mekf_yaw(k-1) = mekf_eul(1);
    
    % Errors
    q_err_mekf = quatmultiply(true_quat, quatinv([q_est(1), q_est(2), q_est(3), q_est(4)]));
    error_angle_rad = 2 * acos(abs(q_err_mekf(1)));
    error_angle_mekf(k-1) = rad2deg(error_angle_rad);
    
    if gps_update && exist('R_triad', 'var')
        q_err_triad = quatmultiply(true_quat, quatinv(rotm2quat(R_triad)));
        error_angle_rad = 2 * acos(abs(q_err_triad(1)));
        if error_angle_rad > 0
            error_angle_triad(k-1) = rad2deg(error_angle_rad);
        else
            error_angle_triad(k-1) = error_angle_triad(k-2);
        end
    elseif k > 2
        error_angle_triad(k-1) = error_angle_triad(k-2);
    end
    
    % Bias estimates
    bias_x(k-1) = rad2deg(bias_est(1));
    bias_y(k-1) = rad2deg(bias_est(2));
    bias_z(k-1) = rad2deg(bias_est(3));
    
    if mod(k, 10000) == 0
        fprintf('Processed %d/%d samples (%.1f%%) - GPS updates: %d\n', ...
            k-1, n_samples, 100*(k-1)/n_samples, sum(gps_update_flags(1:k-1)));
    end
end
fprintf('Processing complete!\n');
fprintf('Total GPS updates: %d (expected: ~%d)\n\n', sum(gps_update_flags), floor(n_samples/gps_update_interval));

%% Plotting
figure('Position', [100, 100, 1400, 900]);

% Roll
subplot(5,2,1);
plot(time_s, true_roll, 'k-', 'LineWidth', 1.5); hold on;
plot(time_s, mekf_roll, 'b-', 'LineWidth', 1);
plot(time_s(gps_update_flags), triad_roll(gps_update_flags), 'r.', 'MarkerSize', 4);
ylabel('Roll (deg)');
legend('Ground Truth', 'MEKF (1kHz)', 'TRIAD (20Hz)', 'Location', 'best');
grid on;
title('Attitude Estimation: MEKF vs TRIAD');

% Pitch
subplot(5,2,3);
plot(time_s, true_pitch, 'k-', 'LineWidth', 1.5); hold on;
plot(time_s, mekf_pitch, 'b-', 'LineWidth', 1);
plot(time_s(gps_update_flags), triad_pitch(gps_update_flags), 'r.', 'MarkerSize', 4);
ylabel('Pitch (deg)');
legend('Ground Truth', 'MEKF (1kHz)', 'TRIAD (20Hz)', 'Location', 'best');
grid on;

% Yaw
subplot(5,2,5);
plot(time_s, true_yaw, 'k-', 'LineWidth', 1.5); hold on;
plot(time_s, mekf_yaw, 'b-', 'LineWidth', 1);
plot(time_s(gps_update_flags), triad_yaw(gps_update_flags), 'r.', 'MarkerSize', 4);
ylabel('Yaw (deg)');
legend('Ground Truth', 'MEKF (1kHz)', 'TRIAD (20Hz)', 'Location', 'best');
grid on;

% Total error
subplot(5,2,7);
plot(time_s, error_angle_mekf, 'b-', 'LineWidth', 1); hold on;
plot(time_s(gps_update_flags), error_angle_triad(gps_update_flags), 'r.', 'MarkerSize', 4);
ylabel('Total Error (deg)');
legend('MEKF', 'TRIAD', 'Location', 'best');
grid on;

% Acceleration magnitude
subplot(5,2,9);
plot(time_s, accel_mag, 'k-', 'LineWidth', 1);
ylabel('Accel Mag (m/s^2)');
xlabel('Time (s)');
grid on;
yline(accel_threshold, 'r--', 'Threshold');

% Gyro biases
subplot(5,2,[2,4]);
plot(time_s, bias_x, 'r-', 'LineWidth', 1); hold on;
plot(time_s, bias_y, 'g-', 'LineWidth', 1);
plot(time_s, bias_z, 'b-', 'LineWidth', 1);
ylabel('Gyro Bias (deg/s)');
legend('X-axis', 'Y-axis', 'Z-axis', 'Location', 'best');
grid on;
title('Estimated Gyro Biases');

% Error comparison zoom (0-80s)
subplot(5,2,[6,8]);
idx_zoom = time_s >= 0 & time_s <= 80;
plot(time_s(idx_zoom), error_angle_mekf(idx_zoom), 'b-', 'LineWidth', 1.5); hold on;
idx_zoom_gps = idx_zoom & gps_update_flags;
plot(time_s(idx_zoom_gps), error_angle_triad(idx_zoom_gps), 'r.', 'MarkerSize', 6);
ylabel('Total Error (deg)');
xlabel('Time (s)');
legend('MEKF', 'TRIAD', 'Location', 'best');
grid on;
title('Error Zoom: 0-80 seconds');

% GPS update indicator
subplot(5,2,10);
plot(time_s, double(gps_update_flags), 'k-', 'LineWidth', 1);
ylabel('GPS Update');
xlabel('Time (s)');
ylim([-0.1, 1.1]);
grid on;
title(sprintf('GPS Update Rate: %d Hz', gps_hz));

%% Statistics
fprintf('=== MEKF ATTITUDE ESTIMATION STATISTICS ===\n');
fprintf('Roll  - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(abs(mekf_roll - true_roll)), std(mekf_roll - true_roll), max(abs(mekf_roll - true_roll)));
fprintf('Pitch - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(abs(mekf_pitch - true_pitch)), std(mekf_pitch - true_pitch), max(abs(mekf_pitch - true_pitch)));
fprintf('Yaw   - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(abs(mekf_yaw - true_yaw)), std(mekf_yaw - true_yaw), max(abs(mekf_yaw - true_yaw)));
fprintf('Total - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n\n', ...
    mean(error_angle_mekf), std(error_angle_mekf), max(error_angle_mekf));

fprintf('=== TRIAD ATTITUDE ESTIMATION STATISTICS (20 Hz samples) ===\n');
triad_indices = find(gps_update_flags);
fprintf('Roll  - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(abs(triad_roll(triad_indices) - true_roll(triad_indices))), ...
    std(triad_roll(triad_indices) - true_roll(triad_indices)), ...
    max(abs(triad_roll(triad_indices) - true_roll(triad_indices))));
fprintf('Pitch - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(abs(triad_pitch(triad_indices) - true_pitch(triad_indices))), ...
    std(triad_pitch(triad_indices) - true_pitch(triad_indices)), ...
    max(abs(triad_pitch(triad_indices) - true_pitch(triad_indices))));
fprintf('Yaw   - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(abs(triad_yaw(triad_indices) - true_yaw(triad_indices))), ...
    std(triad_yaw(triad_indices) - true_yaw(triad_indices)), ...
    max(abs(triad_yaw(triad_indices) - true_yaw(triad_indices))));
fprintf('Total - Mean Error: %.4f deg, Std: %.4f deg, Max: %.4f deg\n', ...
    mean(error_angle_triad(triad_indices)), std(error_angle_triad(triad_indices)), ...
    max(error_angle_triad(triad_indices)));

fprintf('\n=== FINAL GYRO BIAS ESTIMATES ===\n');
fprintf('X-axis: %.4f deg/s\n', bias_x(end));
fprintf('Y-axis: %.4f deg/s\n', bias_y(end));
fprintf('Z-axis: %.4f deg/s\n', bias_z(end));
